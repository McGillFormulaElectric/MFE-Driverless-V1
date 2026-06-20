"""
Gymnasium environment wrapping the MFE Gazebo/ROS2 simulation.

Observation space: 42-dim float32 vector
  - 10 nearest left  cones in car frame: (dx, dy) x 10 → 20 dims
  - 10 nearest right cones in car frame: (dx, dy) x 10 → 20 dims
  - car speed (m/s)  → 1 dim
  - yaw rate (rad/s) → 1 dim

Action space: Box(-1, 1, (2,))
  - action[0]: steering_norm  → ControlCommand.steering  ∈ [-1, 1]
  - action[1]: throttle_brake_norm → positive: throttle, negative: abs → brake
"""

import math
import threading
import time

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mfe_msgs.msg import Track, Cone
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

# CarState comes from eufs_msgs — only used inside the env (not the deploy node)
try:
    from eufs_msgs.msg import CarState
    _CARSTATE_AVAILABLE = True
except ImportError:
    _CARSTATE_AVAILABLE = False
    CarState = None


class MFEDrivingEnv(gym.Env):
    """Gym environment for the MFE Formula Student driverless car."""

    metadata = {'render_modes': []}

    def __init__(self):
        super().__init__()

        # --- Spaces ---
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(42,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

        # --- Shared state (protected by lock) ---
        self._lock = threading.Lock()
        self._cones = None        # mfe_msgs/Track
        self._car_state = None    # eufs_msgs/CarState (or None)
        self._done = False
        self._accuracy = 0.0
        self._cone_hit = False
        self._prev_x = 0.0

        # --- ROS2 setup ---
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('mfe_rl_env')

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._node.create_subscription(
            Track, '/planning/cones', self._cones_callback, best_effort_qos
        )

        if _CARSTATE_AVAILABLE:
            self._node.create_subscription(
                CarState, '/ground_truth/state', self._car_state_callback, best_effort_qos
            )
        else:
            self._node.get_logger().warn(
                'eufs_msgs not available — CarState subscription skipped.'
            )

        self._node.create_subscription(
            Bool, '/planning/mission_finished', self._done_callback, reliable_qos
        )
        self._node.create_subscription(
            Float32, '/perception/accuracy', self._accuracy_callback, best_effort_qos
        )

        self._cmd_pub = self._node.create_publisher(
            ControlCommand, '/control/command', reliable_qos
        )

        self._reset_client = self._node.create_client(SetBool, '/mfe_rl/reset_episode')

        # --- Background spin thread ---
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()

        self._node.get_logger().info('MFEDrivingEnv initialised.')

    # ------------------------------------------------------------------
    # Callbacks (called from spin thread — acquire lock)
    # ------------------------------------------------------------------

    def _cones_callback(self, msg: Track):
        with self._lock:
            self._cones = msg
            self._cone_hit = self._detect_cone_hit_locked()

    def _car_state_callback(self, msg):
        with self._lock:
            self._car_state = msg

    def _done_callback(self, msg: Bool):
        with self._lock:
            if msg.data:
                self._done = True

    def _accuracy_callback(self, msg: Float32):
        with self._lock:
            self._accuracy = msg.data

    # ------------------------------------------------------------------
    # Observation
    # ------------------------------------------------------------------

    def _get_obs(self) -> np.ndarray:
        """Build the 42-dim observation vector."""
        with self._lock:
            cones = self._cones
            state = self._car_state

        # Default zeroed observation if data not yet available
        if state is None:
            return np.zeros(42, dtype=np.float32)

        car_x = state.pose.pose.position.x
        car_y = state.pose.pose.position.y
        yaw = _quaternion_to_yaw(state.pose.pose.orientation)

        vx = state.twist.twist.linear.x
        vy = state.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        yaw_rate = state.twist.twist.angular.z

        left_cones = []   # list of (dx_car, dy_car, dist)
        right_cones = []

        if cones is not None:
            cos_neg_yaw = math.cos(-yaw)
            sin_neg_yaw = math.sin(-yaw)

            for cone in cones.track:
                cx = cone.location.x
                cy = cone.location.y
                dx_world = cx - car_x
                dy_world = cy - car_y
                dx_car = dx_world * cos_neg_yaw - dy_world * sin_neg_yaw
                dy_car = dx_world * sin_neg_yaw + dy_world * cos_neg_yaw
                dist = math.hypot(dx_car, dy_car)

                if cone.color == Cone.BLUE:      # left boundary
                    left_cones.append((dx_car, dy_car, dist))
                elif cone.color == Cone.YELLOW:  # right boundary
                    right_cones.append((dx_car, dy_car, dist))

        # Sort by distance, keep 10 nearest, zero-pad
        left_cones.sort(key=lambda t: t[2])
        right_cones.sort(key=lambda t: t[2])

        def _pad(cone_list, n=10):
            arr = np.zeros((n, 2), dtype=np.float32)
            for i, (dx, dy, _) in enumerate(cone_list[:n]):
                arr[i, 0] = dx
                arr[i, 1] = dy
            return arr.flatten()

        obs = np.concatenate([
            _pad(left_cones),    # 20 dims
            _pad(right_cones),   # 20 dims
            [speed],             # 1 dim
            [yaw_rate],          # 1 dim
        ]).astype(np.float32)

        return obs

    # ------------------------------------------------------------------
    # Gym interface
    # ------------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Stop the car
        self._publish_command(0.0, 0.0, 0.0)

        # Request episode reset from the simulation
        if self._reset_client.service_is_ready():
            req = SetBool.Request()
            req.data = True
            future = self._reset_client.call_async(req)
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if not future.done():
                self._node.get_logger().warn(
                    '/mfe_rl/reset_episode service call timed out.'
                )
        else:
            self._node.get_logger().warn(
                '/mfe_rl/reset_episode service not available — skipping reset call.'
            )

        # Wait for sim to settle
        time.sleep(1.0)

        with self._lock:
            self._done = False
            self._cone_hit = False
            state = self._car_state

        if state is not None:
            self._prev_x = state.pose.pose.position.x
        else:
            self._prev_x = 0.0

        obs = self._get_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        # --- Publish control command ---
        steering = float(action[0])
        tbn = float(action[1])
        throttle = max(0.0, tbn)
        brake = max(0.0, -tbn)
        self._publish_command(steering, throttle, brake)

        # --- Wait one control cycle (20 Hz) ---
        time.sleep(0.05)

        # --- Read current state ---
        with self._lock:
            state = self._car_state
            cone_hit = self._cone_hit
            mission_done = self._done
            cones = self._cones

        car_x = state.pose.pose.position.x if state is not None else self._prev_x
        vx = state.twist.twist.linear.x if state is not None else 0.0
        vy = state.twist.twist.linear.y if state is not None else 0.0
        car_speed = math.hypot(vx, vy)

        # --- Reward computation ---
        forward_progress = car_x - self._prev_x
        reward = 0.0
        terminated = False

        if forward_progress > 0:
            reward += forward_progress * 2.0

        reward += car_speed * 0.1

        if cone_hit:
            reward -= 50.0
            terminated = True

        if not terminated and _off_track(cones, state):
            reward -= 20.0
            terminated = True

        if mission_done:
            terminated = True

        self._prev_x = car_x

        obs = self._get_obs()
        return obs, reward, terminated, False, {}

    def close(self):
        self._node.destroy_node()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _detect_cone_hit_locked(self) -> bool:
        """Return True if any cone is within 0.4 m of the car. Call with lock held."""
        state = self._car_state
        cones = self._cones
        if state is None or cones is None:
            return False
        car_x = state.pose.pose.position.x
        car_y = state.pose.pose.position.y
        for cone in cones.track:
            dx = cone.location.x - car_x
            dy = cone.location.y - car_y
            if dx * dx + dy * dy < 0.4 * 0.4:
                return True
        return False

    def _publish_command(self, steering: float, throttle: float, brake: float):
        cmd = ControlCommand()
        cmd.steering = float(steering)
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        self._cmd_pub.publish(cmd)


# ------------------------------------------------------------------
# Module-level helpers
# ------------------------------------------------------------------

def _quaternion_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _off_track(cones, state) -> bool:
    """Return True if no cone is within 8 m of the car (off-track detection)."""
    if cones is None or state is None:
        return False
    car_x = state.pose.pose.position.x
    car_y = state.pose.pose.position.y
    for cone in cones.track:
        dx = cone.location.x - car_x
        dy = cone.location.y - car_y
        if dx * dx + dy * dy < 8.0 * 8.0:
            return False
    return True
