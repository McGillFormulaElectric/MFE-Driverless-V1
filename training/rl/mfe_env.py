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

Reward shaping:
  + forward_progress * 2.0      encourage making forward progress
  + speed * 0.05                encourage going fast (small weight)
  - boundary_excess * 5.0       continuous penalty for lateral deviation past 70% of track half-width
  - 100.0 + terminate           cone hit (detected against full GT map, not visibility window)
  - 30.0  + terminate           off-track (no cones of either color within 12 m)
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

# Standard FSAE cone base radius ≈ 0.114 m; car half-width ≈ 0.575 m (track_width/2)
# A hit occurs when car centre is within ~0.4 m of a cone centre.
_CONE_HIT_RADIUS = 0.40   # m

# CarState from eufs_msgs — only used inside the env (not the deploy node)
try:
    from eufs_msgs.msg import CarState
    _CARSTATE_AVAILABLE = True
except ImportError:
    _CARSTATE_AVAILABLE = False
    CarState = None


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _nearest_cone(cones_xy: np.ndarray, car_x: float, car_y: float):
    """Return (dx, dy, dist) of the nearest cone, or None if array is empty."""
    if len(cones_xy) == 0:
        return None
    diff = cones_xy - np.array([car_x, car_y])
    dists = np.hypot(diff[:, 0], diff[:, 1])
    i = int(np.argmin(dists))
    return diff[i, 0], diff[i, 1], float(dists[i])


class MFEDrivingEnv(gym.Env):
    """Gym environment for the MFE Formula Student driverless car."""

    metadata = {'render_modes': []}

    def __init__(self):
        super().__init__()

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(42,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

        # Shared state protected by lock
        self._lock = threading.Lock()
        self._planning_cones = None   # mfe_msgs/Track — for observations only
        self._gt_cones_xy = {         # full static map from /ground_truth/track_colored
            Cone.BLUE:   np.zeros((0, 2), dtype=np.float32),
            Cone.YELLOW: np.zeros((0, 2), dtype=np.float32),
        }
        self._car_state = None        # eufs_msgs/CarState
        self._mission_done = False
        self._prev_x = 0.0

        # ROS2 setup
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('mfe_rl_env')
        reliable_qos   = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # /planning/cones — visibility-filtered, used only for the observation vector
        self._node.create_subscription(
            Track, '/planning/cones', self._planning_cones_cb, best_effort_qos
        )

        # /ground_truth/track_colored — FULL static cone map, used for hit detection and boundary
        self._node.create_subscription(
            Track, '/ground_truth/track_colored', self._gt_cones_cb, reliable_qos
        )

        if _CARSTATE_AVAILABLE:
            self._node.create_subscription(
                CarState, '/ground_truth/state', self._car_state_cb, best_effort_qos
            )
        else:
            self._node.get_logger().warn(
                'eufs_msgs not available — /ground_truth/state subscription skipped.'
            )

        self._node.create_subscription(
            Bool, '/planning/mission_finished', self._mission_done_cb, reliable_qos
        )

        self._cmd_pub = self._node.create_publisher(
            ControlCommand, '/control/command', reliable_qos
        )
        self._reset_client = self._node.create_client(SetBool, '/mfe_rl/reset_episode')

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()

        self._node.get_logger().info('MFEDrivingEnv initialised.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _planning_cones_cb(self, msg: Track):
        with self._lock:
            self._planning_cones = msg

    def _gt_cones_cb(self, msg: Track):
        """Cache the full GT cone map split by color."""
        blues, yellows = [], []
        for c in msg.track:
            if c.color == Cone.BLUE:
                blues.append([c.location.x, c.location.y])
            elif c.color == Cone.YELLOW:
                yellows.append([c.location.x, c.location.y])
        with self._lock:
            self._gt_cones_xy[Cone.BLUE]   = np.array(blues,   dtype=np.float32) if blues   else np.zeros((0, 2), dtype=np.float32)
            self._gt_cones_xy[Cone.YELLOW] = np.array(yellows, dtype=np.float32) if yellows else np.zeros((0, 2), dtype=np.float32)

    def _car_state_cb(self, msg):
        with self._lock:
            self._car_state = msg

    def _mission_done_cb(self, msg: Bool):
        with self._lock:
            if msg.data:
                self._mission_done = True

    # ------------------------------------------------------------------
    # Observation
    # ------------------------------------------------------------------

    def _get_obs(self) -> np.ndarray:
        with self._lock:
            cones = self._planning_cones
            state = self._car_state

        if state is None:
            return np.zeros(42, dtype=np.float32)

        car_x = state.pose.pose.position.x
        car_y = state.pose.pose.position.y
        yaw   = _quaternion_to_yaw(state.pose.pose.orientation)
        vx    = state.twist.twist.linear.x
        vy    = state.twist.twist.linear.y
        speed    = math.hypot(vx, vy)
        yaw_rate = state.twist.twist.angular.z

        left_cones, right_cones = [], []

        if cones is not None:
            cos_neg = math.cos(-yaw)
            sin_neg = math.sin(-yaw)
            for cone in cones.track:
                dxw = cone.location.x - car_x
                dyw = cone.location.y - car_y
                dx_car = dxw * cos_neg - dyw * sin_neg
                dy_car = dxw * sin_neg + dyw * cos_neg
                dist   = math.hypot(dx_car, dy_car)
                if cone.color == Cone.BLUE:
                    left_cones.append((dx_car, dy_car, dist))
                elif cone.color == Cone.YELLOW:
                    right_cones.append((dx_car, dy_car, dist))

        left_cones.sort(key=lambda t: t[2])
        right_cones.sort(key=lambda t: t[2])

        def _pad(lst, n=10):
            arr = np.zeros((n, 2), dtype=np.float32)
            for i, (dx, dy, _) in enumerate(lst[:n]):
                arr[i] = [dx, dy]
            return arr.flatten()

        return np.concatenate([
            _pad(left_cones), _pad(right_cones), [speed], [yaw_rate]
        ]).astype(np.float32)

    # ------------------------------------------------------------------
    # Cone hit detection — uses FULL GT cone map, not visibility window
    # ------------------------------------------------------------------

    def _check_cone_hit(self, car_x: float, car_y: float) -> bool:
        """
        True if the car centre is within _CONE_HIT_RADIUS of ANY cone in the
        full ground-truth track map (blue OR yellow).

        Uses /ground_truth/track_colored, which covers the entire track regardless
        of the car's current visibility window — so a hit is detected even if the
        cone is behind the car or outside the planner's 20 m radius.
        """
        with self._lock:
            blues   = self._gt_cones_xy[Cone.BLUE]
            yellows = self._gt_cones_xy[Cone.YELLOW]

        r2 = _CONE_HIT_RADIUS ** 2
        for cones_xy in (blues, yellows):
            if len(cones_xy) == 0:
                continue
            diff  = cones_xy - np.array([car_x, car_y], dtype=np.float32)
            dists2 = (diff ** 2).sum(axis=1)
            if float(dists2.min()) < r2:
                return True
        return False

    # ------------------------------------------------------------------
    # Boundary deviation — continuous lateral penalty
    # ------------------------------------------------------------------

    def _boundary_deviation(self, car_x: float, car_y: float) -> float:
        """
        Returns how far the car is outside 70% of the track half-width.
        Zero when the car is comfortably inside; positive when encroaching on a cone.

        Method: find nearest blue (left) and yellow (right) cone from the full GT
        map, compute the midpoint and half-width, then measure signed lateral
        distance from midpoint.
        """
        with self._lock:
            blues   = self._gt_cones_xy[Cone.BLUE]
            yellows = self._gt_cones_xy[Cone.YELLOW]

        nb = _nearest_cone(blues,   car_x, car_y)
        ny = _nearest_cone(yellows, car_x, car_y)

        if nb is None or ny is None:
            return 0.0

        # Position of nearest left/right cone
        lx = car_x + nb[0]
        ly = car_y + nb[1]
        rx = car_x + ny[0]
        ry = car_y + ny[1]

        # Track half-width and midpoint
        half_w = math.hypot(lx - rx, ly - ry) / 2.0
        if half_w < 0.1:
            return 0.0

        mx = (lx + rx) / 2.0
        my = (ly + ry) / 2.0

        # Signed lateral offset from midpoint (absolute value = deviation)
        deviation = math.hypot(car_x - mx, car_y - my)

        # Penalise only when past 70% of half-width (leaves a comfortable centre zone)
        excess = deviation - 0.70 * half_w
        return max(0.0, excess)

    # ------------------------------------------------------------------
    # Off-track check
    # ------------------------------------------------------------------

    def _is_off_track(self, car_x: float, car_y: float) -> bool:
        """
        True when the car has left the track entirely — no blue AND no yellow
        cone within 12 m.  Requires BOTH colors to be absent so the car is not
        penalised for being near the end of a straight with cones only on one side.
        """
        with self._lock:
            blues   = self._gt_cones_xy[Cone.BLUE]
            yellows = self._gt_cones_xy[Cone.YELLOW]

        def _min_dist(cones_xy):
            if len(cones_xy) == 0:
                return float('inf')
            diff = cones_xy - np.array([car_x, car_y], dtype=np.float32)
            return float(np.hypot(diff[:, 0], diff[:, 1]).min())

        return _min_dist(blues) > 12.0 and _min_dist(yellows) > 12.0

    # ------------------------------------------------------------------
    # Gym interface
    # ------------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self._publish_command(0.0, 0.0, 0.0)

        if self._reset_client.service_is_ready():
            req = SetBool.Request()
            req.data = True
            future = self._reset_client.call_async(req)
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if not future.done():
                self._node.get_logger().warn('/mfe_rl/reset_episode timed out.')
        else:
            self._node.get_logger().warn('/mfe_rl/reset_episode not available — skipping.')

        time.sleep(1.0)

        with self._lock:
            self._mission_done = False
            state = self._car_state

        self._prev_x = state.pose.pose.position.x if state is not None else 0.0
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        steering = float(action[0])
        tbn      = float(action[1])
        self._publish_command(steering, max(0.0, tbn), max(0.0, -tbn))

        time.sleep(0.05)   # 20 Hz

        with self._lock:
            state        = self._car_state
            mission_done = self._mission_done

        car_x = state.pose.pose.position.x if state is not None else self._prev_x
        car_y = state.pose.pose.position.y if state is not None else 0.0
        vx    = state.twist.twist.linear.x if state is not None else 0.0
        vy    = state.twist.twist.linear.y if state is not None else 0.0
        speed = math.hypot(vx, vy)

        # --- Reward ---
        forward_progress = car_x - self._prev_x
        reward     = 0.0
        terminated = False

        reward += max(0.0, forward_progress) * 2.0
        reward += speed * 0.05

        # Continuous boundary penalty (car drifting toward cone line)
        excess = self._boundary_deviation(car_x, car_y)
        reward -= excess * 5.0

        # Cone hit — checked against FULL GT map, always reliable
        if self._check_cone_hit(car_x, car_y):
            reward    -= 100.0
            terminated = True
            self._node.get_logger().warn(f'Cone hit at ({car_x:.1f}, {car_y:.1f}).')

        # Off-track — both boundary colors absent within 12 m
        if not terminated and self._is_off_track(car_x, car_y):
            reward    -= 30.0
            terminated = True
            self._node.get_logger().warn(f'Off-track at ({car_x:.1f}, {car_y:.1f}).')

        if mission_done:
            terminated = True

        self._prev_x = car_x
        return self._get_obs(), reward, terminated, False, {}

    def close(self):
        self._node.destroy_node()

    def _publish_command(self, steering: float, throttle: float, brake: float):
        cmd = ControlCommand()
        cmd.steering = float(steering)
        cmd.throttle = float(throttle)
        cmd.brake    = float(brake)
        self._cmd_pub.publish(cmd)
