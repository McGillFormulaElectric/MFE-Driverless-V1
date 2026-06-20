"""
RL policy deployment node for the MFE Formula Student driverless car.

Loads a trained DrivingPolicy (BC or PPO) and runs inference at 20 Hz,
publishing ControlCommand from cone + odometry observations.

Parameters:
    policy_path (string, default '~/mfe_models/rl/bc_policy.pt')
    use_ppo     (bool,   default False)  — reserved for future SB3 loading
    pose_topic  (string, default '/ekf/output')  — nav_msgs/Odometry topic

Subscribes:
    /planning/cones  (mfe_msgs/Track)
    <pose_topic>     (nav_msgs/Odometry)

Publishes:
    /control/command (fs_msgs/ControlCommand)
"""

import math
import os
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool, Header
from mfe_msgs.msg import Track, Cone

import torch

# Locate policy.py relative to this file so it works from both ros2 install
# and when run directly.
_TRAINING_RL = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', 'training', 'rl'
)
sys.path.insert(0, os.path.abspath(_TRAINING_RL))
try:
    from policy import DrivingPolicy
except ImportError:
    raise ImportError(
        'Cannot import DrivingPolicy.  Make sure training/rl/policy.py is '
        'accessible from the PYTHONPATH or run the node from the repo root.'
    )


# ---------------------------------------------------------------------------
# Helper: quaternion → yaw
# ---------------------------------------------------------------------------

def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class RLPolicyNode(Node):
    """Runs a trained DrivingPolicy at 20 Hz and publishes ControlCommand."""

    def __init__(self):
        super().__init__('rl_policy_node')

        # --- Parameters ---
        self.declare_parameter('policy_path', '~/mfe_models/rl/bc_policy.pt')
        self.declare_parameter('use_ppo',     False)
        self.declare_parameter('pose_topic',  '/ekf/output')

        policy_path = os.path.expanduser(
            self.get_parameter('policy_path').get_parameter_value().string_value
        )
        use_ppo    = self.get_parameter('use_ppo').get_parameter_value().bool_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        # --- Load policy ---
        self.get_logger().info(f'Loading policy from {policy_path}  use_ppo={use_ppo}')
        t0 = time.monotonic()
        self._policy = DrivingPolicy.load(policy_path)
        self._policy.eval()
        load_ms = (time.monotonic() - t0) * 1000.0
        self.get_logger().info(f'Policy loaded in {load_ms:.1f} ms')

        # Warm up with a dummy forward pass and measure inference speed
        dummy = torch.zeros(1, 42)
        with torch.no_grad():
            t0 = time.monotonic()
            for _ in range(100):
                self._policy(dummy)
            infer_us = (time.monotonic() - t0) / 100.0 * 1e6
        self.get_logger().info(f'Inference latency: {infer_us:.1f} µs per step')

        # --- Shared state ---
        self._lock = threading.Lock()
        self._cones = None       # mfe_msgs/Track
        self._odom = None        # nav_msgs/Odometry
        self._mission_done = False

        # --- QoS ---
        reliable_qos    = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # --- Subscriptions ---
        self.create_subscription(
            Track, '/planning/cones', self._cones_cb, best_effort_qos
        )
        self.create_subscription(
            Odometry, pose_topic, self._odom_cb, best_effort_qos
        )
        self.create_subscription(
            Bool, '/planning/mission_finished', self._done_cb, reliable_qos
        )

        # --- Publisher ---
        self._cmd_pub = self.create_publisher(ControlCommand, '/control/command', reliable_qos)

        # --- 20 Hz control loop ---
        self.create_timer(1.0 / 20.0, self._control_loop)

        self.get_logger().info(
            f'rl_policy_node ready — listening to cones on /planning/cones '
            f'and odometry on {pose_topic}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cones_cb(self, msg: Track):
        with self._lock:
            self._cones = msg

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def _done_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Mission finished — stopping RL policy node.')
            with self._lock:
                self._mission_done = True

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        with self._lock:
            if self._mission_done:
                return
            odom = self._odom
            cones = self._cones

        if odom is None:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return

        obs = self._build_obs(cones, odom)

        with torch.no_grad():
            obs_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
            action = self._policy(obs_t).squeeze(0).numpy()

        steering = float(action[0])
        tbn = float(action[1])
        throttle = max(0.0, tbn)
        brake = max(0.0, -tbn)

        self._publish_command(steering, throttle, brake)

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------

    def _build_obs(self, cones, odom: Odometry) -> np.ndarray:
        """Build the 42-dim observation from Odometry and cone Track."""
        car_x = odom.pose.pose.position.x
        car_y = odom.pose.pose.position.y
        yaw = _quaternion_to_yaw(odom.pose.pose.orientation)

        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        yaw_rate = odom.twist.twist.angular.z

        left_cones = []
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

                if cone.color == Cone.BLUE:
                    left_cones.append((dx_car, dy_car, dist))
                elif cone.color == Cone.YELLOW:
                    right_cones.append((dx_car, dy_car, dist))

        left_cones.sort(key=lambda t: t[2])
        right_cones.sort(key=lambda t: t[2])

        def _pad(cone_list, n=10):
            arr = np.zeros((n, 2), dtype=np.float32)
            for i, (dx, dy, _) in enumerate(cone_list[:n]):
                arr[i, 0] = dx
                arr[i, 1] = dy
            return arr.flatten()

        obs = np.concatenate([
            _pad(left_cones),
            _pad(right_cones),
            [speed],
            [yaw_rate],
        ]).astype(np.float32)

        return obs

    # ------------------------------------------------------------------
    # Publisher
    # ------------------------------------------------------------------

    def _publish_command(self, steering: float, throttle: float, brake: float):
        cmd = ControlCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.steering  = float(steering)
        cmd.throttle  = float(throttle)
        cmd.brake     = float(brake)
        self._cmd_pub.publish(cmd)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
