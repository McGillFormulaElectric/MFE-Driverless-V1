"""
Expert demonstration collector for behavioural cloning.

Listens to the pure pursuit controller driving in Gazebo and records
(observation, action) pairs.  Observations are built using the same 42-dim
representation as MFEDrivingEnv._get_obs().

Usage:
    # In one terminal: launch the sim with pure pursuit
    # In another terminal:
    python3 training/rl/collect_demos.py

Saves ~/mfe_models/rl/demos.npz every 30 s and on Ctrl+C.
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

from mfe_msgs.msg import Track, Cone
from fs_msgs.msg import ControlCommand

# CarState from eufs_msgs for ground-truth pose
try:
    from eufs_msgs.msg import CarState
    _CARSTATE_AVAILABLE = True
except ImportError:
    _CARSTATE_AVAILABLE = False
    CarState = None

# ---------------------------------------------------------------------------
# Output path
# ---------------------------------------------------------------------------
SAVE_DIR = os.path.expanduser('~/mfe_models/rl')
SAVE_PATH = os.path.join(SAVE_DIR, 'demos.npz')

SAVE_INTERVAL = 30.0   # seconds between auto-saves
PRINT_INTERVAL = 5.0   # seconds between stats printouts


# ---------------------------------------------------------------------------
# Helpers (shared with mfe_env.py)
# ---------------------------------------------------------------------------

def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _build_obs(cones, state) -> np.ndarray:
    """Build the 42-dim observation from current cone and state messages."""
    if state is None:
        return np.zeros(42, dtype=np.float32)

    car_x = state.pose.pose.position.x
    car_y = state.pose.pose.position.y
    yaw = _quaternion_to_yaw(state.pose.pose.orientation)

    vx = state.twist.twist.linear.x
    vy = state.twist.twist.linear.y
    speed = math.hypot(vx, vy)
    yaw_rate = state.twist.twist.angular.z

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


# ---------------------------------------------------------------------------
# Collector node
# ---------------------------------------------------------------------------

class DemoCollectorNode(Node):
    """ROS2 node that records (obs, action) pairs from the expert policy."""

    def __init__(self):
        super().__init__('demo_collector_node')

        self._lock = threading.Lock()
        self._cones = None
        self._car_state = None

        self._obs_list = []
        self._act_list = []

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Track, '/planning/cones', self._cones_cb, best_effort_qos
        )

        if _CARSTATE_AVAILABLE:
            self.create_subscription(
                CarState, '/ground_truth/state', self._state_cb, best_effort_qos
            )
        else:
            self.get_logger().warn(
                'eufs_msgs/CarState not available — using zero state for obs.'
            )

        # Record every time pure pursuit publishes a command
        self.create_subscription(
            ControlCommand, '/control/command', self._command_cb, reliable_qos
        )

        self.get_logger().info(
            f'DemoCollectorNode ready. Saving to {SAVE_PATH} every {SAVE_INTERVAL:.0f}s.'
        )

    # --- Callbacks ---

    def _cones_cb(self, msg: Track):
        with self._lock:
            self._cones = msg

    def _state_cb(self, msg):
        with self._lock:
            self._car_state = msg

    def _command_cb(self, msg: ControlCommand):
        """Record one (obs, action) pair on every expert command."""
        with self._lock:
            cones = self._cones
            state = self._car_state

        obs = _build_obs(cones, state)

        # Encode action as [steering, throttle_brake_norm]
        # throttle_brake_norm: positive = throttle, negative = brake
        if msg.throttle > 0.0:
            tbn = msg.throttle
        else:
            tbn = -msg.brake
        action = np.array([msg.steering, tbn], dtype=np.float32)

        with self._lock:
            self._obs_list.append(obs)
            self._act_list.append(action)

    # --- Save ---

    def save(self):
        with self._lock:
            if not self._obs_list:
                self.get_logger().warn('No data to save yet.')
                return
            obs_arr = np.stack(self._obs_list, axis=0)
            act_arr = np.stack(self._act_list, axis=0)

        os.makedirs(SAVE_DIR, exist_ok=True)
        np.savez(SAVE_PATH, obs=obs_arr, actions=act_arr)
        self.get_logger().info(
            f'Saved {len(obs_arr)} demos to {SAVE_PATH}  '
            f'(obs shape={obs_arr.shape}, actions shape={act_arr.shape})'
        )

    def n_samples(self) -> int:
        with self._lock:
            return len(self._obs_list)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = DemoCollectorNode()

    last_save = time.time()
    last_print = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            now = time.time()

            if now - last_print >= PRINT_INTERVAL:
                n = node.n_samples()
                print(f'[collect_demos] Collected {n} samples so far.')
                last_print = now

            if now - last_save >= SAVE_INTERVAL:
                node.save()
                last_save = now

    except KeyboardInterrupt:
        print('\n[collect_demos] Interrupted — saving final dataset...')
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
