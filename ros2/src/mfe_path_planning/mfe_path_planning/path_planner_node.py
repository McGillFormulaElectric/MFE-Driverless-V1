#!/usr/bin/env python3
"""
ft-fsd-path-planning ROS 2 wrapper node.

Library: https://github.com/papalotis/ft-fsd-path-planning
Install: pip install ft-fsd-path-planning

Subscribes:
    /planning/cones       (mfe_msgs/Track)        — fused, colored cone list
    /ekf/output           (nav_msgs/Odometry)      — vehicle pose + heading

Publishes:
    /planning/centerline  (nav_msgs/Path)          — computed centerline
    /planning/track_left  (nav_msgs/Path)          — left boundary
    /planning/track_right (nav_msgs/Path)          — right boundary
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mfe_msgs.msg import Cone, Track

try:
    from fsd_path_planning import PathPlanner, ConeTypes, MissionTypes
    _LIB_AVAILABLE = True
except ImportError:
    try:
        from ft_fsd_path_planning import PathPlanner, ConeTypes, MissionTypes
        _LIB_AVAILABLE = True
    except ImportError:
        _LIB_AVAILABLE = False


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# mfe_msgs/Cone color enum → fsd_path_planning ConeTypes
# BLUE=0 is left boundary, YELLOW=1 is right boundary (FSAE standard)
# fsd_path_planning ConeTypes: UNKNOWN=0, RIGHT=1, LEFT=2,
#   START_FINISH_AREA=3, START_FINISH_LINE=4
if _LIB_AVAILABLE:
    _MFE_TO_FSD = {
        Cone.BLUE:         ConeTypes.LEFT,
        Cone.YELLOW:       ConeTypes.RIGHT,
        Cone.ORANGE_BIG:   ConeTypes.UNKNOWN,   # keep planner from triggering finish logic
        Cone.ORANGE_SMALL: ConeTypes.UNKNOWN,
        Cone.UNKNOWN:      ConeTypes.UNKNOWN,
    }
else:
    _MFE_TO_FSD = {}


def _yaw_from_quaternion(q) -> float:
    """Extract yaw angle (radians) from a geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _path_from_xy(xy: np.ndarray, frame_id: str, stamp) -> Path:
    """Convert Nx2 numpy array of (x,y) points to nav_msgs/Path."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for point in xy:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = stamp
        ps.pose.position.x = float(point[0])
        ps.pose.position.y = float(point[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def _make_skidpad_path() -> np.ndarray:
    """
    Generate the full skidpad trajectory for the EUFS Gazebo track.

    Sequence (per user spec): 2 laps LEFT (upper circle, counterclockwise)
    then 2 laps RIGHT (lower circle, clockwise), then exit.

    Parameters tuned to the EUFS standard skidpad track dimensions:
      inner cone radius = 7.625 m (standard FSAE), outer = 10.6 m
      path radius = midpoint = 9.1 m, circle centres at (14.4, ±9.3)
    """
    cx, cy_l, cy_r = 14.4, 9.3, -9.3   # x shared, y for left/right circles
    R = 9.1                              # centreline radius

    N = 50    # waypoints per full lap (keep message small; ~220 total waypoints)

    # Entry straight from (0,0) to left-circle 6-o'clock entry
    n_entry = 40
    entry = np.column_stack([
        np.linspace(0, cx, n_entry),
        np.zeros(n_entry),
    ])

    # Left circle (upper, counterclockwise): entry at 6 o'clock (θ = -π/2), car moving +x
    theta_l = np.linspace(-math.pi / 2, -math.pi / 2 + 4 * math.pi, N * 2 + 1)
    left_circ = np.column_stack([cx + R * np.cos(theta_l), cy_l + R * np.sin(theta_l)])

    # Right circle (lower, clockwise): entry at 12 o'clock (θ = +π/2), car moving +x
    theta_r = np.linspace(math.pi / 2, math.pi / 2 - 4 * math.pi, N * 2 + 1)
    right_circ = np.column_stack([cx + R * np.cos(theta_r), cy_r + R * np.sin(theta_r)])

    # Exit straight from right-circle 12 o'clock back to y=0 then to end zone
    n_exit = 40
    exit_pts = np.column_stack([
        np.linspace(cx, 40.0, n_exit),
        np.zeros(n_exit),
    ])

    # Stitch: omit duplicate junction point between sections
    return np.vstack([entry[:-1], left_circ, right_circ, exit_pts])


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        if not _LIB_AVAILABLE:
            self.get_logger().error(
                'ft_fsd_path_planning is not installed. '
                'Run: pip install ft-fsd-path-planning')

        # ---------- Parameters ----------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mission', 'autocross')  # autocross | trackdrive | acceleration | skidpad

        self._map_frame = self.get_parameter('map_frame').value
        mission_str = self.get_parameter('mission').value
        self._mission = mission_str

        if _LIB_AVAILABLE and mission_str != 'skidpad':
            mission_map = {
                'autocross':   MissionTypes.autocross,
                'trackdrive':  MissionTypes.trackdrive,
                'acceleration': MissionTypes.acceleration,
            }
            mission = mission_map.get(mission_str, MissionTypes.autocross)
            self._planner = PathPlanner(mission)
        else:
            self._planner = None

        # Pre-compute skidpad path (published once on first cone observation)
        self._skidpad_path_published = False
        if mission_str == 'skidpad':
            self._skidpad_path = _make_skidpad_path()
            self.get_logger().info(
                f'PathPlannerNode started. Mission: skidpad | '
                f'hardcoded path: {len(self._skidpad_path)} waypoints, '
                f'sequence: 2 laps LEFT + 2 laps RIGHT + exit')
        else:
            self._skidpad_path = None
            self.get_logger().info(f'PathPlannerNode started. Mission: {mission_str}')

        # ---------- State ----------
        self._car_pos = np.array([0.0, 0.0])
        self._car_dir = np.array([1.0, 0.0])   # unit vector, default facing +X
        self._odom_received = False

        # ---------- Subscribers ----------
        self.create_subscription(Track, '/planning/cones', self._cones_cb, _QOS)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, _QOS)

        # ---------- Publishers ----------
        self._pub_centerline = self.create_publisher(Path, '/planning/centerline', _QOS)
        self._pub_left       = self.create_publisher(Path, '/planning/track_left',  _QOS)
        self._pub_right      = self.create_publisher(Path, '/planning/track_right', _QOS)

    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._car_pos = np.array([pos.x, pos.y])
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._car_dir = np.array([math.cos(yaw), math.sin(yaw)])
        self._odom_received = True

    def _cones_cb(self, msg: Track) -> None:
        # Skidpad: publish the pre-computed hardcoded path every time cones arrive
        # (published repeatedly so pure_pursuit doesn't miss it on startup)
        if self._mission == 'skidpad':
            if self._odom_received:
                stamp = self.get_clock().now().to_msg()
                self._pub_centerline.publish(
                    _path_from_xy(self._skidpad_path, self._map_frame, stamp))
                if not self._skidpad_path_published:
                    self._skidpad_path_published = True
                    self.get_logger().info(
                        f'Skidpad path first publish: {len(self._skidpad_path)} waypoints')
            return

        if not _LIB_AVAILABLE or self._planner is None:
            return

        if not self._odom_received:
            self.get_logger().warn('No odometry yet — skipping path calculation.', throttle_duration_sec=5.0)
            return

        if not msg.track:
            return

        # Build cones_by_type: list of Nx2 arrays indexed by ConeTypes enum value
        # ft-fsd-path-planning expects: [unknown[], right[], left[], orange_big[], orange_small[]]
        # Filter to cones within CONE_VIS_R of the car to avoid confusing the planner
        # with the full track at once (critical for figure-8 / crossing tracks).
        CONE_VIS_R = 20.0
        n_types = len(ConeTypes)
        buckets: list[list] = [[] for _ in range(n_types)]

        for cone in msg.track:
            dx = cone.location.x - self._car_pos[0]
            dy = cone.location.y - self._car_pos[1]
            if dx * dx + dy * dy > CONE_VIS_R * CONE_VIS_R:
                continue
            cone_type = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
            buckets[cone_type].append([cone.location.x, cone.location.y])

        cones_by_type = [
            np.array(b, dtype=np.float64).reshape(-1, 2) if b
            else np.zeros((0, 2), dtype=np.float64)
            for b in buckets
        ]

        try:
            result = self._planner.calculate_path_in_global_frame(
                cones_by_type,
                self._car_pos,
                self._car_dir,
                return_intermediate_results=True,
            )
            path_xy, left_xy, right_xy, *_ = result
        except Exception as e:
            self.get_logger().error(f'Path calculation failed: {e}', throttle_duration_sec=2.0)
            return

        cone_counts = [len(b) for b in buckets]
        self.get_logger().info(
            f'planner: pos={self._car_pos} dir={self._car_dir} '
            f'cones={cone_counts} path_pts={len(path_xy) if path_xy is not None else None}',
            throttle_duration_sec=2.0)

        stamp = self.get_clock().now().to_msg()
        frame = self._map_frame

        # final_path columns: (spline_parameter, path_x, path_y, curvature)
        # Use columns 1:3 for actual (x, y); col 0 is the spline parameter, not a coordinate.
        if path_xy is not None and len(path_xy) > 0:
            self._pub_centerline.publish(_path_from_xy(np.array(path_xy)[:, 1:3], frame, stamp))
        # sorted_left / sorted_right are Mx2 (x,y) already — use :2
        if left_xy is not None and len(left_xy) > 0:
            self._pub_left.publish(_path_from_xy(np.array(left_xy)[:, :2], frame, stamp))
        if right_xy is not None and len(right_xy) > 0:
            self._pub_right.publish(_path_from_xy(np.array(right_xy)[:, :2], frame, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
