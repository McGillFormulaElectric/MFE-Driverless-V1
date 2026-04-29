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
        Cone.ORANGE_BIG:   ConeTypes.START_FINISH_AREA,
        Cone.ORANGE_SMALL: ConeTypes.START_FINISH_LINE,
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


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        if not _LIB_AVAILABLE:
            self.get_logger().error(
                'ft_fsd_path_planning is not installed. '
                'Run: pip install ft-fsd-path-planning')

        # ---------- Parameters ----------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mission', 'autocross')  # autocross | trackdrive | acceleration

        self._map_frame = self.get_parameter('map_frame').value
        mission_str = self.get_parameter('mission').value

        if _LIB_AVAILABLE:
            mission_map = {
                'autocross':   MissionTypes.autocross,
                'trackdrive':  MissionTypes.trackdrive,
                'acceleration': MissionTypes.acceleration,
                'skidpad':     MissionTypes.skidpad,
            }
            mission = mission_map.get(mission_str, MissionTypes.autocross)
            self._planner = PathPlanner(mission)
            self.get_logger().info(
                f'PathPlannerNode started. Mission: {mission_str}')
        else:
            self._planner = None

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
        if not _LIB_AVAILABLE or self._planner is None:
            return

        if not self._odom_received:
            self.get_logger().warn('No odometry yet — skipping path calculation.', throttle_duration_sec=5.0)
            return

        if not msg.track:
            return

        # Build cones_by_type: list of Nx2 arrays indexed by ConeTypes enum value
        # ft-fsd-path-planning expects: [unknown[], right[], left[], orange_big[], orange_small[]]
        n_types = len(ConeTypes)
        buckets: list[list] = [[] for _ in range(n_types)]

        for cone in msg.track:
            cone_type = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
            buckets[cone_type].append([cone.location.x, cone.location.y])

        cones_by_type = [
            np.array(b, dtype=np.float64).reshape(-1, 2) if b
            else np.zeros((0, 2), dtype=np.float64)
            for b in buckets
        ]

        try:
            path_xy, (left_xy, right_xy) = self._planner.calculate_path_in_global_frame(
                cones_by_type,
                self._car_pos,
                self._car_dir,
            )
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

        if path_xy is not None and len(path_xy) > 0:
            self._pub_centerline.publish(_path_from_xy(path_xy[:, :2], frame, stamp))
        if left_xy is not None and len(left_xy) > 0:
            self._pub_left.publish(_path_from_xy(left_xy[:, :2], frame, stamp))
        if right_xy is not None and len(right_xy) > 0:
            self._pub_right.publish(_path_from_xy(right_xy[:, :2], frame, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
