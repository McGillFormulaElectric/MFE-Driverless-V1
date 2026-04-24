#!/usr/bin/env python3
"""
EUFS Sim ↔ MFE Driverless Bridge Node

Translates between EUFS simulator topics and the MFE driverless stack topics.

EUFS Sim → Driverless Stack:
  /velodyne_points              (PointCloud2)              → /lidar/points_raw
  /zed/left/image_rect_color    (Image)                    → /camera/image_raw
  /cones                        (eufs_msgs/ConeArrayWithCovariance) → /planning/cones (mfe_msgs/Track)
  /ground_truth/cones           (eufs_msgs/ConeArrayWithCovariance) → /perception/cones_uncolored (PointCloud2)
  /ground_truth/state           (eufs_msgs/CarState)       → /ekf/output (nav_msgs/Odometry)

Driverless Stack → EUFS Sim:
  /control/command              (fs_msgs/ControlCommand)   → /cmd (ackermann_msgs/AckermannDriveStamped)
"""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from eufs_msgs.msg import ConeArrayWithCovariance, CanState
from mfe_msgs.msg import Cone, Track
from fs_msgs.msg import ControlCommand

try:
    from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
    _ACKERMANN_AVAILABLE = True
except ImportError:
    _ACKERMANN_AVAILABLE = False


# EUFS sim publishes with BEST_EFFORT reliability
_QOS_EUFS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# MFE driverless stack uses RELIABLE
_QOS_MFE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _cone_array_to_track(msg: ConeArrayWithCovariance, frame_id: str) -> Track:
    """
    Convert eufs_msgs/ConeArrayWithCovariance → mfe_msgs/Track.

    EUFS organises cones by color in separate arrays.
    mfe_msgs/Cone uses a flat list with a uint8 color enum:
        BLUE=0, YELLOW=1, ORANGE_BIG=2, ORANGE_SMALL=3, UNKNOWN=4
    """
    track = Track()
    stamp = msg.header.stamp

    def make_cone(point, color_int: int) -> Cone:
        c = Cone()
        c.header.stamp = stamp
        c.header.frame_id = frame_id
        c.location.x = float(point.x)
        c.location.y = float(point.y)
        c.location.z = float(point.z)
        c.color = color_int
        return c

    for cw in msg.blue_cones:
        track.track.append(make_cone(cw.point, Cone.BLUE))
    for cw in msg.yellow_cones:
        track.track.append(make_cone(cw.point, Cone.YELLOW))
    for cw in msg.big_orange_cones:
        track.track.append(make_cone(cw.point, Cone.ORANGE_BIG))
    for cw in msg.orange_cones:
        track.track.append(make_cone(cw.point, Cone.ORANGE_SMALL))
    for cw in msg.unknown_color_cones:
        track.track.append(make_cone(cw.point, Cone.UNKNOWN))

    return track


def _cone_array_to_pointcloud2(msg: ConeArrayWithCovariance) -> PointCloud2:
    """
    Convert all cones in eufs_msgs/ConeArrayWithCovariance to a flat
    sensor_msgs/PointCloud2 (x,y,z float32) for /perception/cones_uncolored.
    Used to feed the boundary_extractor without bypassing the fusion step.
    """
    all_cones = (
        list(msg.blue_cones)
        + list(msg.yellow_cones)
        + list(msg.orange_cones)
        + list(msg.big_orange_cones)
        + list(msg.unknown_color_cones)
    )

    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 12  # 3 × float32
    data = bytearray()
    for cw in all_cones:
        data += struct.pack('<fff', float(cw.point.x), float(cw.point.y), float(cw.point.z))

    cloud = PointCloud2()
    cloud.header = msg.header
    cloud.height = 1
    cloud.width = len(all_cones)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = point_step
    cloud.row_step = point_step * len(all_cones)
    cloud.is_dense = True
    cloud.data = bytes(data)
    return cloud


def _car_state_to_odometry(msg) -> Odometry:
    """
    Convert eufs_msgs/CarState → nav_msgs/Odometry.

    CarState.pose  (PoseWithCovariance) → Odometry.pose
    CarState.twist (TwistWithCovariance) → Odometry.twist
    """
    odom = Odometry()
    odom.header = msg.header
    odom.child_frame_id = msg.child_frame_id

    odom.pose.pose.position.x = msg.pose.pose.position.x
    odom.pose.pose.position.y = msg.pose.pose.position.y
    odom.pose.pose.position.z = msg.pose.pose.position.z
    odom.pose.pose.orientation = msg.pose.pose.orientation
    # CarState covariance is 6×6 row-major; Odometry.pose.covariance is also 36 floats
    if len(msg.pose.covariance) == 36:
        odom.pose.covariance = list(msg.pose.covariance)

    odom.twist.twist.linear = msg.twist.twist.linear
    odom.twist.twist.angular = msg.twist.twist.angular
    if len(msg.twist.covariance) == 36:
        odom.twist.covariance = list(msg.twist.covariance)

    return odom


class EufsSimBridge(Node):

    # Physical limits used for control command conversion
    MAX_STEERING_ANGLE_RAD = math.radians(25.0)   # ±25° max steering
    MAX_SPEED_MS = 15.0                            # m/s — tune for competition event

    def __init__(self):
        super().__init__('mfe_eufs_sim_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_steering_deg', 25.0)
        self.declare_parameter('max_speed_ms', 15.0)
        self.declare_parameter('use_sim_cones_directly', True)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self.MAX_STEERING_ANGLE_RAD = math.radians(
            self.get_parameter('max_steering_deg').value)
        self.MAX_SPEED_MS = self.get_parameter('max_speed_ms').value
        self._use_sim_cones_directly = self.get_parameter('use_sim_cones_directly').value

        # ---------- EUFS → MFE: Sensor streams ----------
        # LiDAR: just a remap — same PointCloud2 type
        self.create_subscription(
            PointCloud2, '/velodyne_points', self._lidar_cb, _QOS_EUFS)
        self._lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/points_raw', _QOS_MFE)

        # Camera: just a remap — same Image type
        self.create_subscription(
            Image, '/zed/left/image_rect_color', self._camera_cb, _QOS_EUFS)
        self._camera_pub = self.create_publisher(
            Image, '/camera/image_raw', _QOS_MFE)

        # ---------- EUFS → MFE: Cones ----------
        # /cones — noisy perception-level cones (simulates what the perception stack
        # would output). When use_sim_cones_directly=True this bypasses the LiDAR/
        # vision pipeline and feeds directly into the path planner — useful for
        # testing path planning in isolation.
        if self._use_sim_cones_directly:
            self.create_subscription(
                ConeArrayWithCovariance, '/cones', self._sim_cones_cb, _QOS_EUFS)
            self._planning_cones_pub = self.create_publisher(
                Track, '/planning/cones', _QOS_MFE)

        # /ground_truth/cones — noiseless, feeds /perception/cones_uncolored so the
        # boundary_extractor and full stack can be exercised end-to-end.
        self.create_subscription(
            ConeArrayWithCovariance, '/ground_truth/cones',
            self._gt_cones_cb, _QOS_EUFS)
        self._uncolored_pub = self.create_publisher(
            PointCloud2, '/perception/cones_uncolored', _QOS_MFE)

        # ---------- EUFS → MFE: Vehicle state ----------
        from eufs_msgs.msg import CarState
        self.create_subscription(
            CarState, '/ground_truth/state', self._car_state_cb, _QOS_EUFS)
        self._odom_pub = self.create_publisher(
            Odometry, '/ekf/output', _QOS_MFE)

        # ---------- MFE → EUFS: Control commands ----------
        self.create_subscription(
            ControlCommand, '/control/command', self._control_cb, _QOS_MFE)
        if _ACKERMANN_AVAILABLE:
            self._cmd_pub = self.create_publisher(
                AckermannDriveStamped, '/cmd', _QOS_MFE)
        else:
            self._cmd_pub = None
            self.get_logger().warn(
                'ackermann_msgs not found — /control/command will NOT be forwarded to /cmd. '
                'Install with: sudo apt install ros-$ROS_DISTRO-ackermann-msgs')

        self.get_logger().info(
            f'EufsSimBridge started. '
            f'Sim cones → planner: {self._use_sim_cones_directly} | '
            f'Max speed: {self.MAX_SPEED_MS} m/s | '
            f'Max steering: {math.degrees(self.MAX_STEERING_ANGLE_RAD):.1f}°')

    # ------------------------------------------------------------------ #
    #  EUFS → MFE callbacks                                               #
    # ------------------------------------------------------------------ #

    def _lidar_cb(self, msg: PointCloud2) -> None:
        """Remap /velodyne_points → /lidar/points_raw."""
        # Preserve original frame_id (velodyne) — the lidar_perception_node
        # uses the frame_id from the message, not a parameter.
        self._lidar_pub.publish(msg)

    def _camera_cb(self, msg: Image) -> None:
        """Remap /zed/left/image_rect_color → /camera/image_raw."""
        self._camera_pub.publish(msg)

    def _sim_cones_cb(self, msg: ConeArrayWithCovariance) -> None:
        """
        Convert noisy sim cones → mfe_msgs/Track and publish to /planning/cones.
        Bypasses lidar_perception_node and boundary_extractor — for planner testing.
        """
        track = _cone_array_to_track(msg, self._map_frame)
        self._planning_cones_pub.publish(track)

    def _gt_cones_cb(self, msg: ConeArrayWithCovariance) -> None:
        """
        Convert ground truth cones → PointCloud2 and publish to
        /perception/cones_uncolored so boundary_extractor can process them.
        """
        cloud = _cone_array_to_pointcloud2(msg)
        self._uncolored_pub.publish(cloud)

    def _car_state_cb(self, msg) -> None:
        """Convert eufs_msgs/CarState → nav_msgs/Odometry → /ekf/output."""
        odom = _car_state_to_odometry(msg)
        self._odom_pub.publish(odom)

    # ------------------------------------------------------------------ #
    #  MFE → EUFS callbacks                                               #
    # ------------------------------------------------------------------ #

    def _control_cb(self, msg: ControlCommand) -> None:
        """
        Convert fs_msgs/ControlCommand → ackermann_msgs/AckermannDriveStamped.

        ControlCommand fields (all normalised):
            steering  ∈ [-1, +1]   (+1 = full left in FSAE convention)
            throttle  ∈ [0, 1]
            brake     ∈ [0, 1]

        AckermannDrive fields:
            steering_angle  (radians)
            speed           (m/s)    — EUFS sim is in velocity control mode
        """
        if not _ACKERMANN_AVAILABLE or self._cmd_pub is None:
            return

        # Steering: normalised → radians
        steering_rad = float(msg.steering) * self.MAX_STEERING_ANGLE_RAD

        # Speed: throttle drives forward, braking overrides
        # Simple blending: net_throttle in [-1, 1]
        net = float(msg.throttle) - float(msg.brake)
        speed_ms = net * self.MAX_SPEED_MS

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._base_frame
        cmd.drive.steering_angle = steering_rad
        cmd.drive.speed = speed_ms

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EufsSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
