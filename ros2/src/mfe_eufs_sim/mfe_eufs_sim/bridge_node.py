#!/usr/bin/env python3
"""
EUFS Sim ↔ MFE Driverless Bridge Node

Translates between EUFS simulator topics and the MFE driverless stack topics.

EUFS Sim → Driverless Stack:
  /velodyne_points              (PointCloud2)              → /lidar/points_raw
  /zed/left/image_rect_color    (Image)                    → /camera/image_raw
  /ground_truth/cones           (eufs_msgs/ConeArrayWithCovariance) → /ground_truth/cones_colored (mfe_msgs/Track)
                                                                       (accuracy measurement only)
  /ground_truth/state           (eufs_msgs/CarState)       → /ground_truth/state_odom (nav_msgs/Odometry)

  When use_sim_cones_directly=True (GT bypass mode):
  /ground_truth/track           (eufs_msgs/ConeArrayWithCovariance) → /planning/cones (mfe_msgs/Track)
                                                                       BLUE+YELLOW only, world frame, no transform

  When use_sim_cones_directly=False (perception mode):
  /lidar/points_raw is consumed by lidar_perception_node → /perception/cones_uncolored
  Bridge does NOT publish to /perception/cones_uncolored — LiDAR detector owns that topic.

Driverless Stack → EUFS Sim:
  /control/command              (fs_msgs/ControlCommand)   → /cmd (ackermann_msgs/AckermannDriveStamped)

NOTE: The EKF node (mfe_state_estimation) publishes to /ekf/output from GPS+IMU data.
      The bridge publishes ground truth as /ground_truth/state_odom for accuracy measurement only.
      Do NOT publish to /ekf/output from the bridge — that topic belongs to the EKF node.
"""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from eufs_msgs.msg import ConeArrayWithCovariance
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
    MAX_SPEED_MS = 10.0                            # m/s — max speed (throttle=1, velocity mode)

    def __init__(self):
        super().__init__('mfe_eufs_sim_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_steering_deg', 25.0)
        self.declare_parameter('max_speed_ms', 10.0)
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
        # Ground truth cones (proximity-limited) → /ground_truth/cones_colored
        self.create_subscription(
            ConeArrayWithCovariance, '/ground_truth/cones', self._gt_cones_cb, _QOS_EUFS)
        self._gt_colored_pub = self.create_publisher(
            Track, '/ground_truth/cones_colored', _QOS_MFE)

        # Full track cones (ALL cones, map frame) → /ground_truth/track_colored
        # Used by boundary_extractor as a color fallback when camera is not running.
        self.create_subscription(
            ConeArrayWithCovariance, '/ground_truth/track', self._gt_track_cb, _QOS_EUFS)
        self._gt_track_colored_pub = self.create_publisher(
            Track, '/ground_truth/track_colored', _QOS_MFE)

        # GT bypass mode: forward full track cones directly to /planning/cones so
        # the path planner can run without the perception pipeline.
        # /ground_truth/track gives ALL cones in Gazebo world frame (= TF map frame) —
        # no coordinate transformation needed; positions are absolute and stable.
        if self._use_sim_cones_directly:
            self.create_subscription(
                ConeArrayWithCovariance, '/ground_truth/track', self._sim_cones_cb, _QOS_EUFS)
            self._planning_cones_pub = self.create_publisher(
                Track, '/planning/cones', _QOS_MFE)

        # ---------- EUFS → MFE: Vehicle state (ground truth for accuracy measurement) ----------
        # Publishes to /ground_truth/state_odom — NOT to /ekf/output.
        # The EKF node (mfe_state_estimation) is the sole publisher of /ekf/output.
        from eufs_msgs.msg import CarState
        self.create_subscription(
            CarState, '/ground_truth/state', self._car_state_cb, _QOS_EUFS)
        self._odom_pub = self.create_publisher(
            Odometry, '/ground_truth/state_odom', _QOS_MFE)

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

        mode = 'GT-bypass (→ /planning/cones)' if self._use_sim_cones_directly else 'perception (LiDAR detector owns /perception/cones_uncolored)'
        self.get_logger().info(
            f'EufsSimBridge started (velocity mode). '
            f'Cone mode: {mode} | '
            f'GT cones always → /ground_truth/cones_colored | '
            f'Ground truth state → /ground_truth/state_odom (NOT /ekf/output) | '
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
        Full track cones (GT bypass) → /planning/cones.

        /ground_truth/track gives ALL track cones in Gazebo world frame, which is
        identical to the TF map frame in a fresh sim (publish_gt_tf:=true sets
        map→odom→base_footprint from Gazebo). Positions are absolute and stable —
        no coordinate transformation required.

        Only BLUE and YELLOW boundary cones are forwarded. ORANGE_BIG are
        start/finish pylons that sit off to the side and confuse the path planner.
        """
        track = Track()
        stamp = msg.header.stamp

        def make_cone(point, color_int: int) -> Cone:
            c = Cone()
            c.header.stamp = stamp
            c.header.frame_id = self._map_frame
            c.location.x = float(point.x)
            c.location.y = float(point.y)
            c.location.z = float(point.z)
            c.color = color_int
            return c

        for cw in msg.blue_cones:
            track.track.append(make_cone(cw.point, Cone.BLUE))
        for cw in msg.yellow_cones:
            track.track.append(make_cone(cw.point, Cone.YELLOW))
        # ORANGE_BIG omitted — start/finish pylons at off-track positions confuse the planner

        self._planning_cones_pub.publish(track)

    def _gt_cones_cb(self, msg: ConeArrayWithCovariance) -> None:
        """Proximity-limited GT cones → /ground_truth/cones_colored."""
        # Use actual frame_id (base_footprint) so consumers can transform correctly.
        track = _cone_array_to_track(msg, msg.header.frame_id)
        self._gt_colored_pub.publish(track)

    def _gt_track_cb(self, msg: ConeArrayWithCovariance) -> None:
        """Full track cones (all, not proximity-limited) → /ground_truth/track_colored.
        Published in the EUFS track-local frame for debugging/visualisation in Foxglove.
        NOTE: frame_id='map' here is the Gazebo track-local origin, NOT the TF map frame.
        """
        track = _cone_array_to_track(msg, msg.header.frame_id)
        self._gt_track_colored_pub.publish(track)

    def _car_state_cb(self, msg) -> None:
        """Convert eufs_msgs/CarState → nav_msgs/Odometry → /ground_truth/state_odom."""
        odom = _car_state_to_odometry(msg)
        self._odom_pub.publish(odom)

    # ------------------------------------------------------------------ #
    #  MFE → EUFS callbacks                                               #
    # ------------------------------------------------------------------ #

    def _control_cb(self, msg: ControlCommand) -> None:
        """
        Convert fs_msgs/ControlCommand → ackermann_msgs/AckermannDriveStamped.

        EUFS sim must be launched with commandMode:=velocity so that
        AckermannDrive.speed is treated as m/s (target speed).

        throttle ∈ [0,1] → forward speed: throttle * MAX_SPEED_MS
        brake    ∈ [0,1] → overrides throttle, reduces speed proportionally
        steering ∈ [-1,+1] → steering_angle in radians
        """
        if not _ACKERMANN_AVAILABLE or self._cmd_pub is None:
            return

        # Steering: normalised → radians
        steering_rad = float(msg.steering) * self.MAX_STEERING_ANGLE_RAD

        # Speed: brake reduces the throttle demand (brake-by-wire)
        throttle = float(msg.throttle)
        brake = float(msg.brake)
        speed_ms = (throttle - brake) * self.MAX_SPEED_MS
        speed_ms = max(0.0, speed_ms)   # clamp — no reverse

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
