#!/usr/bin/env python3
"""
EUFS sim <-> MFE driverless bridge: remaps sensor and ground-truth topics, and converts
/control/command to /cmd via the drivetrain model (EUFS must run with commandMode:=acceleration).
Never publishes /ekf/output, which belongs to the EKF node.
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

from mfe_eufs_sim.drivetrain_model import AMKDrivetrainModel

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

_QOS_MFE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _cone_array_to_track(msg: ConeArrayWithCovariance, frame_id: str) -> Track:
    """Flatten the per-colour EUFS cone arrays into one mfe Track."""
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
    """Flatten all cones into an x, y, z float32 PointCloud2."""
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
    """Convert eufs CarState to nav_msgs/Odometry (pose and twist)."""
    odom = Odometry()
    odom.header = msg.header
    odom.child_frame_id = msg.child_frame_id

    odom.pose.pose.position.x = msg.pose.pose.position.x
    odom.pose.pose.position.y = msg.pose.pose.position.y
    odom.pose.pose.position.z = msg.pose.pose.position.z
    odom.pose.pose.orientation = msg.pose.pose.orientation
    # Both covariances are 6x6 row-major.
    if len(msg.pose.covariance) == 36:
        odom.pose.covariance = list(msg.pose.covariance)

    odom.twist.twist.linear = msg.twist.twist.linear
    odom.twist.twist.angular = msg.twist.twist.angular
    if len(msg.twist.covariance) == 36:
        odom.twist.covariance = list(msg.twist.covariance)

    return odom


class EufsSimBridge(Node):

    MAX_STEERING_ANGLE_RAD = math.radians(25.0)
    MAX_SPEED_MS = 10.0   # m/s

    def __init__(self):
        super().__init__('mfe_eufs_sim_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_steering_deg', 25.0)
        self.declare_parameter('max_speed_ms', 10.0)
        self.declare_parameter('use_sim_cones_directly', True)

        # Vehicle numbers are placeholders; see drivetrain_model.py.
        self.declare_parameter('gear_ratio', 14.0)
        self.declare_parameter('wheel_radius_m', 0.2525)
        self.declare_parameter('vehicle_mass_kg', 225.0)
        self.declare_parameter('max_system_power_w', 80000.0)
        self.declare_parameter('tire_mu', 1.4)
        self.declare_parameter('brake_bias_front', 0.6)
        self.declare_parameter('regen_braking_enabled', True)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self.MAX_STEERING_ANGLE_RAD = math.radians(
            self.get_parameter('max_steering_deg').value)
        self.MAX_SPEED_MS = self.get_parameter('max_speed_ms').value
        self._use_sim_cones_directly = self.get_parameter('use_sim_cones_directly').value

        self._drivetrain = AMKDrivetrainModel(
            gear_ratio=self.get_parameter('gear_ratio').value,
            wheel_radius_m=self.get_parameter('wheel_radius_m').value,
            vehicle_mass_kg=self.get_parameter('vehicle_mass_kg').value,
            max_system_power_w=self.get_parameter('max_system_power_w').value,
            tire_mu=self.get_parameter('tire_mu').value,
            brake_bias_front=self.get_parameter('brake_bias_front').value,
            regen_enabled=self.get_parameter('regen_braking_enabled').value,
        )
        self._current_speed_ms = 0.0
        self._last_control_time = None

        self.create_subscription(
            PointCloud2, '/velodyne_points', self._lidar_cb, _QOS_EUFS)
        self._lidar_pub = self.create_publisher(
            PointCloud2, '/lidar/points_raw', _QOS_MFE)

        self.create_subscription(
            Image, '/zed/left/image_rect_color', self._camera_cb, _QOS_EUFS)
        self._camera_pub = self.create_publisher(
            Image, '/camera/image_raw', _QOS_MFE)

        self.create_subscription(
            ConeArrayWithCovariance, '/ground_truth/cones', self._gt_cones_cb, _QOS_EUFS)
        self._gt_colored_pub = self.create_publisher(
            Track, '/ground_truth/cones_colored', _QOS_MFE)

        # Colour fallback for boundary_extractor when the camera is not running.
        self.create_subscription(
            ConeArrayWithCovariance, '/ground_truth/track', self._gt_track_cb, _QOS_EUFS)
        self._gt_track_colored_pub = self.create_publisher(
            Track, '/ground_truth/track_colored', _QOS_MFE)

        # GT bypass: forward all track cones to /planning/cones. World frame = TF map frame, no transform needed.
        if self._use_sim_cones_directly:
            self.create_subscription(
                ConeArrayWithCovariance, '/ground_truth/track', self._sim_cones_cb, _QOS_EUFS)
            self._planning_cones_pub = self.create_publisher(
                Track, '/planning/cones', _QOS_MFE)

        # Not /ekf/output: the EKF node (mfe_state_estimation) owns that topic.
        from eufs_msgs.msg import CarState
        self.create_subscription(
            CarState, '/ground_truth/state', self._car_state_cb, _QOS_EUFS)
        self._odom_pub = self.create_publisher(
            Odometry, '/ground_truth/state_odom', _QOS_MFE)

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
            f'EufsSimBridge started (acceleration mode — 4x AMK DD5-14-10-POW drivetrain model). '
            f'Cone mode: {mode} | '
            f'GT cones always → /ground_truth/cones_colored | '
            f'Ground truth state → /ground_truth/state_odom (NOT /ekf/output) | '
            f'gear_ratio={self._drivetrain.gear_ratio} wheel_r={self._drivetrain.wheel_radius_m}m '
            f'mass={self._drivetrain.vehicle_mass_kg}kg tire_mu={self._drivetrain.tire_mu} | '
            f'Max steering: {math.degrees(self.MAX_STEERING_ANGLE_RAD):.1f}°')

    def _lidar_cb(self, msg: PointCloud2) -> None:
        """Remap /velodyne_points → /lidar/points_raw."""
        # Keep the velodyne frame_id: lidar_perception_node reads it from the message.
        self._lidar_pub.publish(msg)

    def _camera_cb(self, msg: Image) -> None:
        """Remap /zed/left/image_rect_color → /camera/image_raw."""
        self._camera_pub.publish(msg)

    def _sim_cones_cb(self, msg: ConeArrayWithCovariance) -> None:
        """Forward BLUE and YELLOW track cones (world frame = TF map frame) to /planning/cones."""
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
        # ORANGE_BIG (start/finish pylons, off track) confuse the planner.

        self._planning_cones_pub.publish(track)

    def _gt_cones_cb(self, msg: ConeArrayWithCovariance) -> None:
        """Proximity-limited GT cones → /ground_truth/cones_colored."""
        # Keep the source frame_id (base_footprint) so consumers can transform.
        track = _cone_array_to_track(msg, msg.header.frame_id)
        self._gt_colored_pub.publish(track)

    def _gt_track_cb(self, msg: ConeArrayWithCovariance) -> None:
        """All track cones → /ground_truth/track_colored. frame_id 'map' is the Gazebo track-local origin, not the TF map frame."""
        track = _cone_array_to_track(msg, msg.header.frame_id)
        self._gt_track_colored_pub.publish(track)

    def _car_state_cb(self, msg) -> None:
        """Publish ground-truth odometry and cache speed for the drivetrain model."""
        odom = _car_state_to_odometry(msg)
        self._odom_pub.publish(odom)
        # Ground-truth speed stands in for wheel-speed encoders, not the noisy Xsens odom.
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed_ms = math.hypot(vx, vy)

    def _control_cb(self, msg: ControlCommand) -> None:
        """
        Convert fs_msgs/ControlCommand → ackermann_msgs/AckermannDriveStamped.

        Requires commandMode:=acceleration: AckermannDrive.acceleration is applied directly, bypassing
        the plugin's velocity loop (see gazebo_ros_race_car_model.cpp).
        throttle in [0, 1] gives positive acceleration; brake in [0, 1] gives negative acceleration
        and overrides throttle. steering in [-1, 1] maps to steering_angle in radians.
        """
        if not _ACKERMANN_AVAILABLE or self._cmd_pub is None:
            return

        steering_rad = float(msg.steering) * self.MAX_STEERING_ANGLE_RAD

        throttle = float(msg.throttle)
        brake = float(msg.brake)
        speed_ms = self._current_speed_ms

        now = self.get_clock().now()
        dt = ((now - self._last_control_time).nanoseconds * 1e-9
              if self._last_control_time is not None else 0.02)
        self._last_control_time = now

        if brake > 0.0:
            accel_ms2 = -self._drivetrain.compute_deceleration(brake, speed_ms)
        elif speed_ms >= self.MAX_SPEED_MS:
            # Hard top-speed cap, independent of pure_pursuit's max_speed.
            accel_ms2 = 0.0
        else:
            accel_ms2 = self._drivetrain.compute_acceleration(throttle, speed_ms, dt)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self._base_frame
        cmd.drive.steering_angle = steering_rad
        cmd.drive.acceleration = accel_ms2

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EufsSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
