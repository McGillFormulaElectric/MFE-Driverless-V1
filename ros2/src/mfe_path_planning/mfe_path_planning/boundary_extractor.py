#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
SensorDataQoS = lambda: QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import PointCloud2, PointField
from mfe_msgs.msg import Cone, Track
from std_msgs.msg import Header


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """
    Extract XYZ points from a sensor_msgs/PointCloud2 message.
    Assumes fields x, y, z as float32. Returns Nx3 numpy array.
    Uses vectorized numpy strides — safe for VLP-16 ~30k point clouds.
    """
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)

    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = msg.point_step

    def extract_field(offset: int) -> np.ndarray:
        # Build a strided view: take 4 bytes starting at `offset` in every `step`-byte block
        idx = np.arange(n) * step + offset
        col = np.stack([raw[idx], raw[idx+1], raw[idx+2], raw[idx+3]], axis=1)
        return col.view(np.float32).reshape(-1)

    xs = extract_field(field_map['x'].offset)
    ys = extract_field(field_map['y'].offset)
    zs = extract_field(field_map['z'].offset)

    pts = np.stack([xs, ys, zs], axis=1)
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid].astype(np.float32)


def _apply_tf(pts_xyz: np.ndarray, transform) -> np.ndarray:
    """Apply a geometry_msgs/TransformStamped to an Nx3 float32 array."""
    import math
    t = transform.transform
    tx, ty, tz = t.translation.x, t.translation.y, t.translation.z
    qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),   1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float32)
    t_vec = np.array([tx, ty, tz], dtype=np.float32)
    return (pts_xyz @ R.T) + t_vec


class BoundaryExtractor(Node):

    MATCH_RADIUS = 0.5  # meters — max distance to associate LiDAR centroid with camera cone

    def __init__(self):
        super().__init__('boundary_extractor')

        self._lidar_pts: np.ndarray = np.zeros((0, 3), dtype=np.float32)   # Nx3
        self._camera_cones: list = []   # list of mfe_msgs/Cone

        self._lidar_header: Header = Header()
        self._camera_header: Header = Header()

        # TF2 buffer and listener for frame transformations
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/perception/cones_uncolored',
            self._lidar_callback,
            SensorDataQoS())

        self.sub_camera = self.create_subscription(
            Track,
            'image/track',
            self._camera_callback,
            10)

        self.pub_track = self.create_publisher(Track, '/planning/cones', 10)

        # Publish at 10 Hz regardless of sensor rate
        self.timer = self.create_timer(0.1, self._publish_fused)

        self.get_logger().info('BoundaryExtractor started. Fusing LiDAR + camera cones.')

    def _lidar_callback(self, msg: PointCloud2) -> None:
        self._lidar_pts = pointcloud2_to_xyz(msg)
        self._lidar_header = msg.header

    def _camera_callback(self, msg: Track) -> None:
        self._camera_cones = msg.track
        if msg.track:
            self._camera_header = msg.track[0].header

    def _transform_lidar_pts_to_map(self, pts: np.ndarray, src_frame: str) -> tuple:
        """
        Try to transform an Nx3 array from src_frame to 'map'.
        Returns (transformed_pts, success). On failure returns (pts, False).
        """
        if pts.shape[0] == 0 or not src_frame:
            return pts, True

        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                'map',
                src_frame,
                rclpy.time.Time())
            return _apply_tf(pts, tf_stamped), True
        except TransformException as e:
            self.get_logger().warn(
                f'Could not transform LiDAR points from {src_frame!r} to map: {e}',
                throttle_duration_sec=2.0)
            return pts, False

    def _transform_camera_cones_to_map(self, cones: list, src_frame: str) -> tuple:
        """
        Try to transform camera cone positions from src_frame to 'map'.
        Returns (Nx3 ndarray of transformed xyz, success).
        On failure returns the raw positions, False.
        """
        if not cones or not src_frame:
            cam_xyz = np.array([[c.location.x, c.location.y, c.location.z]
                                for c in cones], dtype=np.float32)
            return cam_xyz, True

        cam_xyz = np.array([[c.location.x, c.location.y, c.location.z]
                            for c in cones], dtype=np.float32)
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                'map',
                src_frame,
                rclpy.time.Time())
            return _apply_tf(cam_xyz, tf_stamped), True
        except TransformException as e:
            self.get_logger().warn(
                f'Could not transform camera cones from {src_frame!r} to map: {e}',
                throttle_duration_sec=2.0)
            return cam_xyz, False

    def _publish_fused(self) -> None:
        now = self.get_clock().now().to_msg()
        fused: list[Cone] = []

        lidar_pts = self._lidar_pts          # Nx3
        camera_cones = self._camera_cones    # list of Cone msgs

        # --- Transform LiDAR points to map frame ---
        lidar_src_frame = self._lidar_header.frame_id
        lidar_pts_map, lidar_tf_ok = self._transform_lidar_pts_to_map(lidar_pts, lidar_src_frame)

        # --- Transform camera cone positions to map frame ---
        cam_src_frame = ''
        if camera_cones:
            cam_src_frame = self._camera_header.frame_id
        cam_xyz_map, _cam_tf_ok = self._transform_camera_cones_to_map(camera_cones, cam_src_frame)

        # Build camera KD-tree (x,y only) for fast lookup — using map-frame positions
        cam_xy = None
        if camera_cones and cam_xyz_map.shape[0] > 0:
            cam_xy = cam_xyz_map[:, :2]

        matched_camera_indices = set()

        # --- Step 1: For each LiDAR centroid (in map frame), find best camera match ---
        for i in range(len(lidar_pts_map)):
            lx = float(lidar_pts_map[i, 0])
            ly = float(lidar_pts_map[i, 1])
            lz = float(lidar_pts_map[i, 2])

            cone = Cone()
            cone.header.stamp = now
            cone.header.frame_id = 'map'
            cone.location.x = lx
            cone.location.y = ly
            cone.location.z = lz
            cone.color = Cone.UNKNOWN

            if cam_xy is not None and len(cam_xy) > 0:
                dists = np.sqrt((cam_xy[:, 0] - lx)**2 + (cam_xy[:, 1] - ly)**2)
                best_idx = int(np.argmin(dists))
                if dists[best_idx] <= self.MATCH_RADIUS:
                    cone.color = camera_cones[best_idx].color
                    matched_camera_indices.add(best_idx)

            fused.append(cone)

        # --- Step 2: Add unmatched camera cones (beyond LiDAR range or lateral) ---
        for j, cam_cone in enumerate(camera_cones):
            if j not in matched_camera_indices:
                cone = Cone()
                cone.header.stamp = now
                cone.header.frame_id = 'map'
                cone.location.x = float(cam_xyz_map[j, 0])
                cone.location.y = float(cam_xyz_map[j, 1])
                cone.location.z = float(cam_xyz_map[j, 2])
                cone.color = cam_cone.color
                fused.append(cone)

        track_msg = Track()
        track_msg.track = fused
        self.pub_track.publish(track_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
