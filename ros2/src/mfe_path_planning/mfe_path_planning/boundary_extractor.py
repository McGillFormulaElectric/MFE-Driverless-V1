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

    MATCH_RADIUS    = 0.5   # m — max distance to associate LiDAR centroid with camera cone
    GT_MATCH_RADIUS = 2.0   # m — looser match for GT color fallback
    GRID_STEP       = 0.3   # m — spatial bin size for confidence tracking

    def __init__(self):
        super().__init__('boundary_extractor')

        self.declare_parameter('min_observations', 3)   # frames a cone must appear before publishing
        self.declare_parameter('confidence_decay', 1)   # how much to subtract each frame it's not seen
        self._min_obs     = int(self.get_parameter('min_observations').value)
        self._conf_decay  = int(self.get_parameter('confidence_decay').value)

        # confidence_map: grid key -> [count, color, x, y, z]
        self._confidence_map: dict = {}

        self._lidar_pts: np.ndarray = np.zeros((0, 3), dtype=np.float32)   # Nx3
        self._camera_cones: list = []   # list of mfe_msgs/Cone
        self._gt_cones: list = []       # fallback color source when camera unavailable

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

        # GT color fallback when camera is not running.
        # Use /ground_truth/track_colored (ALL track cones, map frame) — not proximity-limited.
        # Matched against the already-transformed map-frame LiDAR centroids in _publish_fused.
        # This is more robust than /ground_truth/cones_colored (proximity-limited, base_footprint)
        # because the full track list is available from the first message, independent of car position.
        self.sub_gt = self.create_subscription(
            Track,
            '/ground_truth/track_colored',
            self._gt_callback,
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

    def _gt_callback(self, msg: Track) -> None:
        self._gt_cones = msg.track

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

        # Build camera lookup (x,y only, map frame)
        cam_xy = None
        if camera_cones and cam_xyz_map.shape[0] > 0:
            cam_xy = cam_xyz_map[:, :2]

        # Build GT color lookup — only used when camera unavailable.
        # /ground_truth/track_colored is in map frame — match against map-frame LiDAR centroids.
        gt_xy = None
        gt_colors = None
        if not camera_cones and self._gt_cones:
            gt_xy = np.array([[c.location.x, c.location.y]
                              for c in self._gt_cones], dtype=np.float32)
            gt_colors = [c.color for c in self._gt_cones]

        matched_camera_indices = set()

        # --- Step 1: For each LiDAR centroid (in map frame), find best color match ---
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
            elif gt_xy is not None and len(gt_xy) > 0:
                # GT cones in map frame; match against map-frame LiDAR centroid (lx, ly)
                dists = np.sqrt((gt_xy[:, 0] - lx)**2 + (gt_xy[:, 1] - ly)**2)
                best_idx = int(np.argmin(dists))
                if dists[best_idx] <= self.GT_MATCH_RADIUS:
                    cone.color = gt_colors[best_idx]

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

        # Count colors for diagnostics
        color_counts = {}
        for c in fused:
            color_counts[c.color] = color_counts.get(c.color, 0) + 1
        self.get_logger().info(
            f'fused {len(fused)} cones | lidar={len(lidar_pts)} gt_src={len(self._gt_cones)} '
            f'cam={len(camera_cones)} colors={color_counts}',
            throttle_duration_sec=2.0)

        # ---- Temporal confidence filtering ----
        # Accumulate confidence for each cone seen this frame; decay unseen cones.
        # Only publish cones that have been consistently observed (min_observations).
        step = self.GRID_STEP
        seen_keys: set = set()
        for cone in fused:
            gx = round(cone.location.x / step) * step
            gy = round(cone.location.y / step) * step
            key = (gx, gy)
            seen_keys.add(key)
            if key in self._confidence_map:
                entry = self._confidence_map[key]
                entry[0] = min(entry[0] + 1, 20)   # cap at 20
                entry[1] = cone.color               # update color with latest
            else:
                self._confidence_map[key] = [1, cone.color,
                                             cone.location.x, cone.location.y, cone.location.z]

        # Decay unseen cones; remove if confidence drops to zero
        to_remove = []
        for key, entry in self._confidence_map.items():
            if key not in seen_keys:
                entry[0] -= self._conf_decay
                if entry[0] <= 0:
                    to_remove.append(key)
        for key in to_remove:
            del self._confidence_map[key]

        # Publish only cones with sufficient confidence
        stable_cones: list[Cone] = []
        for key, (count, color, x, y, z) in self._confidence_map.items():
            if count >= self._min_obs:
                c = Cone()
                c.header.stamp    = now
                c.header.frame_id = 'map'
                c.location.x = x
                c.location.y = y
                c.location.z = z
                c.color      = color
                stable_cones.append(c)

        self.get_logger().info(
            f'temporal filter: raw={len(fused)} stable={len(stable_cones)} '
            f'tracked={len(self._confidence_map)}',
            throttle_duration_sec=2.0)

        track_msg = Track()
        track_msg.track = stable_cones
        self.pub_track.publish(track_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
