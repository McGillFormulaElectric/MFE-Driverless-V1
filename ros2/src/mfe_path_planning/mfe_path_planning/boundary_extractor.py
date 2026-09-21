#!/usr/bin/env python3

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
SensorDataQoS = lambda: QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import PointCloud2, PointField
from mfe_msgs.msg import Cone, Track
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# ConeTrack dataclass + Kalman model constants
# ---------------------------------------------------------------------------

@dataclass
class ConeTrack:
    id: int
    color: int                          # mfe_msgs/Cone color enum
    x: float                            # map-frame position (convenience copy of state[0])
    y: float                            # map-frame position (convenience copy of state[1])
    # 4-state Kalman: [x, y, vx, vy]
    state: np.ndarray = field(default_factory=lambda: np.zeros(4))
    cov: np.ndarray   = field(default_factory=lambda: np.eye(4) * 1.0)
    hits: int   = 0   # confirmed observations
    misses: int = 0   # consecutive frames without a match


# Constant-velocity Kalman model at 10 Hz
_DT = 0.1
_F  = np.array([[1, 0, _DT, 0],
                [0, 1, 0,  _DT],
                [0, 0, 1,   0],
                [0, 0, 0,   1]], dtype=float)
_H  = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0]], dtype=float)
_Q  = np.diag([0.01, 0.01, 0.1, 0.1])   # process noise
_R  = np.diag([0.04, 0.04])              # observation noise (2 cm std)


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# BoundaryExtractor node
# ---------------------------------------------------------------------------

class BoundaryExtractor(Node):

    MATCH_RADIUS = 0.5   # meters — max distance to associate LiDAR centroid with camera cone
    GT_MATCH_RADIUS = 2.0  # meters — looser match for GT color fallback

    def __init__(self):
        super().__init__('boundary_extractor')

        self._lidar_pts: np.ndarray = np.zeros((0, 3), dtype=np.float32)   # Nx3
        self._camera_cones: list = []   # list of mfe_msgs/Cone
        self._gt_cones: list = []       # fallback color source when camera unavailable

        self._lidar_header: Header = Header()
        self._camera_header: Header = Header()

        # TF2 buffer and listener for frame transformations
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Per-cone Kalman tracks
        self._tracks: list[ConeTrack] = []
        self._next_track_id: int = 0

        self.declare_parameter('min_hits', 3)
        self.declare_parameter('max_misses', 5)
        self.declare_parameter('match_radius', 1.0)
        self._min_hits   = int(self.get_parameter('min_hits').value)
        self._max_misses = int(self.get_parameter('max_misses').value)
        self._match_r    = float(self.get_parameter('match_radius').value)

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

        self.get_logger().info(
            f'BoundaryExtractor started. '
            f'Fusing LiDAR + camera cones with Hungarian+Kalman tracking '
            f'(min_hits={self._min_hits}, max_misses={self._max_misses}, '
            f'match_radius={self._match_r} m).')

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _lidar_callback(self, msg: PointCloud2) -> None:
        self._lidar_pts = pointcloud2_to_xyz(msg)
        self._lidar_header = msg.header

    def _camera_callback(self, msg: Track) -> None:
        self._camera_cones = msg.track
        if msg.track:
            self._camera_header = msg.track[0].header

    def _gt_callback(self, msg: Track) -> None:
        self._gt_cones = msg.track

    # ------------------------------------------------------------------
    # Frame transforms
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Hungarian + Kalman track management
    # ------------------------------------------------------------------

    def _update_tracks(self, fused_cones: list[Cone]) -> list[ConeTrack]:
        """
        Run one predict-update cycle of Hungarian data association + per-track
        Kalman filter.

        Args:
            fused_cones: raw fused cone detections for this frame (map frame).

        Returns:
            List of confirmed tracks (hits >= min_hits) after update.
        """
        # --- Predict all existing tracks ---
        for t in self._tracks:
            t.state = _F @ t.state
            t.cov   = _F @ t.cov @ _F.T + _Q
            # Sync convenience x/y with prediction
            t.x = float(t.state[0])
            t.y = float(t.state[1])

        if not fused_cones:
            # No detections this frame — increment misses for all tracks
            for t in self._tracks:
                t.misses += 1
            self._tracks = [t for t in self._tracks if t.misses <= self._max_misses]
            return [t for t in self._tracks if t.hits >= self._min_hits]

        det_xy = np.array([[c.location.x, c.location.y] for c in fused_cones], dtype=float)

        matched_track_ids: set[int] = set()
        matched_det_indices: set[int] = set()

        if self._tracks:
            track_xy = np.array([[t.x, t.y] for t in self._tracks], dtype=float)

            # Build N_tracks x N_detections cost matrix (Euclidean distance)
            dx = track_xy[:, 0:1] - det_xy[:, 0]   # (N, M) broadcast
            dy = track_xy[:, 1:2] - det_xy[:, 1]
            cost = np.sqrt(dx**2 + dy**2)

            row_ind, col_ind = linear_sum_assignment(cost)

            for r, c in zip(row_ind, col_ind):
                if cost[r, c] <= self._match_r:
                    track = self._tracks[r]
                    det   = fused_cones[c]

                    # Kalman update
                    z = np.array([det.location.x, det.location.y], dtype=float)
                    S = _H @ track.cov @ _H.T + _R
                    K = track.cov @ _H.T @ np.linalg.inv(S)
                    track.state = track.state + K @ (z - _H @ track.state)
                    track.cov   = (np.eye(4) - K @ _H) @ track.cov

                    # Sync convenience fields
                    track.x = float(track.state[0])
                    track.y = float(track.state[1])

                    # Accumulate hits; reset misses
                    track.hits  += 1
                    track.misses = 0

                    # Refine color if currently UNKNOWN and detection has a known color
                    if track.color == Cone.UNKNOWN and det.color != Cone.UNKNOWN:
                        track.color = det.color

                    matched_track_ids.add(id(track))
                    matched_det_indices.add(c)

        # --- Unmatched tracks: increment misses ---
        for t in self._tracks:
            if id(t) not in matched_track_ids:
                t.misses += 1

        # --- Unmatched detections: spawn new tentative tracks ---
        for j, det in enumerate(fused_cones):
            if j not in matched_det_indices:
                init_state = np.array(
                    [det.location.x, det.location.y, 0.0, 0.0], dtype=float)
                new_track = ConeTrack(
                    id=self._next_track_id,
                    color=det.color,
                    x=det.location.x,
                    y=det.location.y,
                    state=init_state,
                    cov=np.eye(4) * 1.0,
                    hits=1,
                    misses=0,
                )
                self._tracks.append(new_track)
                self._next_track_id += 1

        # --- Delete stale tracks ---
        self._tracks = [t for t in self._tracks if t.misses <= self._max_misses]

        return [t for t in self._tracks if t.hits >= self._min_hits]

    # ------------------------------------------------------------------
    # Main publish callback
    # ------------------------------------------------------------------

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

        # --- Step 3: Hungarian + Kalman tracking ---
        confirmed = self._update_tracks(fused)

        # Build stable cone list from confirmed tracks
        stable_cones: list[Cone] = []
        for t in confirmed:
            c = Cone()
            c.header.stamp = now
            c.header.frame_id = 'map'
            c.location.x = float(t.state[0])   # Kalman-filtered position
            c.location.y = float(t.state[1])
            c.location.z = 0.0
            c.color = t.color
            stable_cones.append(c)

        # Diagnostics
        color_counts: dict = {}
        for c in stable_cones:
            color_counts[c.color] = color_counts.get(c.color, 0) + 1
        self.get_logger().info(
            f'detections={len(fused)} '
            f'tracks_total={len(self._tracks)} '
            f'tracks_confirmed={len(confirmed)} '
            f'| lidar={len(lidar_pts)} gt_src={len(self._gt_cones)} '
            f'cam={len(camera_cones)} colors={color_counts}',
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
