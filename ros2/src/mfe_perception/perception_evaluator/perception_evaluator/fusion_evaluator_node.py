#!/usr/bin/env python3
"""
Fusion perception evaluator.

Independently fuses D435i (YOLO camera) + LiDAR centroid detections, then
compares the merged result against ground truth to produce:

  Location accuracy %  — fraction of GT cones detected within EVAL_RADIUS
  Color accuracy %     — of matched pairs, fraction with correct color label
  Combined score %     — mean of the two above

Subscriptions:
  image/track                   (mfe_msgs/Track)          camera YOLO (camera_base frame)
  /perception/cones_uncolored   (sensor_msgs/PointCloud2)  LiDAR centroids (velodyne frame)
  /ground_truth/cones_colored   (mfe_msgs/Track)           GT cones (base_footprint frame)

Publications:
  /perception/fusion_accuracy   (diagnostic_msgs/DiagnosticArray) @ 2 Hz

CSV log: ~/mfe_logs/fusion_accuracy.csv
"""

import csv
import math
import os
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mfe_msgs.msg import Track  # noqa: F401
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, PointCloud2
import tf2_ros
from tf2_ros import TransformException

TARGET_FRAME   = 'base_footprint'
CAMERA_MATCH_R = 0.5    # m — max dist to assign camera colour to a LiDAR centroid (3D mode)
PIXEL_MATCH_R  = 60.0   # px — max pixel distance to assign colour from camera (pixel mode)
EVAL_RADIUS    = 1.5    # m — GT ↔ detection association radius
UNKNOWN        = 4
COLOR_NAMES    = {0: 'BLUE', 1: 'YELLOW', 2: 'ORANGE_BIG', 3: 'ORANGE_SMALL', 4: 'UNKNOWN'}


def _quat_to_rot3(qx, qy, qz, qw) -> np.ndarray:
    """3×3 rotation matrix from unit quaternion."""
    return np.array([
        [1-2*(qy*qy+qz*qz),  2*(qx*qy-qw*qz),    2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),    1-2*(qx*qx+qz*qz),  2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),    2*(qy*qz+qw*qx),    1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def _extract_xyz(cloud: PointCloud2) -> np.ndarray:
    """Decode PointCloud2 → Nx3 float64 (x, y, z)."""
    fields = {f.name: f for f in cloud.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.zeros((0, 3))
    n = cloud.width * cloud.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8)
    step = cloud.point_step

    def _get(offset):
        idx = np.arange(n) * step + offset
        col = np.stack([raw[idx], raw[idx+1], raw[idx+2], raw[idx+3]], axis=1)
        return col.view(np.float32).reshape(-1).astype(np.float64)

    pts = np.stack([_get(fields['x'].offset),
                    _get(fields['y'].offset),
                    _get(fields['z'].offset)], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


class FusionEvaluator(Node):

    def __init__(self):
        super().__init__('fusion_evaluator')

        self._lidar_cloud: PointCloud2 | None = None
        self._camera_cones: list = []   # list of mfe_msgs/Cone
        self._gt_cones: list = []       # list of (x, y, color) in base_footprint

        # Camera intrinsics — populated from CameraInfo; used for pixel-mode matching
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self.create_subscription(Track, 'image/track', self._camera_cb, 10)
        self.create_subscription(PointCloud2, '/perception/cones_uncolored', self._lidar_cb, 10)
        self.create_subscription(Track, '/ground_truth/cones_colored', self._gt_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._camera_info_cb, 10)

        self._pub = self.create_publisher(DiagnosticArray, '/perception/fusion_accuracy', 10)
        self.create_timer(2.0, self._evaluate)

        log_dir = os.path.expanduser('~/mfe_logs')
        os.makedirs(log_dir, exist_ok=True)
        self._csv_path = os.path.join(log_dir, 'fusion_accuracy.csv')
        if not os.path.exists(self._csv_path):
            with open(self._csv_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'timestamp', 'location_pct', 'color_pct', 'combined_pct',
                    'gt_total', 'detected', 'color_correct', 'color_checked', 'fused_total',
                ])

        self.get_logger().info('FusionEvaluator ready — waiting for data...')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _camera_cb(self, msg: Track):
        self._camera_cones = list(msg.track)

    def _lidar_cb(self, msg: PointCloud2):
        self._lidar_cloud = msg

    def _gt_cb(self, msg: Track):
        self._gt_cones = [(c.location.x, c.location.y, c.color) for c in msg.track]

    def _camera_info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}'
            )

    # ------------------------------------------------------------------ #
    # TF helpers
    # ------------------------------------------------------------------ #

    def _lookup_tf(self, src_frame: str):
        """Return (R3×3, t3) for src_frame → TARGET_FRAME, or None on failure."""
        if src_frame == TARGET_FRAME:
            return np.eye(3), np.zeros(3)
        try:
            tf = self._tf.lookup_transform(TARGET_FRAME, src_frame, rclpy.time.Time())
        except TransformException:
            return None
        r = tf.transform.rotation
        t = tf.transform.translation
        R = _quat_to_rot3(r.x, r.y, r.z, r.w)
        T = np.array([t.x, t.y, t.z])
        return R, T

    def _transform_cones(self, cones: list) -> list:
        """Transform mfe_msgs/Cone list → TARGET_FRAME. Returns [(x, y, color), ...]."""
        if not cones:
            return []
        frame = cones[0].header.frame_id or TARGET_FRAME
        tf_result = self._lookup_tf(frame)
        if tf_result is None:
            return []
        R, T = tf_result
        out = []
        for c in cones:
            p = np.array([c.location.x, c.location.y, c.location.z])
            p2 = R @ p + T
            out.append((float(p2[0]), float(p2[1]), int(c.color)))
        return out

    def _transform_cloud(self, cloud: PointCloud2) -> np.ndarray:
        """Extract cone centroids from PointCloud2 → TARGET_FRAME xy. Returns Nx2."""
        pts = _extract_xyz(cloud)
        if len(pts) == 0:
            return np.zeros((0, 2))
        tf_result = self._lookup_tf(cloud.header.frame_id)
        if tf_result is None:
            return np.zeros((0, 2))
        R, T = tf_result
        pts3 = (R @ pts.T).T + T
        return pts3[:, :2]

    # ------------------------------------------------------------------ #
    # Fusion: LiDAR centroids coloured by nearest camera cone
    # ------------------------------------------------------------------ #

    def _project_lidar_to_pixels(self, lidar_xy_bf: np.ndarray) -> np.ndarray:
        """
        Project LiDAR centroids (Nx2, base_footprint XY) to camera image pixels (Nx2).
        Returns Nx2 array of (px, py) or empty array if intrinsics / TF unavailable.
        Drops points behind the camera (Z <= 0 in camera frame).
        """
        if self._fx is None:
            return np.zeros((0, 2))

        # Look up base_footprint → camera_base (inverse of camera_base → base_footprint)
        try:
            tf_cam = self._tf.lookup_transform(
                'camera_base', TARGET_FRAME, rclpy.time.Time()
            )
        except TransformException:
            return np.zeros((0, 2))

        r = tf_cam.transform.rotation
        t = tf_cam.transform.translation
        R = _quat_to_rot3(r.x, r.y, r.z, r.w)
        T = np.array([t.x, t.y, t.z])

        # Lift XY to XYZ (z=0 in base_footprint plane) then transform
        pts3 = np.column_stack([lidar_xy_bf[:, 0], lidar_xy_bf[:, 1], np.zeros(len(lidar_xy_bf))])
        cam_pts = (R @ pts3.T).T + T   # Nx3 in camera_base frame

        valid = cam_pts[:, 2] > 0.05   # only points in front of camera
        if not np.any(valid):
            return np.zeros((0, 2))

        cp = cam_pts[valid]
        px = self._fx * cp[:, 0] / cp[:, 2] + self._cx
        py = self._fy * cp[:, 1] / cp[:, 2] + self._cy
        pixels = np.column_stack([px, py])
        return pixels, valid   # type: ignore[return-value]

    def _fuse(self) -> list:
        """
        Merge camera + LiDAR into [(x, y, color), ...] in TARGET_FRAME.

        Two matching modes depending on camera cone frame_id:
          'camera_pixel': pixel-space matching — LiDAR centroids are projected to image
                          and matched to camera detections (pixel centers) by pixel distance.
          other frames:   3D metric matching — camera cones transformed to TARGET_FRAME
                          and matched by Euclidean distance.
        """
        lidar_xy = self._transform_cloud(self._lidar_cloud) if self._lidar_cloud is not None \
            else np.zeros((0, 2))

        # Separate pixel-mode and metric-mode camera cones
        pixel_cones = [c for c in self._camera_cones if c.header.frame_id == 'camera_pixel']
        metric_cones = [c for c in self._camera_cones if c.header.frame_id != 'camera_pixel']

        camera_metric = self._transform_cones(metric_cones)

        if len(lidar_xy) == 0:
            return camera_metric

        # --- metric camera matching (3D distance) ---
        used_cam_metric: set[int] = set()
        fused = []
        cam_xy_metric = np.array([(x, y) for x, y, _ in camera_metric]) \
            if camera_metric else np.zeros((0, 2))

        lidar_colors = [UNKNOWN] * len(lidar_xy)

        for i, (lx, ly) in enumerate(lidar_xy):
            if len(cam_xy_metric) > 0:
                dists = np.hypot(cam_xy_metric[:, 0] - lx, cam_xy_metric[:, 1] - ly)
                best = int(np.argmin(dists))
                if dists[best] <= CAMERA_MATCH_R:
                    lidar_colors[i] = camera_metric[best][2]
                    used_cam_metric.add(best)

        # --- pixel-mode camera matching (project LiDAR → image) ---
        if pixel_cones and len(lidar_xy) > 0:
            proj_result = self._project_lidar_to_pixels(lidar_xy)
            if isinstance(proj_result, tuple) and len(proj_result) == 2:
                pixels, valid_mask = proj_result
                if len(pixels) > 0:
                    cam_pixels = np.array([[c.location.x, c.location.y] for c in pixel_cones])
                    cam_colors_px = [c.color for c in pixel_cones]
                    used_cam_px: set[int] = set()

                    valid_indices = np.where(valid_mask)[0]
                    for proj_i, lidar_i in enumerate(valid_indices):
                        if lidar_colors[lidar_i] != UNKNOWN:
                            continue   # already coloured by metric match
                        dists = np.hypot(cam_pixels[:, 0] - pixels[proj_i, 0],
                                         cam_pixels[:, 1] - pixels[proj_i, 1])
                        best = int(np.argmin(dists))
                        if dists[best] <= PIXEL_MATCH_R:
                            lidar_colors[lidar_i] = cam_colors_px[best]
                            used_cam_px.add(best)

        for (lx, ly), color in zip(lidar_xy, lidar_colors):
            fused.append((float(lx), float(ly), color))

        # append unmatched metric camera cones
        for i, cone in enumerate(camera_metric):
            if i not in used_cam_metric:
                fused.append(cone)

        return fused

    # ------------------------------------------------------------------ #
    # Evaluation
    # ------------------------------------------------------------------ #

    def _evaluate(self):
        gt    = self._gt_cones
        fused = self._fuse()

        if not gt:
            self.get_logger().warn('Waiting for GT cones (/ground_truth/cones_colored)...',
                                   throttle_duration_sec=10.0)
            return

        used_gt: set[int] = set()
        color_correct = 0
        color_checked = 0

        # Greedy nearest-neighbour matching of fused → GT
        for dx, dy, det_col in fused:
            best_d, best_i = float('inf'), -1
            for i, (gx, gy, _) in enumerate(gt):
                if i in used_gt:
                    continue
                d = math.hypot(dx - gx, dy - gy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_i >= 0 and best_d <= EVAL_RADIUS:
                used_gt.add(best_i)
                gt_col = gt[best_i][2]
                if det_col != UNKNOWN:
                    color_checked += 1
                    if det_col == gt_col:
                        color_correct += 1

        detected = len(used_gt)
        total_gt = len(gt)
        loc_pct  = 100.0 * detected / total_gt       if total_gt      > 0 else 0.0
        col_pct  = 100.0 * color_correct / color_checked if color_checked > 0 else 0.0
        combined = (loc_pct + col_pct) / 2.0

        W = 46
        self.get_logger().info(
            f'\n'
            f'┌{"─"*W}┐\n'
            f'│{"  Perception Fusion Evaluation":^{W}}│\n'
            f'├{"─"*W}┤\n'
            f'│  GT cones         : {total_gt:<5}                    │\n'
            f'│  Fused detections : {len(fused):<5}                    │\n'
            f'│  Matched to GT    : {detected:<5} / {total_gt:<5}              │\n'
            f'│  Color checked    : {color_checked:<5} (non-UNKNOWN dets) │\n'
            f'│  Color correct    : {color_correct:<5}                    │\n'
            f'├{"─"*W}┤\n'
            f'│  Location score   : {loc_pct:>6.1f}%                  │\n'
            f'│  Color score      : {col_pct:>6.1f}%                  │\n'
            f'│  COMBINED SCORE   : {combined:>6.1f}%                  │\n'
            f'└{"─"*W}┘'
        )

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name  = 'fusion_perception_accuracy'
        status.level = DiagnosticStatus.OK if combined >= 70.0 else DiagnosticStatus.WARN
        status.values = [
            KeyValue(key='location_pct',  value=f'{loc_pct:.1f}'),
            KeyValue(key='color_pct',     value=f'{col_pct:.1f}'),
            KeyValue(key='combined_pct',  value=f'{combined:.1f}'),
            KeyValue(key='gt_total',      value=str(total_gt)),
            KeyValue(key='detected',      value=str(detected)),
            KeyValue(key='color_correct', value=str(color_correct)),
            KeyValue(key='color_checked', value=str(color_checked)),
            KeyValue(key='fused_cones',   value=str(len(fused))),
        ]
        diag.status = [status]
        self._pub.publish(diag)

        with open(self._csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.time(),
                f'{loc_pct:.2f}', f'{col_pct:.2f}', f'{combined:.2f}',
                total_gt, detected, color_correct, color_checked, len(fused),
            ])


def main(args=None):
    rclpy.init(args=args)
    node = FusionEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
