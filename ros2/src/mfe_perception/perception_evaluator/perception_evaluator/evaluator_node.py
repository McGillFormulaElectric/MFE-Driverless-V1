#!/usr/bin/env python3
"""
Perception accuracy evaluator.

Compares the full pipeline output (/planning/cones, map frame) against ground truth
cones (/ground_truth/cones_colored, base_footprint frame).  TF2 is used to transform
GT cones into the map frame before position comparison so the distance calculation
is geometrically valid.

Publishes metrics to /perception/accuracy as a DiagnosticArray at 1 Hz.
Also logs a CSV row per evaluation to ~/mfe_logs/perception_accuracy.csv.
"""
import csv
import math
import os
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mfe_msgs.msg import Track
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

MATCH_RADIUS  = 1.5   # metres — max dist to count a cone as detected
TARGET_FRAME  = 'map'  # frame used by boundary_extractor for /planning/cones


def _quat_to_rot3(qx, qy, qz, qw) -> np.ndarray:
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx),   1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


class PerceptionEvaluator(Node):
    def __init__(self):
        super().__init__('perception_evaluator')
        self._gt_cones: list = []   # list of (x, y, color) in self._gt_frame
        self._gt_frame: str = 'base_footprint'
        self._det_cones: list = []  # list of (x, y, color) in map frame

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self.create_subscription(Track, '/ground_truth/cones_colored', self._gt_cb, 10)
        self.create_subscription(Track, '/planning/cones', self._det_cb, 10)
        self._pub = self.create_publisher(DiagnosticArray, '/perception/accuracy', 10)
        self.create_timer(1.0, self._evaluate)

        log_dir = os.path.expanduser('~/mfe_logs')
        os.makedirs(log_dir, exist_ok=True)
        self._csv_path = os.path.join(log_dir, 'perception_accuracy.csv')
        if not os.path.exists(self._csv_path):
            with open(self._csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['timestamp', 'precision', 'recall', 'f1', 'mean_dist_m', 'tp', 'fp', 'fn'])

        self.get_logger().info('PerceptionEvaluator started.')

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _gt_cb(self, msg: Track) -> None:
        if msg.track:
            self._gt_frame = msg.track[0].header.frame_id or 'base_footprint'
        self._gt_cones = [(c.location.x, c.location.y, c.color) for c in msg.track]

    def _det_cb(self, msg: Track) -> None:
        self._det_cones = [(c.location.x, c.location.y, c.color) for c in msg.track]

    # ------------------------------------------------------------------ #
    # Frame transformation                                                 #
    # ------------------------------------------------------------------ #

    def _transform_gt_to_map(self, cones: list) -> list:
        """
        Transform GT cones from self._gt_frame to TARGET_FRAME ('map') using TF2.

        /ground_truth/cones_colored arrives in base_footprint (car-relative) because
        EUFS sim publishes /ground_truth/cones in that frame.  /planning/cones from
        boundary_extractor is in map frame.  Without this transform, distance
        comparisons are meaningless (car coords vs world coords).
        """
        if not cones:
            return cones
        src = self._gt_frame
        if not src or src == TARGET_FRAME:
            return cones
        try:
            tf = self._tf.lookup_transform(TARGET_FRAME, src, rclpy.time.Time())
            R = _quat_to_rot3(
                tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w)
            t = np.array([tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z])
            out = []
            for x, y, color in cones:
                p = R @ np.array([x, y, 0.0]) + t
                out.append((float(p[0]), float(p[1]), color))
            return out
        except TransformException as e:
            self.get_logger().warn(
                f'TF {src!r}→{TARGET_FRAME!r} failed: {e} '
                f'— GT positions not transformed; accuracy numbers will be wrong.',
                throttle_duration_sec=5.0)
            return cones  # degrade gracefully; warn every 5 s

    # ------------------------------------------------------------------ #
    # Evaluation (1 Hz)                                                    #
    # ------------------------------------------------------------------ #

    def _evaluate(self) -> None:
        gt  = self._transform_gt_to_map(self._gt_cones)
        det = self._det_cones

        used_gt: set = set()
        tp_dists: list = []
        fp = 0

        for dx, dy, _ in det:
            best_d, best_i = float('inf'), -1
            for i, (gx, gy, _) in enumerate(gt):
                if i in used_gt:
                    continue
                d = math.hypot(dx - gx, dy - gy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_i >= 0 and best_d <= MATCH_RADIUS:
                used_gt.add(best_i)
                tp_dists.append(best_d)
            else:
                fp += 1

        tp        = len(tp_dists)
        fn        = len(gt) - len(used_gt)
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1        = 2*precision*recall / (precision+recall) if (precision+recall) > 0 else 0.0
        mean_d    = sum(tp_dists) / len(tp_dists) if tp_dists else 0.0

        out_msg = DiagnosticArray()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name  = 'perception_accuracy'
        status.level = DiagnosticStatus.OK if recall > 0.7 else DiagnosticStatus.WARN
        status.values = [
            KeyValue(key='precision',   value=f'{precision:.3f}'),
            KeyValue(key='recall',      value=f'{recall:.3f}'),
            KeyValue(key='f1',          value=f'{f1:.3f}'),
            KeyValue(key='mean_dist_m', value=f'{mean_d:.3f}'),
            KeyValue(key='TP',          value=str(tp)),
            KeyValue(key='FP',          value=str(fp)),
            KeyValue(key='FN',          value=str(fn)),
            KeyValue(key='gt_total',    value=str(len(gt))),
            KeyValue(key='det_total',   value=str(len(det))),
        ]
        out_msg.status = [status]
        self._pub.publish(out_msg)

        with open(self._csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.time(),
                f'{precision:.4f}', f'{recall:.4f}', f'{f1:.4f}', f'{mean_d:.4f}',
                tp, fp, fn,
            ])

        self.get_logger().info(
            f'Accuracy — P:{precision:.2f} R:{recall:.2f} F1:{f1:.2f} '
            f'mean_dist:{mean_d:.2f}m | TP:{tp} FP:{fp} FN:{fn}',
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
