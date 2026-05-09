#!/usr/bin/env python3
"""
Perception accuracy evaluator.

Compares detected cones (/planning/cones) against ground truth (/ground_truth/cones_colored).
Publishes metrics to /perception/accuracy as a DiagnosticArray at 1 Hz.
Also logs a CSV row per evaluation to ~/mfe_logs/perception_accuracy.csv.
"""
import os, csv, math, time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mfe_msgs.msg import Track

MATCH_RADIUS = 1.5  # metres — max dist to consider a cone "detected"


class PerceptionEvaluator(Node):
    def __init__(self):
        super().__init__('perception_evaluator')
        self._gt_cones = []    # list of (x,y,color)
        self._det_cones = []   # list of (x,y,color)

        self.create_subscription(Track, '/ground_truth/cones_colored', self._gt_cb, 10)
        self.create_subscription(Track, '/planning/cones', self._det_cb, 10)
        self._pub = self.create_publisher(DiagnosticArray, '/perception/accuracy', 10)
        self.create_timer(1.0, self._evaluate)

        log_dir = os.path.expanduser('~/mfe_logs')
        os.makedirs(log_dir, exist_ok=True)
        self._csv_path = os.path.join(log_dir, 'perception_accuracy.csv')
        if not os.path.exists(self._csv_path):
            with open(self._csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'precision', 'recall', 'f1', 'mean_dist_m', 'tp', 'fp', 'fn'])

        self.get_logger().info('PerceptionEvaluator started.')

    def _gt_cb(self, msg: Track):
        self._gt_cones = [(c.location.x, c.location.y, c.color) for c in msg.track]

    def _det_cb(self, msg: Track):
        self._det_cones = [(c.location.x, c.location.y, c.color) for c in msg.track]

    def _evaluate(self):
        gt = self._gt_cones
        det = self._det_cones
        used_gt = set()
        tp_dists = []
        fp = 0

        for dx, dy, _ in det:
            best_d, best_i = float('inf'), -1
            for i, (gx, gy, _) in enumerate(gt):
                if i in used_gt: continue
                d = math.hypot(dx - gx, dy - gy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_i >= 0 and best_d <= MATCH_RADIUS:
                used_gt.add(best_i)
                tp_dists.append(best_d)
            else:
                fp += 1

        tp = len(tp_dists)
        fn = len(gt) - len(used_gt)
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = 2*precision*recall/(precision+recall) if (precision+recall) > 0 else 0.0
        mean_d = sum(tp_dists)/len(tp_dists) if tp_dists else 0.0

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'perception_accuracy'
        status.level = DiagnosticStatus.OK if recall > 0.7 else DiagnosticStatus.WARN
        status.values = [
            KeyValue(key='precision', value=f'{precision:.3f}'),
            KeyValue(key='recall',    value=f'{recall:.3f}'),
            KeyValue(key='f1',        value=f'{f1:.3f}'),
            KeyValue(key='mean_dist_m', value=f'{mean_d:.3f}'),
            KeyValue(key='TP', value=str(tp)),
            KeyValue(key='FP', value=str(fp)),
            KeyValue(key='FN', value=str(fn)),
            KeyValue(key='gt_total', value=str(len(gt))),
            KeyValue(key='det_total', value=str(len(det))),
        ]
        msg.status = [status]
        self._pub.publish(msg)

        with open(self._csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([time.time(), f'{precision:.4f}', f'{recall:.4f}', f'{f1:.4f}', f'{mean_d:.4f}', tp, fp, fn])

        self.get_logger().info(
            f'Accuracy — P:{precision:.2f} R:{recall:.2f} F1:{f1:.2f} mean_dist:{mean_d:.2f}m | TP:{tp} FP:{fp} FN:{fn}',
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
