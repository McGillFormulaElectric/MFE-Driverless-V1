"""Helpers shared by the validation nodes."""
import csv
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read_track(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def car_start(rows):
    """(x, y, yaw) of the car_start row of a track CSV, or None."""
    found = None
    for r in rows:
        if r['tag'] == 'car_start':
            found = (float(r['x']), float(r['y']), float(r['direction']))
    return found
