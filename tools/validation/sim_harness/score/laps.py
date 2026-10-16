"""Count skidpad laps from a run bag using the ground-truth heading.

  python3 laps.py <bag>   ->  prints: left_laps right_laps exited

Left (CCW) laps = total positive heading change / 2pi while moving; right (CW) laps likewise for negative.
Frame-free, so it doesn't depend on where the track sits in the map. 'exited' is 1 if the car ends
more than 15 m from where it first turned, i.e. it left the circles along the exit lane.
"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/state_odom"]))
xy, yaw, v = [], [], []
while r.has_next():
    _, d, _ = r.read_next()
    m = deserialize_message(d, types["/ground_truth/state_odom"])
    q = m.pose.pose.orientation
    xy.append((m.pose.pose.position.x, m.pose.pose.position.y))
    yaw.append(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    v.append(np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
if len(yaw) < 10:
    print("nan nan 0"); sys.exit()
xy, yaw, v = np.array(xy), np.unwrap(np.array(yaw)), np.array(v)
d = np.diff(yaw)
moving = v[1:] > 0.5
left = d[moving & (d > 0)].sum() / (2 * np.pi)
right = -d[moving & (d < 0)].sum() / (2 * np.pi)
turn_idx = np.argmax(np.abs(np.cumsum(d)) > 0.5)  # first real turn
# >8 m: the stop zone starts 9 m past the crossing (first turn is ~1-2 m after it). It was 15 m while
# the car stopped beyond the end cones (~23 m); since finish_x 30 it stops ~16 m past the crossing.
exited = int(np.linalg.norm(xy[-1] - xy[turn_idx]) > 8)
import os
if os.environ.get('EXIT_DIST'): print('end distance from first turn', round(float(np.linalg.norm(xy[-1] - xy[turn_idx])), 2), file=sys.stderr)
print(f"{left:.2f} {right:.2f} {exited}")
