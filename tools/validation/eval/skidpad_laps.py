"""Counts skidpad laps from a recorded bag: winding of the true path around each circle centre. Expect 2 left + 2 right, then exit.

  source /tmp/r.sh; python3 eval/skidpad_laps.py test_bag/vid_skidpad
Odom frame (start-relative): left circle centre (14.4, +9.3), right (14.4, -9.3); the car starts at (0, 0) heading +x.
"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
P = []
while r.has_next():
    t, d, _ = r.read_next()
    if t == "/ground_truth/state_odom":
        m = deserialize_message(d, ty[t]); P.append((m.pose.pose.position.x, m.pose.pose.position.y))
P = np.array(P)
for name, c in (("LEFT ", (14.4, 9.3)), ("RIGHT", (14.4, -9.3))):
    d = P - np.array(c); rad = np.hypot(d[:, 0], d[:, 1]); on = rad < 12.5      # on that circle (path radius 9.1 m)
    ang = np.unwrap(np.arctan2(d[on, 1], d[on, 0]))
    print("%s circle: %5.2f laps (%s)" % (name, abs(ang[-1] - ang[0]) / (2 * np.pi) if len(ang) > 1 else 0.0,
          "counter-clockwise" if len(ang) > 1 and ang[-1] > ang[0] else "clockwise"))
print("final position (%.1f, %.1f), max x %.1f" % (P[-1, 0], P[-1, 1], P[:, 0].max()))
