"""Effective command-application interval from GT: the plugin holds steering/acceleration between commands,
so yaw rate (and longitudinal accel) change in steps. Report the median spacing between steps while moving.
  python3 hold_rate.py <bag>..."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/state_odom"]))
    s, w, v = [], [], []
    while r.has_next():
        _, d, _ = r.read_next(); m = deserialize_message(d, T["/ground_truth/state_odom"])
        s.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); w.append(m.twist.twist.angular.z)
        v.append(np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
    s, w, v = map(np.array, (s, w, v)); mv = v > 3.0
    s, w = s[mv], w[mv]
    import os
    if os.environ.get("WIN"): keep = s < s[0] + float(os.environ["WIN"]); s, w = s[keep], w[keep]
    if len(s) < 50: print(bag.split("/")[-1], "too short"); continue
    # a "step onset" = yaw-rate change > 0.15 deg/s per sample after a quiet sample
    dw = np.abs(np.diff(np.degrees(w)))
    on = np.where((dw[1:] > 0.15) & (dw[:-1] <= 0.15))[0] + 1
    gaps = np.diff(s[on]); gaps = gaps[gaps > 0.02]
    hist = np.histogram(gaps, bins=[0.02, 0.075, 0.15, 0.3, 0.6, 5])[0]
    print(f"{bag.split('/')[-1]}: step spacing median {np.median(gaps):.3f}s  counts <75ms {hist[0]}, 75-150 {hist[1]}, 150-300 {hist[2]}, 300-600 {hist[3]}, >600 {hist[4]}  (GT {1/np.median(np.diff(s)):.0f} Hz)")
