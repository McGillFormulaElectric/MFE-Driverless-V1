"""Acceleration run from GT (car frame at spawn: finish gate x=78, stop zone ends x=103, cones at y=+-1.5).
  python3 accel_check.py <bag>  ->  prints: max_x max_abs_y top_speed_mps t_gate_s(from first movement, -1 if never)"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/state_odom"]))
T, P, V = [], [], []
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, ty["/ground_truth/state_odom"])
    T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    P.append((m.pose.pose.position.x, m.pose.pose.position.y)); V.append(np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
T, P, V = np.array(T), np.array(P), np.array(V)
mv = np.argmax(np.linalg.norm(P - P[0], axis=1) > 0.05); g = np.argmax(P[:, 0] >= 78.0)
tg = T[g] - T[mv] if P[:, 0].max() >= 78.0 else -1
print(f"{P[:,0].max():.1f} {np.abs(P[:,1]).max():.2f} {V.max():.2f} {tg:.2f}")
