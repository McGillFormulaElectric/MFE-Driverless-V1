"""Cones the car's footprint actually touched, from the GT path in a run bag.
Footprint: rectangle about the CoG (EUFS 'eufs' car: front axle +0.869 m, rear -0.711 m, wheel r 0.2525, width 1.6 m)
taken as x in [-1.0, +1.4], y in [-0.8, +0.8]; a cone (base radius 0.114 m) is hit if its circle overlaps the rectangle.
  python3 cone_hits.py <bag> <track>  ->  prints: n_hit first_hit_s colours"""
import sys, os, csv, math, numpy as np, rosbag2_py
EUFS_TRACKS_CSV = os.environ.get("EUFS_TRACKS_CSV", os.path.join(os.environ.get("EUFS_WS", os.path.expanduser("~/Develop/MFE26-eufs-sim")), "eufs_tracks", "csv"))
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, track = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(os.path.join(EUFS_TRACKS_CSV, f"{track}.csv"))))
cs = [r for r in rows if r["tag"] == "car_start"][0]
X0, Y0, YAW0 = float(cs["x"]), float(cs["y"]), float(cs["direction"])
C = np.array([(float(r["x"]), float(r["y"])) for r in rows if r["tag"] in ("blue", "yellow", "orange", "big_orange")])
tags = [r["tag"] for r in rows if r["tag"] in ("blue", "yellow", "orange", "big_orange")]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/state_odom"]))
hit = {}; t0 = None
FX, RX, HW, CR = 1.4, -1.0, 0.8, 0.114
while r.has_next():
    _, d, ts = r.read_next(); m = deserialize_message(d, T["/ground_truth/state_odom"])
    s = ts * 1e-9; t0 = t0 or s
    p = m.pose.pose.position; q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    # odom frame -> map frame (odom origin = car_start pose)
    mx = X0 + math.cos(YAW0) * p.x - math.sin(YAW0) * p.y; my = Y0 + math.sin(YAW0) * p.x + math.cos(YAW0) * p.y
    h = YAW0 + yaw
    d_ = C - (mx, my); lx = d_[:, 0] * math.cos(h) + d_[:, 1] * math.sin(h); ly = -d_[:, 0] * math.sin(h) + d_[:, 1] * math.cos(h)
    ex = np.maximum(0, np.maximum(RX - lx, lx - FX)); ey = np.maximum(0, np.abs(ly) - HW)
    for i in np.where(np.hypot(ex, ey) < CR)[0]:
        hit.setdefault(int(i), s - t0)
first = min(hit.values()) if hit else -1
cols = {}
for i in hit: cols[tags[i]] = cols.get(tags[i], 0) + 1
if os.environ.get("WHERE"): print(sorted((round(C[i][0],1), round(C[i][1],1), tags[i], round(t,1)) for i, t in hit.items()))
print(f"{len(hit)} {first:.1f} {','.join(f'{k}:{v}' for k, v in sorted(cols.items())) or '-'}")
