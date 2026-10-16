"""Did the car stay on a closed track? GT path from a run bag vs the track's cone boundary polygons.
  python3 offtrack.py <bag> <track>  ->  prints: off(0/1) first_off_travel_m laps_before_off pct_on_track
  OFFTRACK_EXACT=1: exact 0.5 m margin both sides (see track_poly.py docstring); default is the legacy rule."""
import sys, os, numpy as np, rosbag2_py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from track_poly import on_track, to_map
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/state_odom"]))
P = []
while r.has_next():
    _, d, _ = r.read_next()
    p = deserialize_message(d, types["/ground_truth/state_odom"]).pose.pose.position
    P.append((p.x, p.y))
P = to_map(np.array(P)[::5], sys.argv[2])   # GT odom is car_start-relative; polygons are in the csv (map) frame
travel = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
ok = on_track(P, sys.argv[2], margin=0.5)   # car centre may be up to 0.5 m past a cone line
run = np.convolve((~ok).astype(int), np.ones(5, int), "same") >= 5
first = float(travel[np.argmax(run)]) if run.any() else -1
# A lap = crossing the start line forwards: position along the start heading goes from behind to past the
# start, within 4 m sideways. "Within 4 m of the start" alone counted laps on peanut runs that stopped
# 4 m short of it (the track passes that close before reaching the start).
s = P[0]; laps = 0; far = False
h = P[np.argmax(travel > 1.0)] - s; h = h / np.linalg.norm(h)    # start heading from the first metre driven
rel = P - s; lon = rel @ h; lat = rel @ np.array([-h[1], h[0]])
for i in range(1, len(P)):
    if first >= 0 and travel[i] >= first: break
    if np.linalg.norm(P[i] - s) > 15: far = True
    if far and lon[i - 1] < 0.0 <= lon[i] and abs(lat[i]) < 4: laps += 1; far = False
print(f"{int(run.any())} {first:.0f} {laps} {100*ok.mean():.0f}")
