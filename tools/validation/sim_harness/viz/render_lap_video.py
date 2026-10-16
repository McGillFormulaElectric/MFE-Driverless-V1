"""Real-time top-down video from a run bag.

  python3 render_lap_video.py <bag_dir> <out.mp4> [track=its_a_mess] [speed=1] [label]

Title: "<track> — <label>" (label from argv[5] or $VIDEO_LABEL). Lap count from $NUM_LAPS (default 5). Needs ffmpeg.

Overlays: true cones (dim gray), LiDAR scan, SLAM landmarks from /planning/cones, true path (green),
SLAM pose trail (red dashed), planned path /planning/centerline (magenta, most recent message),
car marker, HUD with sim time and speed (|twist| from /ground_truth/state_odom).
Frames are sampled at FPS on the sim clock (header stamps): speed=1 is real time.
"""
import os, sys, csv, bisect, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

bag, out = sys.argv[1], sys.argv[2]
track = sys.argv[3] if len(sys.argv) > 3 else "its_a_mess"
speed = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
LABEL = sys.argv[5] if len(sys.argv) > 5 else os.environ.get("VIDEO_LABEL", "")
FPS = 20
TITLE = track + (" — " + LABEL if LABEL else "")
MAX_S = float(os.environ.get("MAX_S", "0"))  # optional cap on rendered sim seconds (testing)
LAP_N, LAP_LINE_R, LAP_MIN_D = int(os.environ.get("NUM_LAPS", "5")), 4.0, 30.0  # laps total, start-line half-width (m), min distance between laps (m)
TRAIL_S = 15.0  # thick recent true-path trail (s)

EUFS_TRACKS_CSV = os.environ.get("EUFS_TRACKS_CSV", os.path.join(os.environ.get("EUFS_WS", os.path.expanduser("~/Develop/MFE26-eufs-sim")), "eufs_tracks", "csv"))
rows = list(csv.DictReader(open(os.path.join(EUFS_TRACKS_CSV, "%s.csv" % track))))
cs = [r for r in rows if r["tag"] == "car_start"][0]
X0, Y0, YAW = float(cs["x"]), float(cs["y"]), float(cs["direction"])
cr, sr = np.cos(YAW), np.sin(YAW)
to_map = lambda xy: np.c_[X0 + cr * xy[:, 0] - sr * xy[:, 1], Y0 + sr * xy[:, 0] + cr * xy[:, 1]]
COL = {"blue": "tab:blue", "yellow": "gold", "orange": "orange", "big_orange": "orangered"}
cones = np.array([(float(r["x"]), float(r["y"])) for r in rows if r["tag"] in COL])

TOPICS = {"/ground_truth/state_odom", "/slam/odom", "/planning/centerline", "/planning/cones", "/lidar/points_raw",
          "/planning/laps_completed"}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}

def stamp(m, bag_ns):
    # header stamps are not usable: /planning/centerline carries wall-clock stamps while odom/lidar carry
    # sim time, and mfe_msgs/Track has no header. Bag receive time is one consistent clock.
    return bag_ns * 1e-9

ts = {k: [] for k in ("state", "slam", "path", "land", "lidar", "laps")}
vs = {k: [] for k in ts}
pose = None
while r.has_next():
    topic, d, bag_ns = r.read_next()
    if topic not in TOPICS: continue
    m = deserialize_message(d, types[topic])
    if topic == "/planning/laps_completed":
        ts["laps"].append(stamp(m, bag_ns)); vs["laps"].append(int(m.data))
    elif topic == "/ground_truth/state_odom":
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        sp = float(np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
        ts["state"].append(stamp(m, bag_ns)); vs["state"].append((pose, sp))
    elif topic == "/slam/odom":
        ts["slam"].append(stamp(m, bag_ns)); vs["slam"].append((m.pose.pose.position.x, m.pose.pose.position.y))
    elif topic == "/planning/centerline":
        ts["path"].append(stamp(m, bag_ns))
        vs["path"].append(np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses]).reshape(-1, 2))
    elif topic == "/planning/cones":
        ts["land"].append(stamp(m, bag_ns))
        vs["land"].append(np.array([(c.location.x, c.location.y, c.color) for c in m.track], float).reshape(-1, 3))
    elif topic == "/lidar/points_raw" and pose is not None:
        p = point_cloud2.read_points(m, field_names=("x", "y"), skip_nans=True)
        ts["lidar"].append(stamp(m, bag_ns))
        vs["lidar"].append((np.c_[p["x"], p["y"]].astype(float), pose))
print({k: len(v) for k, v in ts.items()})
assert ts["state"], "no ground-truth odom in bag"

t0, t1 = ts["state"][0], ts["state"][-1]
if MAX_S > 0: t1 = min(t1, t0 + MAX_S)
def latest(k, t):
    i = bisect.bisect_right(ts[k], t) - 1
    return vs[k][max(i, 0)] if i >= 0 and vs[k] else None

# SLAM trail and true path in map frame, full history up to each frame
gt_all = to_map(np.array([v[0][:2] for v in vs["state"]]))
slam_all = to_map(np.array(vs["slam"])) if vs["slam"] else np.zeros((0, 2))
slam_t = np.array(ts["slam"]); gt_t = np.array(ts["state"])

# Lap count. Prefer /planning/laps_completed if recorded; otherwise detect start-line crossings on the
# ground-truth path: forward crossing of the line through the start pose (perpendicular to the initial
# heading), within LAP_LINE_R metres sideways, with >= LAP_MIN_D metres driven since the previous lap.
if ts["laps"]:
    lap_src = "topic /planning/laps_completed"
    lap_t = np.array(ts["laps"]); lap_n = np.array(vs["laps"])
    def laps_done(t):
        i = bisect.bisect_right(ts["laps"], t) - 1
        return int(vs["laps"][i]) if i >= 0 else 0
else:
    lap_src = "computed from ground-truth start-line crossings"
    xy = np.array([v[0][:2] for v in vs["state"]], float)
    (x0, y0, yaw0) = vs["state"][0][0]
    dx, dy = np.cos(yaw0), np.sin(yaw0)
    sa = (xy[:, 0] - x0) * dx + (xy[:, 1] - y0) * dy      # along heading
    la = -(xy[:, 0] - x0) * dy + (xy[:, 1] - y0) * dx     # sideways
    seg = np.r_[0.0, np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))]
    odo = np.cumsum(seg)
    crossings, last_odo = [], 0.0
    for i in range(1, len(xy)):
        if sa[i - 1] < 0 <= sa[i] and abs(la[i]) <= LAP_LINE_R and odo[i] - last_odo >= LAP_MIN_D:
            crossings.append(gt_t[i]); last_odo = odo[i]
    crossings = np.array(crossings)
    print("lap crossings (sim s):", np.round(crossings - t0, 2).tolist())
    def laps_done(t):
        return int(np.searchsorted(crossings, t, side="right"))

lim = (cones[:, 0].min() - 8, cones[:, 0].max() + 8, cones[:, 1].min() - 8, cones[:, 1].max() + 8)
fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
ax = fig.add_axes([0.04, 0.06, 0.94, 0.86])
fig.suptitle(TITLE, fontsize=15, y=0.97)
w = FFMpegWriter(fps=FPS, extra_args=["-pix_fmt", "yuv420p", "-vcodec", "libx264"])
n_frames = int((t1 - t0) * FPS / speed)
with w.saving(fig, out, 100):
    for k in range(n_frames):
        tf = t0 + k * speed / FPS
        ax.clear(); ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3])
        ax.set_xticks([]); ax.set_yticks([])
        ax.scatter(cones[:, 0], cones[:, 1], s=14, c="dimgray", alpha=.6, label="true cones")
        (px, py, yaw), sp = latest("state", tf)
        nst = bisect.bisect_right(gt_t.tolist(), tf)
        if nst > 1:
            # full history thin and faded; last TRAIL_S seconds thick so overlapping laps stay readable
            ax.plot(gt_all[:nst, 0], gt_all[:nst, 1], "-", c="green", lw=0.8, alpha=0.3, label="true path")
            nrec = bisect.bisect_left(gt_t, tf - TRAIL_S)
            if nst - nrec > 1:
                ax.plot(gt_all[nrec:nst, 0], gt_all[nrec:nst, 1], "-", c="green", lw=3, alpha=0.9,
                        label="true path (last %d s)" % TRAIL_S)
        nsl = bisect.bisect_right(slam_t.tolist(), tf)
        if nsl > 1: ax.plot(slam_all[:nsl, 0], slam_all[:nsl, 1], "r--", lw=1, label="SLAM pose")
        lidar = latest("lidar", tf)
        if lidar is not None:
            p, (lx, ly, lyaw) = lidar
            c, s = np.cos(lyaw), np.sin(lyaw)
            pm = to_map(np.c_[lx + c * p[:, 0] - s * p[:, 1], ly + s * p[:, 0] + c * p[:, 1]]) if len(p) else np.zeros((0, 2))
            ax.scatter(pm[:, 0], pm[:, 1], s=1, c="gray", label="LiDAR")
        land = latest("land", tf)
        if land is not None and len(land):
            lm = to_map(land[:, :2])
            lc = [{0: "lightskyblue", 1: "khaki"}.get(int(k2), "silver") for k2 in land[:, 2]]
            ax.scatter(lm[:, 0], lm[:, 1], marker="D", c=lc, edgecolors="k", linewidths=.6, s=28, label="SLAM landmarks")
        path = latest("path", tf)
        if path is not None and len(path) > 1:
            pm = to_map(path)
            ax.plot(pm[:, 0], pm[:, 1], "-", c="magenta", lw=2.5, label="planned path", zorder=5)
        cm = to_map(np.array([[px, py]]))[0]
        ax.plot(cm[0], cm[1], "o", c="red", ms=8, zorder=6)
        nl = laps_done(tf)
        lap_txt = "finished" if nl >= LAP_N else "lap %d/%d" % (nl + 1, LAP_N)
        ax.text(0.012, 0.975, "sim t = %6.2f s\nspeed = %5.2f m/s\n%s" % (tf - t0, sp, lap_txt), transform=ax.transAxes,
                va="top", ha="left", family="monospace", fontsize=12,
                bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="0.4", alpha=.9), zorder=10)
        ax.legend(loc="upper right", fontsize=8, framealpha=.9)
        w.grab_frame()
print("lap source:", lap_src)
print("frames", n_frames, "duration %.2f s" % (n_frames / FPS), "wrote", out)
