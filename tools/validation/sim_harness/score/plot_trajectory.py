"""Static top-down trajectory plot from a lidar-only run bag (the bag written by run_sim_test.sh / run_batch.sh).

  python3 plot_trajectory.py <bag_dir> <out.png> <track> [title]

Shows: true cones (from track CSV, GT used for drawing only), the car's true (ground-truth)
path, and the SLAM-estimated pose path, so SLAM tracking error is visible directly on the plot.
Same coordinate convention and cone-loading as eval/render_video.py (car-start-relative local
frame -> map frame), just a single static frame of the full path instead of an animation.
"""
import os, sys, csv, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out, track = sys.argv[1], sys.argv[2], sys.argv[3]
title = sys.argv[4] if len(sys.argv) > 4 else "LiDAR + GTSAM SLAM only"
EUFS_TRACKS = os.environ.get("EUFS_TRACKS_CSV", os.path.join(os.environ.get("EUFS_WS", os.path.expanduser("~/Develop/MFE26-eufs-sim")), "eufs_tracks", "csv"))

rows = list(csv.DictReader(open(os.path.join(EUFS_TRACKS, "%s.csv" % track))))
cs = [r for r in rows if r["tag"] == "car_start"][0]
X0, Y0, YAW = float(cs["x"]), float(cs["y"]), float(cs["direction"])
cr, sr = np.cos(YAW), np.sin(YAW)
to_map = lambda xy: np.c_[X0 + cr * xy[:, 0] - sr * xy[:, 1], Y0 + sr * xy[:, 0] + cr * xy[:, 1]]
COL = {"blue": "tab:blue", "yellow": "gold", "orange": "orange", "big_orange": "orangered"}
cones = [(float(r["x"]), float(r["y"]), COL[r["tag"]]) for r in rows if r["tag"] in COL]

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
gt, slam, gt_t, slam_t = [], [], [], []
t0 = None
while r.has_next():
    t, d, stamp = r.read_next()
    if t0 is None:
        t0 = stamp
    m = deserialize_message(d, types[t])
    if t == "/ground_truth/state_odom":
        gt.append((m.pose.pose.position.x, m.pose.pose.position.y))
        gt_t.append((stamp - t0) * 1e-9)
    elif t == "/slam/odom":
        slam.append((m.pose.pose.position.x, m.pose.pose.position.y))
        slam_t.append((stamp - t0) * 1e-9)
print(len(gt), "GT samples,", len(slam), "SLAM samples")

gt = to_map(np.array(gt)) if gt else np.zeros((0, 2))
slam = to_map(np.array(slam)) if slam else np.zeros((0, 2))
cx = np.array([[c[0], c[1]] for c in cones])
pad = 8
lim = (cx[:, 0].min() - pad, cx[:, 0].max() + pad, cx[:, 1].min() - pad, cx[:, 1].max() + pad)
fig, ax = plt.subplots(figsize=(8, 8) if lim[1] - lim[0] < 2 * (lim[3] - lim[2]) else (10, 5), dpi=120)
ax.set_aspect("equal")
ax.set_xlim(lim[0], lim[1])
ax.set_ylim(lim[2], lim[3])

ax.scatter([c[0] for c in cones], [c[1] for c in cones], c=[c[2] for c in cones], s=24, alpha=.5, label="true cones")

if len(gt):
    # colour the GT path by elapsed time so "location by time" is visible on a single static frame
    sc = ax.scatter(gt[:, 0], gt[:, 1], c=gt_t, cmap="viridis", s=18, zorder=5, label="true path (colour = time)")
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("sim time (s)")
if len(slam):
    ax.plot(slam[:, 0], slam[:, 1], "r--", lw=1.5, zorder=6, label="SLAM pose")

# Inset: zoomed view centred on the actual path, since on long/large tracks (accel's
# ~100m straight, autocross-mode peanut) the car's travel in one fixed-duration
# recording can be a small fraction of the full cone field and is hard to read at
# full-track zoom — confirmed 2026-10-11 on the accel plot.
if len(gt) > 1:
    gx0, gx1 = gt[:, 0].min(), gt[:, 0].max()
    gy0, gy1 = gt[:, 1].min(), gt[:, 1].max()
    span = max(gx1 - gx0, gy1 - gy0, 5.0)
    if span < 0.35 * (lim[1] - lim[0]):   # only worth it if the path is small vs. the full track
        cxm, cym = (gx0 + gx1) / 2, (gy0 + gy1) / 2
        ipad = span * 0.6 + 2.0
        axins = ax.inset_axes([0.60, 0.05, 0.38, 0.38])
        axins.set_aspect("equal")
        axins.scatter([c[0] for c in cones], [c[1] for c in cones], c=[c[2] for c in cones], s=24, alpha=.5)
        axins.scatter(gt[:, 0], gt[:, 1], c=gt_t, cmap="viridis", s=18, zorder=5)
        if len(slam):
            axins.plot(slam[:, 0], slam[:, 1], "r--", lw=1.5, zorder=6)
        axins.set_xlim(cxm - ipad, cxm + ipad)
        axins.set_ylim(cym - ipad, cym + ipad)
        axins.set_xticks([]); axins.set_yticks([])
        axins.set_title("zoom on path", fontsize=7)
        ax.indicate_inset_zoom(axins, edgecolor="black")

duration = max(gt_t[-1] if gt_t else 0, slam_t[-1] if slam_t else 0)
ax.set_title("%s: %s   duration=%.1f s" % (track, title, duration))
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
fig.savefig(out)
print("wrote", out)
