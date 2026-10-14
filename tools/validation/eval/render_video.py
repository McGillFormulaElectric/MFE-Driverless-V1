"""Top-down video from a lidar-only run bag (see run_slam_headless.sh with REC=<dir>).

  source /tmp/r.sh; python3 eval/render_video.py test_bag/peanut_lidaronly out.mp4 [track=peanut] [speed=2]

Shows: true cones (dim, from track CSV; GT used for drawing only), LiDAR points, the cones the stack believes
(SLAM landmarks on /planning/cones or /skidpad/cones, light blue/yellow, black line = offset to true cone), the car's true path and the SLAM pose. Frames are one per LiDAR scan (~10 Hz sim time).
"""
import os, sys, csv, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

bag, out = sys.argv[1], sys.argv[2]
track = sys.argv[3] if len(sys.argv) > 3 else "peanut"
speed = int(sys.argv[4]) if len(sys.argv) > 4 else 2
NOLIDAR = bool(os.environ.get('NOLIDAR'))

EUFS_MASTER = os.environ.get("EUFS_MASTER", os.path.expanduser("~/Develop/MFE26-eufs-sim"))
rows = list(csv.DictReader(open("%s/eufs_sim/eufs_tracks/csv/%s.csv" % (EUFS_MASTER, track))))
cs = [r for r in rows if r["tag"] == "car_start"][0]
X0, Y0, YAW = float(cs["x"]), float(cs["y"]), float(cs["direction"])
cr, sr = np.cos(YAW), np.sin(YAW)
to_map = lambda xy: np.c_[X0 + cr * xy[:, 0] - sr * xy[:, 1], Y0 + sr * xy[:, 0] + cr * xy[:, 1]]
COL = {"blue": "tab:blue", "yellow": "gold", "orange": "orange", "big_orange": "orangered"}
cones = [(float(r["x"]), float(r["y"]), COL[r["tag"]]) for r in rows if r["tag"] in COL]

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
gt, slam, land, frames = [], [], None, []
pose = None
while r.has_next():
    t, d, _ = r.read_next()
    m = deserialize_message(d, types[t])
    if t == "/ground_truth/state_odom":
        q = m.pose.pose.orientation
        pose = (m.pose.pose.position.x, m.pose.pose.position.y, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        gt.append(pose[:2])
        if NOLIDAR and len(gt) % 20 == 0:      # ground-truth control runs have no LiDAR: one frame per 20 odom msgs (~10 Hz)
            frames.append((pose, np.zeros((0, 2)), land, len(gt), len(slam)))
    elif t == "/slam/odom":
        slam.append((m.pose.pose.position.x, m.pose.pose.position.y))
    elif t in ("/planning/cones", "/skidpad/cones"):
        land = [(c.location.x, c.location.y, c.color) for c in m.track]
    elif t == "/lidar/points_raw" and pose is not None:
        p = np.array([(a, b) for a, b, z in point_cloud2.read_points(m, ("x", "y", "z"), skip_nans=True)]).reshape(-1, 2)
        frames.append((pose, p, land, len(gt), len(slam)))
print(len(frames), "frames; gt", len(gt), "slam", len(slam))

gt = to_map(np.array(gt))
slam = to_map(np.array(slam)) if slam else np.zeros((0, 2))
cx = np.array([[c[0], c[1]] for c in cones]); pad = 8
lim = (cx[:, 0].min() - pad, cx[:, 0].max() + pad, cx[:, 1].min() - pad, cx[:, 1].max() + pad)
fig, ax = plt.subplots(figsize=(8, 8) if lim[1] - lim[0] < 2 * (lim[3] - lim[2]) else (10, 5), dpi=100)
w = FFMpegWriter(fps=20)  # 20 fps = 2x sim speed when speed=1
with w.saving(fig, out, 100):
    for i, (pose, p, land, ng, ns) in enumerate(frames):
        if i % speed: continue
        ax.clear(); ax.set_aspect("equal"); ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3])
        ax.scatter([c[0] for c in cones], [c[1] for c in cones], c=[c[2] for c in cones], s=18, alpha=.35, label="true cones")
        c, s = np.cos(pose[2]), np.sin(pose[2])
        pm = to_map(np.c_[pose[0] + c * p[:, 0] - s * p[:, 1], pose[1] + s * p[:, 0] + c * p[:, 1]]) if len(p) else np.zeros((0, 2))
        ax.scatter(pm[:, 0], pm[:, 1], s=1, c="gray", label="LiDAR")
        if land:
            la = np.array(land, float); lm = to_map(la[:, :2])
            for a, b in zip(lm, cx[np.linalg.norm(cx[None] - lm[:, None], axis=2).argmin(1)]):   # offset to nearest true cone
                ax.plot([a[0], b[0]], [a[1], b[1]], "-", c="k", lw=.5, alpha=.6)
            lc = [{0: "lightskyblue", 1: "khaki"}.get(int(k), "silver") for k in la[:, 2]]
            ax.scatter(lm[:, 0], lm[:, 1], marker="D", c=lc, edgecolors="k", linewidths=.6, s=28, label="SLAM landmarks (light blue/yellow)")
        ax.plot(gt[:ng, 0], gt[:ng, 1], "g-", lw=1, label="true path")
        if ns: ax.plot(slam[:ns, 0], slam[:ns, 1], "r--", lw=1, label="SLAM pose")
        ax.plot(*to_map(np.array([pose[:2]]))[0], "o", c="red", ms=8)
        ax.set_title("%s: %s   sim t=%.1f s" % (track, os.environ.get("TITLE", "LiDAR + GTSAM only"), i / 10)); ax.legend(loc="upper left", fontsize=7)
        w.grab_frame()
print("wrote", out)
