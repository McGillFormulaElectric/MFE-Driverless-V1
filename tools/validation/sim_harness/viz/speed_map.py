"""Speed-by-location figure for one track: old vs new run, shared speed colour scale.

  python3 speed_map.py <track> <old_bag> <new_bag> <out.png> [old_label] [new_label]

Bags are run-bag directories (ground truth /ground_truth/state_odom). Only samples while moving are drawn, from
first to last motion. Loading, cone CSV parsing and the car_start-relative -> map transform follow
score/plot_trajectory.py.
"""
import os, sys, csv, math, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

EUFS_TRACKS_CSV = os.environ.get("EUFS_TRACKS_CSV", os.path.join(os.environ.get("EUFS_WS", os.path.expanduser("~/Develop/MFE26-eufs-sim")), "eufs_tracks", "csv"))


def load_run(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    xy, spd = [], []
    while r.has_next():
        t, d, stamp = r.read_next()
        if t != "/ground_truth/state_odom":
            continue
        m = deserialize_message(d, types[t])
        xy.append((m.pose.pose.position.x, m.pose.pose.position.y))
        spd.append(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
    return np.array(xy), np.array(spd)


def process(track, bag):
    rows = list(csv.DictReader(open(os.path.join(EUFS_TRACKS_CSV, "%s.csv" % track))))
    cs = [r for r in rows if r["tag"] == "car_start"][0]
    X0, Y0, YAW = float(cs["x"]), float(cs["y"]), float(cs["direction"])
    cr, sr = np.cos(YAW), np.sin(YAW)
    to_map = lambda xy: np.c_[X0 + cr * xy[:, 0] - sr * xy[:, 1], Y0 + sr * xy[:, 0] + cr * xy[:, 1]]
    COL = {"blue": "tab:blue", "yellow": "gold", "orange": "orange", "big_orange": "orangered"}
    cones = np.array([(float(r["x"]), float(r["y"])) for r in rows if r["tag"] in COL])

    xy, spd = load_run(bag)
    xy = to_map(xy)
    moving = np.where(spd > 0.3)[0]
    i0, i1 = moving[0], moving[-1]
    return cones, xy[i0:i1 + 1], spd[i0:i1 + 1]


def stats(spd):
    m = spd[spd > 0.5]
    return m.mean(), m.max()


def main():
    if len(sys.argv) < 5:
        print(__doc__); sys.exit(2)
    track, old_bag, new_bag, out = sys.argv[1:5]
    old_label = sys.argv[5] if len(sys.argv) > 5 else "old"
    new_label = sys.argv[6] if len(sys.argv) > 6 else "new"

    cones, old_xy, old_v = process(track, old_bag)
    _, new_xy, new_v = process(track, new_bag)
    print(track, old_label, "samples", len(old_v), "avg %.2f max %.2f" % stats(old_v),
          "|", new_label, "samples", len(new_v), "avg %.2f max %.2f" % stats(new_v))

    vmax = math.ceil(max(old_v.max(), new_v.max()))
    allxy = np.vstack([cones, old_xy, new_xy])
    pad = 5
    x0, x1 = allxy[:, 0].min() - pad, allxy[:, 0].max() + pad
    y0, y1 = allxy[:, 1].min() - pad, allxy[:, 1].max() + pad
    # equal aspect: expand the shorter axis so both panels share identical limits
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    half = max(x1 - x0, y1 - y0) / 2
    xl, yl = (cx - half, cx + half), (cy - half, cy + half)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5), dpi=100, facecolor="white")
    norm = plt.Normalize(vmin=0, vmax=vmax)
    lc = None
    panels = [("Before: %s" % old_label, old_xy, old_v), ("After: %s" % new_label, new_xy, new_v)]
    for ax, (title, xy, v) in zip(axes, panels):
        ax.set_facecolor("white")
        ax.scatter(cones[:, 0], cones[:, 1], s=4, c="0.6", zorder=1)
        pts = xy.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap="turbo", norm=norm, linewidths=3, zorder=3)
        lc.set_array(v[:-1])
        ax.add_collection(lc)
        # start marker: small black triangle at the first sample, oriented along the first segment
        d = xy[min(5, len(xy) - 1)] - xy[0]
        ang = math.atan2(d[1], d[0])
        tri = np.array([[1.0, 0.0], [-0.6, 0.6], [-0.6, -0.6]])
        R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        ax.add_patch(Polygon(xy[0] + 1.6 * tri @ R.T, closed=True, color="black", zorder=5))
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(title, fontsize=12, pad=26)
        avg, mx = stats(v)
        ax.text(0.5, 1.01, "avg %.1f m/s, max %.1f m/s" % (avg, mx), transform=ax.transAxes,
                ha="center", va="bottom", fontsize=10)
    fig.subplots_adjust(left=0.02, right=0.9, wspace=0.08, top=0.85, bottom=0.03)
    cax = fig.add_axes([0.92, 0.12, 0.015, 0.7])
    cb = fig.colorbar(lc, cax=cax)
    cb.set_label("speed (m/s)")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=100, facecolor="white")
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    main()
