"""Lap times for one track: old vs new run bag, from /ground_truth/state_odom (bag receive time).

  python3 lap_compare.py <track> <old_bag> <new_bag> [old_label] [new_label]

Start line = line through the first position, perpendicular to the first heading (taken from the first point
> 1 m away). A crossing is lon going from <0 to >=0 with |lat| <= 4 m, after >= 30 m travelled since the previous
crossing (or since the start for lap 1). Lap 1 is timed from the first motion. Lap count: $NUM_LAPS (default 5).
Prints a markdown table (per-lap times, total, mean and max speed) and the change in total time.
"""
import os, sys, math, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LAP_N = int(os.environ.get("NUM_LAPS", "5"))
LAP_LINE_R, LAP_MIN_D = 4.0, 30.0
MOVE_V, MEAN_V = 0.3, 0.5
TOPIC = "/ground_truth/state_odom"


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    assert TOPIC in types, "no %s in %s" % (TOPIC, bag)
    cls = get_message(types[TOPIC])
    ts, xy, sp = [], [], []
    while r.has_next():
        topic, d, ns = r.read_next()
        if topic != TOPIC:
            continue
        m = deserialize_message(d, cls)
        ts.append(ns * 1e-9)
        xy.append((m.pose.pose.position.x, m.pose.pose.position.y))
        sp.append(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
    return np.array(ts), np.array(xy, float), np.array(sp)


def crossings(xy):
    s = xy[0]
    rel = xy - s
    d = np.hypot(rel[:, 0], rel[:, 1])
    j = int(np.argmax(d > 1.0))
    assert d[j] > 1.0, "car never moved more than 1 m from start"
    h = rel[j] / d[j]
    perp = np.array([-h[1], h[0]])
    lon, lat = rel @ h, rel @ perp
    arc = np.cumsum(np.r_[0.0, np.hypot(*np.diff(xy, axis=0).T)])
    out, last_arc = [], 0.0
    for i in range(1, len(xy)):
        if lon[i - 1] < 0 <= lon[i] and abs(lat[i]) <= LAP_LINE_R and arc[i] - last_arc >= LAP_MIN_D:
            out.append(i)
            last_arc = arc[i]
    return out


def stats(bag, name):
    ts, xy, sp = load(bag)
    idx = crossings(xy)
    t_cross = [ts[i] for i in idx]
    t_move = ts[int(np.nonzero(sp > MOVE_V)[0][0])]
    c = t_cross[:LAP_N]
    print("%s crossings rel. bag start: %s" % (name, np.round(np.array(t_cross) - ts[0], 2).tolist()))
    print("%s first motion rel. bag start: %.2f s" % (name, t_move - ts[0]))
    if len(c) < LAP_N:
        print("WARNING %s: only %d crossings (need %d)" % (name, len(c), LAP_N))
        return None
    lap = [c[0] - t_move] + [c[k] - c[k - 1] for k in range(1, LAP_N)]
    total = c[LAP_N - 1] - t_move
    window = (ts >= t_move) & (ts <= c[LAP_N - 1])
    avg = float(sp[window & (sp > MEAN_V)].mean())
    mx = float(sp[window].max())
    return dict(lap=lap, total=total, avg=avg, max=mx)


def fmt_row(track, label, st):
    laps = " | ".join("%.2f s" % x for x in st["lap"])
    return "| %s | %s | %s | %.2f s | %.2f m/s | %.2f m/s |" % (track, label, laps, st["total"], st["avg"], st["max"])


def main():
    if len(sys.argv) < 4:
        print(__doc__); sys.exit(2)
    track, old_bag, new_bag = sys.argv[1:4]
    old_label = sys.argv[4] if len(sys.argv) > 4 else "old"
    new_label = sys.argv[5] if len(sys.argv) > 5 else "new"
    head = "| Track | Build | " + " | ".join("Lap %d" % (k + 1) for k in range(LAP_N)) + \
           " | Total (%d laps) | Avg speed | Max speed |" % LAP_N
    lines = [head, "|" + "---|" * (LAP_N + 5)]
    old = stats(old_bag, old_label); new = stats(new_bag, new_label)
    for label, st in ((old_label, old), (new_label, new)):
        if st is None:
            lines.append("| %s | %s | n/a |" % (track, label))
        else:
            lines.append(fmt_row(track, label, st))
    if old and new:
        lines.append("")
        lines.append("- %s: total %d-lap time %.2f s (%s) -> %.2f s (%s), change %+.1f%%"
                     % (track, LAP_N, old["total"], old_label, new["total"], new_label,
                        100 * (new["total"] - old["total"]) / old["total"]))
    print()
    print("\n".join(lines))


if __name__ == "__main__":
    main()
