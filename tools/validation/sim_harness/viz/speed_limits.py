"""Offline speed-limiter analysis for autocross bags (replicates pure_pursuit_node._compute_target_speed).

  python3 speed_limits.py <bag_dir> [<bag_dir> ...]

Per bag: share of samples where max_speed / curvature / path_end is the binding limit, median and p10 remaining
path length, mean ground-truth speed vs mean limit, and curvature-radius medians at the binding corner. Uses
/ground_truth/state_odom, /slam/odom and /planning/centerline from each bag.
"""
import os, sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MAX_SPEED = 8.0
MAX_LAT_ACCEL = 8.0
MAX_DECEL = 10.0
PE_FRAC = 0.6
PE_MARGIN = 1.5
LOOKAHEAD = 40          # lookahead_waypoints default of pure_pursuit_node
MOVING = 0.5
STEP = 0.1
WANT = ('/ground_truth/state_odom', '/slam/odom', '/planning/centerline')


def load(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    gt, odom, cl = [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in WANT:
            continue
        ts = t * 1e-9
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/planning/centerline':
            cl.append((ts, np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])))
        elif topic == '/slam/odom':
            p = msg.pose.pose.position
            odom.append((ts, p.x, p.y))
        else:
            tw = msg.twist.twist.linear
            gt.append((ts, math.hypot(tw.x, tw.y)))
    return (np.array(gt), np.array(odom), cl)


def kappa_code(p0, p1, p2):
    """Exact curvature formula from _compute_target_speed. Returns the radius R, or None."""
    dx1 = p1[0] - p0[0]; dy1 = p1[1] - p0[1]
    dx2 = p2[0] - p1[0]; dy2 = p2[1] - p1[1]
    cross = abs(dx1 * dy2 - dy1 * dx2)
    l1 = math.hypot(dx1, dy1); l2 = math.hypot(dx2, dy2)
    if l1 < 0.01 or l2 < 0.01:
        return None
    kappa = cross / (l1 * l2 * (l1 + l2) / 2.0)
    R = 1.0 / kappa if kappa > 1e-4 else float('inf')
    return R


def v_curv(path, idx, v_now):
    """Replica of _compute_target_speed curvature branch (no planner target_speeds). Returns (v, corner_i)."""
    n = len(path)
    scan_end = min(n, idx + LOOKAHEAD)
    v_target = MAX_SPEED
    dist_to_target = float('inf')
    cum_dist = 0.0
    corner = -1
    for i in range(idx, scan_end - 2):
        if i > idx:
            cum_dist += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        R = kappa_code(path[i], path[i + 1], path[i + 2])
        if R is None:
            continue
        v_corner = min(MAX_SPEED, math.sqrt(MAX_LAT_ACCEL * R))
        if v_corner < v_target:
            v_target = v_corner
            dist_to_target = cum_dist
            corner = i
    d_brake = max(0.0, (v_now ** 2 - v_target ** 2) / (2.0 * MAX_DECEL))
    ramp_dist = max(20.0, d_brake * 5.0)
    if v_target < MAX_SPEED and dist_to_target <= ramp_dist:
        t = dist_to_target / ramp_dist
        return max(0.0, v_target + t * (MAX_SPEED - v_target)), corner
    return MAX_SPEED, corner


def smooth5(P):
    """5-point moving average, edge-padded so index i stays aligned with the raw path."""
    out = np.empty_like(P)
    for k in range(2):
        pad = np.pad(P[:, k], 2, mode='edge')
        out[:, k] = np.convolve(pad, np.ones(5) / 5.0, mode='valid')
    return out


def analyse(bag):
    gt, odom, cl = load(bag)
    gt_t, gt_v = gt[:, 0], gt[:, 1]
    od_t = odom[:, 0]
    cl_t = np.array([c[0] for c in cl])
    rows = []
    t = gt_t[0]
    t_end = gt_t[-1]
    while t <= t_end:
        gi = np.searchsorted(gt_t, t, side='right') - 1
        v_now = gt_v[gi] if gi >= 0 else 0.0
        if v_now > MOVING:
            ci = np.searchsorted(cl_t, t, side='right') - 1
            oi = np.searchsorted(od_t, t, side='right') - 1
            if ci >= 0 and oi >= 0:
                P = cl[ci][1]
                path = P.tolist()
                pos = odom[oi, 1:3]
                idx = int(np.argmin(np.hypot(P[:, 0] - pos[0], P[:, 1] - pos[1])))
                seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
                remaining = float(seg[idx:].sum())
                vc, corner = v_curv(path, idx, v_now)
                v_end = math.sqrt(max(0.0, 2 * PE_FRAC * MAX_DECEL * (remaining - PE_MARGIN)))
                lims = [MAX_SPEED, vc, v_end]
                b = int(np.argmin(lims))
                binding = ('max_speed', 'curvature', 'path_end')[b]
                R_code = R_smooth = R_true = np.nan
                if binding == 'curvature' and corner >= 0:
                    R_code = kappa_code(P[corner], P[corner + 1], P[corner + 2])
                    S = smooth5(P)
                    R_smooth = kappa_code(S[corner], S[corner + 1], S[corner + 2])
                    a = P[corner + 1] - P[corner]; c = P[corner + 2] - P[corner + 1]
                    l1, l2 = math.hypot(*a), math.hypot(*c)
                    l3 = math.hypot(*(P[corner + 2] - P[corner]))
                    cr = abs(a[0] * c[1] - a[1] * c[0])
                    R_true = l1 * l2 * l3 / (2 * cr) if cr > 0 else np.inf
                    R_code = np.nan if R_code == float('inf') else R_code
                    R_smooth = np.nan if R_smooth == float('inf') else R_smooth
                    R_true = np.nan if R_true == float('inf') else R_true
                rows.append((v_now, remaining, min(lims), vc, v_end, binding, R_code, R_smooth, R_true))
        t += STEP
    return rows


def med(x):
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    return float(np.median(x)) if len(x) else float('nan')


def main():
    bags = sys.argv[1:]
    if not bags:
        print(__doc__); sys.exit(2)
    print(f"{'bag':20} {'N':>5} {'%max':>6} {'%curv':>6} {'%end':>6} {'medRem':>7} {'p10Rem':>7} "
          f"{'GTmean':>7} {'vlimMn':>7} {'lim-GT':>7} {'Rcode':>7} {'Rsmooth':>8} {'Rtrue':>7} {'nCurv':>6}")
    for bag in bags:
        name = os.path.basename(os.path.normpath(bag))
        rows = analyse(bag)
        if not rows:
            print(name, 'no samples'); continue
        v = np.array([r[0] for r in rows]); rem = np.array([r[1] for r in rows])
        vl = np.array([r[2] for r in rows]); b = np.array([r[5] for r in rows])
        Rc = np.array([r[6] for r in rows], float); Rs = np.array([r[7] for r in rows], float)
        Rt = np.array([r[8] for r in rows], float)
        n = len(rows)
        pm = 100 * np.mean(b == 'max_speed'); pc = 100 * np.mean(b == 'curvature'); pe = 100 * np.mean(b == 'path_end')
        print(f"{name:20} {n:5d} {pm:6.1f} {pc:6.1f} {pe:6.1f} {med(rem):7.1f} {np.percentile(rem, 10):7.1f} "
              f"{v.mean():7.2f} {vl.mean():7.2f} {(vl - v).mean():7.2f} {med(Rc):7.2f} {med(Rs):8.2f} "
              f"{med(Rt):7.2f} {int(np.sum(b == 'curvature')):6d}")


if __name__ == '__main__':
    main()
