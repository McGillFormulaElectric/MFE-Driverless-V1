#!/usr/bin/python3
"""Cone-landmark GraphSLAM (GTSAM iSAM2) on a recorded bag; scores the pose against ground truth.

  eval/cone_slam.py [--bag test_bag/peanut_slam] [--track peanut] [--obs lidar|gt]

Graph: Pose2 chain from /sim/xsens/state_odom (BetweenFactor of successive *relative* poses - the absolute Xsens pose drifts
~1 m, its deltas do not), cone landmarks Point2 seen as BearingRange from each LiDAR frame, nearest-neighbour association
inside a gate, a landmark is only added to the graph after a 2nd sighting (rejects one-off false positives).
Evaluation: pose error vs /ground_truth/state_odom (raw Xsens vs SLAM) and landmark error vs the track CSV.
Needs: pip install --user --no-deps gtsam (numpy must stay 1.x). Run with ROS sourced (reads the bag with rosbag2_py).
"""
import argparse, csv, os, sys
import numpy as np
import gtsam

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
X = gtsam.symbol_shorthand.X
L = gtsam.symbol_shorthand.L


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(bag, obs_topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from mfe_msgs.msg import Track
    from sensor_msgs_py import point_cloud2 as pc2
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    xs, gt, obs = [], [], []   # (bag time, x, y, yaw) / same / (bag time, Nx2 cones in car frame)
    while r.has_next():
        t, d, ts = r.read_next()
        if t in ("/sim/xsens/state_odom", "/ground_truth/state_odom"):
            m = deserialize_message(d, Odometry); o = m.pose.pose
            (xs if "xsens" in t else gt).append((ts, o.position.x, o.position.y, yaw_of(o.orientation), m.twist.twist.linear.x,
                                                  m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))   # col 5: SIM time (bag time is wall clock, sim runs slower)
        elif t == obs_topic:
            if "uncolored" in t:
                m = deserialize_message(d, PointCloud2)
                p = np.array([[a[0], a[1]] for a in pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)]).reshape(-1, 2)
            else:
                p = np.array([[c.location.x, c.location.y] for c in deserialize_message(d, Track).track]).reshape(-1, 2)
            obs.append((ts, p))
    return np.array(xs), np.array(gt), obs


def at(arr, ts):
    return arr[int(np.argmin(np.abs(arr[:, 0] - ts)))]


def align(p, q):
    """Best rigid (rotation+translation) alignment of Nx2 points p onto q (Umeyama, no scale); returns aligned p."""
    mp, mq = p.mean(0), q.mean(0)
    u, _, vt = np.linalg.svd((p - mp).T @ (q - mq))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1, d]) @ u.T
    return (p - mp) @ r.T + mq


def twist_delta(arr, t0, t1):
    """Relative motion from t0 to t1 by integrating the (accurate) forward speed along the (slowly drifting) yaw: Pose2 in the t0 body frame."""
    i0, i1 = np.searchsorted(arr[:, 0], [t0, t1])
    seg = arr[i0:i1 + 1]
    if len(seg) < 2:
        return gtsam.Pose2(0, 0, 0)
    dt = np.diff(seg[:, 5])
    th = wrap(seg[:, 3] - seg[0, 3])
    mid = 0.5 * (th[1:] + th[:-1])
    return gtsam.Pose2(float((seg[1:, 4] * dt * np.cos(mid)).sum()), float((seg[1:, 4] * dt * np.sin(mid)).sum()), float(th[-1]))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def run(xs, obs, gate, rng_sigma, brg_sigma, odo_sigma, max_range, every, refresh=10, odo_mode="twist", abs_sigma=0.0, abs_every=10):
    isam = gtsam.ISAM2(gtsam.ISAM2Params())
    odo_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(odo_sigma))
    obs_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([brg_sigma, rng_sigma]))
    robust = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345), obs_noise)
    graph, init = gtsam.NonlinearFactorGraph(), gtsam.Values()
    lms = {}        # id -> map xy (current estimate)
    tent = []       # tentative landmarks: [xy, [(pose_key, bearing, range)]]
    nxt = 0
    poses_t, keys = [], []
    prev_raw = None; prev_ts = None
    est = None      # current pose estimate (gtsam.Pose2)
    result = gtsam.Values()
    for k, (ts, pts) in enumerate(obs):
        raw = at(xs, ts); raw_pose = gtsam.Pose2(raw[1], raw[2], raw[3])
        if prev_raw is not None:     # keyframe only after moving `every` metres (or turning 5 deg)
            dl = prev_raw.between(raw_pose)
            if np.hypot(dl.x(), dl.y()) < every and abs(dl.theta()) < np.radians(5):
                continue
        key = X(len(keys))
        if prev_raw is None:
            graph.add(gtsam.PriorFactorPose2(key, raw_pose, gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.01]))))
            init.insert(key, raw_pose); est = raw_pose
        else:
            if odo_mode == "twist":      # forward speed integrated along the yaw: ~7% error/step vs ~36% for differencing the noisy position
                delta = twist_delta(xs, prev_ts, ts)
                n = np.hypot(delta.x(), delta.y())
                noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([odo_sigma[0] + 0.08 * n, odo_sigma[1] + 0.08 * n, odo_sigma[2]]))
            else:
                delta, noise = prev_raw.between(raw_pose), odo_noise
            graph.add(gtsam.BetweenFactorPose2(X(len(keys) - 1), key, delta, noise))
            est = est.compose(delta); init.insert(key, est)
        prev_raw = raw_pose; prev_ts = ts
        if abs_sigma > 0 and len(keys) % abs_every == 0:   # weak absolute anchor from the Xsens pose (its error is correlated, so keep it loose and sparse)
            graph.add(gtsam.PriorFactorPose2(key, raw_pose, gtsam.noiseModel.Diagonal.Sigmas(np.array([abs_sigma, abs_sigma, 0.03]))))
        keys.append(key); poses_t.append(ts)
        # --- landmark observations ---
        for p in pts:
            rr = np.hypot(*p)
            if rr > max_range or rr < 1.0:
                continue
            br = gtsam.Rot2.fromAngle(np.arctan2(p[1], p[0]))
            w = est.transformFrom(gtsam.Point2(*p)); wxy = np.array([w[0], w[1]])
            best, bd = None, gate
            for j, m in lms.items():
                d = np.hypot(*(m - wxy))
                if d < bd: best, bd = j, d
            if best is not None:
                graph.add(gtsam.BearingRangeFactor2D(key, L(best), br, rr, robust))
                continue
            for tl in tent:      # 2nd sighting promotes a tentative landmark
                if np.hypot(*(tl[0] - wxy)) < gate:
                    tl[1].append((key, br, rr));
                    if len(tl[1]) >= 2:
                        init.insert(L(nxt), gtsam.Point2(*tl[0]))
                        for kk, b, r_ in tl[1]:
                            graph.add(gtsam.BearingRangeFactor2D(kk, L(nxt), b, r_, robust))
                        lms[nxt] = tl[0]; nxt += 1
                        tent[:] = [t_ for t_ in tent if t_ is not tl]
                    break
            else:
                tent.append([wxy, [(key, br, rr)]])
        isam.update(graph, init); graph, init = gtsam.NonlinearFactorGraph(), gtsam.Values()
        if len(keys) % refresh == 0:     # full estimate is O(graph): refresh poses/landmarks only every `refresh` keyframes
            result = isam.calculateEstimate()
            est = result.atPose2(key)
            for j in lms: lms[j] = np.array([result.atPoint2(L(j))[0], result.atPoint2(L(j))[1]])
        tent[:] = tent[-300:]            # bounded tentative pool
    result = isam.calculateEstimate()
    pose = np.array([[result.atPose2(k_).x(), result.atPose2(k_).y(), result.atPose2(k_).theta()] for k_ in keys])
    lm = np.array([[result.atPoint2(L(j))[0], result.atPoint2(L(j))[1]] for j in lms]) if lms else np.zeros((0, 2))
    return np.array(poses_t), pose, lm, len(tent)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", default=f"{ROOT}/test_bag/peanut_slam")
    ap.add_argument("--track", default="peanut")
    ap.add_argument("--obs", default="lidar", choices=["lidar", "gt"], help="cone observations: LiDAR detector output or ground-truth cones in car frame")
    ap.add_argument("--gate", type=float, default=1.0)
    ap.add_argument("--rng", type=float, default=0.15); ap.add_argument("--brg", type=float, default=0.03)
    ap.add_argument("--odo", type=float, nargs=3, default=[0.05, 0.05, 0.01], help="odometry delta sigma x y theta")
    ap.add_argument("--odo-mode", default="twist", choices=["twist", "pose"], help="twist: integrate Xsens velocity/yaw; pose: difference Xsens positions")
    ap.add_argument("--abs-sigma", type=float, default=0.0, help="m; >0 adds weak absolute Xsens pose priors every --abs-every keyframes")
    ap.add_argument("--abs-every", type=int, default=10)
    ap.add_argument("--max-range", type=float, default=15.0)
    ap.add_argument("--pose", default="xsens", choices=["xsens", "gt"], help="odometry source (gt = control experiment with perfect poses)")
    ap.add_argument("--tmax", type=float, default=0, help="only use the first N seconds of the bag (0 = all)")
    ap.add_argument("--every", type=float, default=1.0, help="keyframe spacing in metres")
    a = ap.parse_args()
    topic = "/perception/cones_uncolored" if a.obs == "lidar" else "/ground_truth/cones_colored"
    xs, gt, obs = load(a.bag, topic)
    if a.pose == "gt": xs = gt
    if a.tmax: obs = [o for o in obs if (o[0] - obs[0][0]) * 1e-9 < a.tmax]
    print(f"{len(xs)} xsens, {len(gt)} gt odom, {len(obs)} {a.obs} observation frames")
    t, pose, lm, ntent = run(xs, obs, a.gate, a.rng, a.brg, a.odo, a.max_range, a.every, odo_mode=a.odo_mode, abs_sigma=a.abs_sigma, abs_every=a.abs_every)
    # pose error vs GT (same odom frame as Xsens; Xsens = GT + noise)
    g = np.array([at(gt, ti)[1:] for ti in t]); x = np.array([at(xs, ti)[1:] for ti in t])
    e_x = np.hypot(*(x[:, :2] - g[:, :2]).T); e_s = np.hypot(*(pose[:, :2] - g[:, :2]).T)
    ey_x = np.abs(wrap(x[:, 2] - g[:, 2])); ey_s = np.abs(wrap(pose[:, 2] - g[:, 2]))
    print(f"\npose over {len(t)} keyframes:            raw Xsens      SLAM")
    print(f"  position RMSE (m)             {np.sqrt((e_x**2).mean()):8.3f} {np.sqrt((e_s**2).mean()):10.3f}")
    print(f"  position max  (m)             {e_x.max():8.3f} {e_s.max():10.3f}")
    a_x = np.hypot(*(align(x[:, :2], g[:, :2]) - g[:, :2]).T); a_s = np.hypot(*(align(pose[:, :2], g[:, :2]) - g[:, :2]).T)
    print(f"  position RMSE after rigid align {np.sqrt((a_x**2).mean()):8.3f} {np.sqrt((a_s**2).mean()):10.3f}   <- removes the unobservable global offset")
    print(f"  yaw RMSE (deg)                {np.degrees(np.sqrt((ey_x**2).mean())):8.3f} {np.degrees(np.sqrt((ey_s**2).mean())):10.3f}")
    # landmarks vs track CSV (CSV is in map frame; odom origin = car_start)
    _EUFS_MASTER = os.environ.get("EUFS_MASTER", os.path.expanduser("~/Develop/MFE26-eufs-sim"))
    rows = list(csv.DictReader(open(f"{_EUFS_MASTER}/eufs_sim/eufs_tracks/csv/{a.track}.csv")))
    cs = [r for r in rows if r["tag"] == "car_start"][0]
    x0, y0, yaw0 = float(cs["x"]), float(cs["y"]), float(cs["direction"])
    cones = np.array([[float(r["x"]), float(r["y"])] for r in rows if r["tag"] in ("blue", "yellow", "orange", "big_orange")])
    c, s = np.cos(yaw0), np.sin(yaw0)
    cones_o = np.c_[c * (cones[:, 0] - x0) + s * (cones[:, 1] - y0), -s * (cones[:, 0] - x0) + c * (cones[:, 1] - y0)]
    if len(lm):
        for _ in range(20):      # SLAM frame is anchored at the (noisy) first pose: fit a rigid transform to the track first
            nn = cones_o[np.linalg.norm(lm[:, None] - cones_o[None], axis=2).argmin(1)]
            lm = align(lm, nn)
        d = np.linalg.norm(lm[:, None] - cones_o[None], axis=2)
        near = d.min(1); cover = d.min(0)
        print(f"\nlandmarks (rigidly aligned to track): {len(lm)} in graph ({ntent} tentative left) vs {len(cones_o)} true cones")
        print(f"  landmarks within 0.5 m of a true cone: {(near < 0.5).sum()} ({(near < 0.5).mean():.0%}); spurious (>1 m): {(near > 1).sum()}")
        print(f"  true cones with a landmark within 0.5 m: {(cover < 0.5).sum()} of {len(cones_o)}; RMSE of matched {np.sqrt((near[near<0.5]**2).mean()):.3f} m")
    n = len(t)
    for name, sl in (("1st half", slice(0, n // 2)), ("2nd half", slice(n // 2, n))):
        print(f"  {name}: Xsens RMSE {np.sqrt((e_x[sl]**2).mean()):.3f} m, SLAM RMSE {np.sqrt((e_s[sl]**2).mean()):.3f} m")


if __name__ == "__main__":
    main()
