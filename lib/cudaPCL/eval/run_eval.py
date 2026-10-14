#!/usr/bin/python3
"""Closed-loop eval: run lidar_perception_node on a bag, score cone detections vs ground truth.

  eval/run_eval.py [-p leaf_size=0.1 ...]          single run
  eval/run_eval.py --sweep [--json out.json]        small grid search
  eval/run_eval.py --tune 30 [--study eval/study.db]  Optuna TPE, tuned on first 70% of frames, held-out on rest
Needs the workspace built; self-sources ROS. --tune needs `/usr/bin/python3 -m pip install --user optuna`.
"""
import argparse, itertools, json, os, re, signal, subprocess, sys, time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BAG = f"{ROOT}/test_bag/accel_moving"   # override with --bag
SETUP = f"source /opt/ros/humble/setup.bash && source {ROOT}/prefix/ws/install/setup.bash"
ROI_X = (-1.0, 15.0)          # node's passthrough ROI, sensor frame
# FRAME NOTE (verified on accel_moving): /lidar/points_raw is in the sensor frame and /ground_truth/cones_colored is
# published relative to the car (only cones in view, changes every message), so GT needs no transform but must be looked
# up PER FRAME (nearest GT message in bag time). (The old accel_cuda_test bag had a parked car, so GT looked static.)
if "AMENT_PREFIX_PATH" not in os.environ:   # not in a ROS shell: re-exec self with ROS sourced
    os.execvp("bash", ["bash", "-c", f'{SETUP} && exec /usr/bin/python3 "$0" "$@"', os.path.abspath(__file__), *sys.argv[1:]])
os.environ["ROS_DOMAIN_ID"] = "77"   # private DDS domain: stray nodes elsewhere can't pollute counts


def load_bag():
    """Return ({lidar header stamp: Nx2 GT cones in ROI}, sorted lidar stamps). GT = nearest GT msg in bag time."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    gts, lidar = [], []   # (bag time, Nx2) / (bag time, header stamp)
    while r.has_next():
        topic, data, ts = r.read_next()
        if "cones" in topic:
            m = deserialize_message(data, get_message(types[topic]))
            gts.append((ts, np.array([[c.location.x, c.location.y] for c in m.track]).reshape(-1, 2)))
        elif topic == "/lidar/points_raw":
            h = deserialize_message(data, get_message(types[topic])).header
            lidar.append((ts, h.stamp.sec + h.stamp.nanosec * 1e-9))
    gt_t = np.array([g[0] for g in gts])
    out = {}
    for ts, stamp in lidar:
        g = gts[int(np.argmin(np.abs(gt_t - ts)))][1]
        out[stamp] = g[(g[:, 0] >= ROI_X[0]) & (g[:, 0] <= ROI_X[1])]
    return out, np.array(sorted(out))


def kill_group(p):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        try:
            p.wait(5); break
        except subprocess.TimeoutExpired:
            pass
    try: os.killpg(p.pid, signal.SIGKILL)   # reap any group stragglers
    except ProcessLookupError: pass


def run_node(params, rate, logfile):
    """Play bag through node with params; return {stamp: Nx2 detections} and latencies (ms)."""
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    args = " ".join(f"-p {k}:={v}" for k, v in params.items())
    sh = lambda c, out: subprocess.Popen(["bash", "-c", f"{SETUP} && exec {c}"], stdout=out, stderr=subprocess.STDOUT,
                                         start_new_session=True)  # own process group -> killpg
    dets = {}
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))   # so `finally` below still reaps children
    log = open(logfile, "w")
    node = sh(f"ros2 run lidar_cone_detector lidar_perception_node --ros-args {args}", log)
    play = None
    rclpy.init()
    try:
        n = rclpy.create_node("eval_collector")
        def cb(m):
            pts = pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
            dets[m.header.stamp.sec + m.header.stamp.nanosec * 1e-9] = np.array([[p[0], p[1]] for p in pts]).reshape(-1, 2)
        n.create_subscription(PointCloud2, "/perception/cones_uncolored", cb, 100)
        t0 = time.time()
        while "Initialized" not in open(logfile).read():   # wait for node startup
            rclpy.spin_once(n, timeout_sec=0.1)
            assert time.time() - t0 < 30 and node.poll() is None, "node failed to start, see " + logfile
        for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)   # let DDS discovery settle
        play = sh(f"ros2 bag play {BAG} --rate {rate}", subprocess.DEVNULL)
        while play.poll() is None: rclpy.spin_once(n, timeout_sec=0.1)
        t0 = time.time()
        while time.time() - t0 < 1.5: rclpy.spin_once(n, timeout_sec=0.1)   # drain
        n.destroy_node()
    finally:
        for p in (play, node):
            if p: kill_group(p)
        rclpy.shutdown()
        log.close()
    lat = [float(x) for x in re.findall(r"proc_ms=([\d.]+)", open(logfile).read())]
    return dets, lat


def score(dets, lidar, gt, thr, sel):
    """Per-frame one-to-one nearest-neighbour matching (greedy by distance) over frames `sel`."""
    tp = fp = fn = 0; errs = []; res = []
    for t in lidar[sel]:
        g = gt[t]
        d = dets[min(dets, key=lambda k: abs(k - t))] if dets and min(abs(k - t) for k in dets) < 1e-3 else np.zeros((0, 2))
        pairs = sorted((np.hypot(*(g[i] - d[j])), i, j) for i in range(len(g)) for j in range(len(d)))
        ug, ud = set(), set()
        for e, i, j in pairs:
            if e <= thr and i not in ug and j not in ud:
                ug.add(i); ud.add(j); errs.append(e); res.append(d[j] - g[i])
        tp += len(ug); fp += len(d) - len(ud); fn += len(g) - len(ug)
    p = tp / max(tp + fp, 1); r = tp / max(tp + fn, 1)
    return dict(precision=p, recall=r, f1=2 * p * r / max(p + r, 1e-9), mean_err=float(np.mean(errs)) if errs else None,
                tp=tp, fp=fp, fn=fn, resid=np.mean(res, 0).round(3).tolist() if res else None)


def evaluate(params, ctx, tag="run", trial=None):
    gt, lidar, a = ctx
    dets, lat = run_node(params, a.rate, f"/tmp/eval_{tag}.log")
    lat = np.array(lat) if lat else np.array([np.nan])
    out = {"params": params, "frames_out": len(dets), "frames_in": len(lidar),
           "lat_mean_ms": float(lat.mean()), "lat_p95_ms": float(np.percentile(lat, 95))}
    k = int(len(lidar) * a.split)
    out["train"] = score(dets, lidar, gt, a.thr, slice(0, k))
    out["test"] = score(dets, lidar, gt, a.thr, slice(k, None))
    out["all"] = score(dets, lidar, gt, a.thr, slice(None))
    log_row(tag, out, trial, "all" if tag == "run" else "train")
    return out


RESULTS = f"{ROOT}/eval/RESULTS.md"   # append-only history of every evaluated point
_best = [-1e9]                        # best reward so far this session (rows are append-only, so mark when set)


def md(text):
    with open(RESULTS, "a") as f: f.write(text)


def log_header(mode, argv):
    import datetime, hashlib
    src = open(f"{ROOT}/prefix/ws/src/lidar_cone_detector/src/lidar_perception_node.cpp", "rb").read()
    md(f"\n## {datetime.datetime.now():%Y-%m-%d %H:%M:%S} - {mode}\n\nbag `{os.path.relpath(BAG, ROOT)}`, node src md5 "
       f"`{hashlib.md5(src).hexdigest()[:8]}`, cmd `{' '.join(argv)}`. P/R/F1/err on the train split "
       f"(first 70% of frames) unless row says `all`; best = new best reward so far.\n\n"
       "| time | mode | trial | split | params | P | R | F1 | err m | lat ms | p95 ms | best |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n")


def log_row(mode, o, trial=None, key="train"):
    import datetime
    s, r = o[key], reward(o, key)
    star = "**best**" if r > _best[0] else ""
    _best[0] = max(_best[0], r)
    p = " ".join(f"{k}={round(v, 4) if isinstance(v, float) else v}" for k, v in o["params"].items()) or "(defaults)"
    md(f"| {datetime.datetime.now():%H:%M:%S} | {mode} | {'' if trial is None else trial} | {key} | {p} | {s['precision']:.3f} | "
       f"{s['recall']:.3f} | {s['f1']:.3f} | {s['mean_err'] or float('nan'):.3f} | {o['lat_mean_ms']:.1f} | {o['lat_p95_ms']:.1f} | {star} |\n")


def fmt(o, key="all"):
    s = o[key]
    return (f"P={s['precision']:.3f} R={s['recall']:.3f} F1={s['f1']:.3f} err={s['mean_err'] or float('nan'):.3f}m "
            f"(tp={s['tp']} fp={s['fp']} fn={s['fn']}) lat={o['lat_mean_ms']:.1f}ms p95={o['lat_p95_ms']:.1f}ms "
            f"frames={o['frames_out']}/{o['frames_in']}")


def reward(o, key="train"):
    s = o[key]   # recall, hard-ish precision floor of 0.95, small latency penalty
    return s["recall"] - 2 * max(0, 0.95 - s["precision"]) - 0.001 * o["lat_mean_ms"]


def rosarg(params):
    return "--ros-args " + " ".join(f"-p {k}:={v}" for k, v in params.items())


def main():
    global BAG
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", action="append", default=[], metavar="K=V", help="node ROS param (repeatable)")
    ap.add_argument("--rate", type=float, default=1.0, help="bag playback rate; 1.0 = recorded LiDAR frequency")
    ap.add_argument("--bag", default=BAG)
    ap.add_argument("--thr", type=float, default=0.5, help="match distance (m)")
    ap.add_argument("--split", type=float, default=0.7, help="train fraction of frames (tune/sweep ranking)")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--tune", type=int, metavar="N")
    ap.add_argument("--study", default=f"{ROOT}/eval/study.db")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    BAG = os.path.abspath(a.bag)
    base = {k: json.loads(v) if re.fullmatch(r"-?[\d.]+", v) else v for k, v in (x.split("=") for x in a.p)}
    gt, lidar = load_bag()
    print(f"bag {os.path.basename(BAG)}: {len(lidar)} lidar frames, GT cones in ROI x{ROI_X}: "
          f"{np.mean([len(g) for g in gt.values()]):.1f}/frame (min {min(len(g) for g in gt.values())}, max {max(len(g) for g in gt.values())})")
    ctx = (gt, lidar, a)
    results = []
    log_header("tune" if a.tune else "sweep" if a.sweep else "run", sys.argv[1:])
    if a.sweep:
        grid = {"leaf_size": [0.05, 0.1], "ground_threshold": [0.05, 0.1], "cluster_tolerance": [0.3, 0.5],
                "min_cluster_size": [2, 3], "far_min_cluster_size": [1, 2]}
        for i, v in enumerate(itertools.product(*grid.values())):
            o = evaluate({**base, **dict(zip(grid, v))}, ctx, "sweep", i); results.append(o)
            print(f"[{i+1}] {o['params']}\n    {fmt(o, 'train')}", flush=True)
        results.sort(key=reward, reverse=True)
        print("\nRanked (train reward):")
        for o in results[:10]: print(f"  {reward(o):.3f} {o['params']}\n        train: {fmt(o,'train')}\n        test:  {fmt(o,'test')}")
    elif a.tune:
        import optuna
        study = optuna.create_study(study_name="cones", storage=f"sqlite:///{a.study}", load_if_exists=True,
                                    direction="maximize", sampler=optuna.samplers.TPESampler(seed=0))
        def obj(tr):
            params = {**base,
                "leaf_size": tr.suggest_float("leaf_size", 0.03, 0.2, log=True),
                "ground_threshold": tr.suggest_float("ground_threshold", 0.03, 0.25),
                "cluster_tolerance": tr.suggest_float("cluster_tolerance", 0.2, 0.8),
                "min_cluster_size": tr.suggest_int("min_cluster_size", 1, 4),
                "far_min_cluster_size": tr.suggest_int("far_min_cluster_size", 1, 3),
                "near_far_split_x": tr.suggest_float("near_far_split_x", 3.0, 10.0),
                "height_min": tr.suggest_float("height_min", 0.005, 0.1),
                "height_max": tr.suggest_float("height_max", 0.4, 0.9),
                "cluster_2d": tr.suggest_categorical("cluster_2d", [True, False]),
                "min_cluster_dz": tr.suggest_float("min_cluster_dz", 0.0, 0.2)}
            o = evaluate(params, ctx, "tune", tr.number); results.append(o)
            print(f"trial {tr.number}: reward={reward(o):.3f} {fmt(o, 'train')}", flush=True)
            return reward(o)
        study.optimize(obj, n_trials=a.tune)
        best = {**base, **study.best_params}
        print(f"\nBest train reward {study.best_value:.3f} over {len(study.trials)} trials")
        o = evaluate(best, ctx, "best"); results = [o]
        print(f"train:    {fmt(o, 'train')}\nHELD-OUT: {fmt(o, 'test')}")
        line = rosarg({k: (round(v, 4) if isinstance(v, float) else v) for k, v in best.items()})
        print("\nPaste: " + line)
        log_row("best", o, study.best_trial.number, "test")
        md(f"\n### Tune summary (best trial {study.best_trial.number} of {len(study.trials)}, train reward {study.best_value:.3f})\n\n"
           f"| split | P | R | F1 | err m | lat ms |\n|---|---|---|---|---|---|\n" +
           "".join(f"| {k} | {o[k]['precision']:.3f} | {o[k]['recall']:.3f} | {o[k]['f1']:.3f} | {o[k]['mean_err'] or float('nan'):.3f} | {o['lat_mean_ms']:.1f} |\n" for k in ("train", "test")) +
           f"\nparams: `{study.best_params}`\n\n```\n{line}\n```\n")
    else:
        o = evaluate(base, ctx); results = [o]
        print(fmt(o)); print(f"mean residual det-GT (x,y) = {o['all']['resid']}  <- should be ~0 if frame/offset is right")
    if a.json: json.dump(results, open(a.json, "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
