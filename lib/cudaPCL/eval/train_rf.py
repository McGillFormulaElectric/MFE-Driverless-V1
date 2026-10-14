#!/usr/bin/python3
"""Collect candidate clusters from the bag, label them against GT, train the RF-paper classifier, export for the node.

  eval/train_rf.py [--out eval/rf_model.txt]
Then: eval/run_eval.py -p rf_model_path=eval/rf_model.txt -p min_cluster_size=1 ... (see printed line).
Labels: cluster centroid within 0.3 m of a GT cone => cone, else not. Validation is leave-one-block-out over 20-frame blocks
(neighbouring frames are near-duplicates, so random splits would leak).
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_eval as ev   # re-execs with ROS sourced if needed

# loose candidate generation: let the forest decide, not the cluster-size/box gates
CAND = dict(gpu_voxel=False, leaf_size=0.05, ground_threshold=0.04, cluster_tolerance=0.4, min_cluster_size=1, far_min_cluster_size=1,
            cluster_2d=True, height_min=0.02)
FEATS = ["height", "width", "depth", "aspect", "volume", "density", "distance"]


def export(rf, path):
    with open(path, "w") as f:
        f.write(f"{len(rf.estimators_)}\n")
        for e in rf.estimators_:
            t = e.tree_
            f.write(f"{t.node_count}\n")
            for i in range(t.node_count):
                v = t.value[i][0]
                leaf = t.children_left[i] == -1
                f.write(f"{-1 if leaf else t.feature[i]} {t.threshold[i]:.9g} {t.children_left[i]} {t.children_right[i]} {v[1] / v.sum():.6g}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=f"{ev.ROOT}/eval/rf_model.txt")
    ap.add_argument("--rate", type=float, default=2.5)
    ap.add_argument("--bag", default=ev.BAG)
    ap.add_argument("--train-frac", type=float, default=1.0, help="train only on this leading fraction of frames (rest = honest end-to-end test)")
    a = ap.parse_args()
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.model_selection import LeaveOneGroupOut
    from sklearn.metrics import precision_recall_fscore_support
    ap_bag = a.bag; ev.BAG = os.path.abspath(ap_bag)
    gt, lidar = ev.load_bag()   # {stamp: Nx2 GT cones in ROI, per frame}
    csv = "/tmp/rf_clusters.csv"
    if os.path.exists(csv): os.remove(csv)
    ev.run_node({**CAND, "dump_clusters_path": csv}, a.rate, "/tmp/eval_rfcollect.log")
    d = np.loadtxt(csv, delimiter=",")
    xy, X = d[:, 1:3], d[:, 5:12]
    # per-cluster frame -> that frame's GT; label = centroid within 0.3 m of a GT cone
    fi = np.clip(np.searchsorted(lidar, d[:, 0] * 1e-9 - 1e-3), 0, len(lidar) - 1)
    y = np.zeros(len(d), int); gid = np.full(len(d), -1)
    for i, (k, p) in enumerate(zip(fi, xy)):
        g = gt[lidar[k]]
        if len(g):
            dd = np.linalg.norm(g - p, axis=1)
            if dd.min() < 0.3: y[i] = 1; gid[i] = k
    # group = frame block of 20 consecutive frames: the car moves, so blocks see different parts of the track
    g = fi // 20
    keep = fi < a.train_frac * len(lidar)
    xy, X, y, g = xy[keep], X[keep], y[keep], g[keep]
    print(f"{len(y)} candidate clusters over {len(lidar)} frames, {y.sum()} cone ({y.mean():.1%}); {g.max()+1} frame blocks")
    mk = lambda: RandomForestClassifier(n_estimators=65, max_depth=10, random_state=42, class_weight="balanced")
    pred = np.zeros(len(y), int)
    for tr, te in LeaveOneGroupOut().split(X, y, g):
        pred[te] = mk().fit(X[tr], y[tr]).predict(X[te])
    p, r, f, _ = precision_recall_fscore_support(y, pred, average="binary", zero_division=0)
    print(f"leave-one-block-out cluster classification: precision {p:.3f} recall {r:.3f} F1 {f:.3f}")
    rf = mk().fit(X, y)
    print("feature importance:", {n: round(v, 3) for n, v in zip(FEATS, rf.feature_importances_)})
    export(rf, a.out)
    print(f"wrote {a.out}")
    print("run: eval/run_eval.py " + " ".join(f"-p {k}={v}" for k, v in {**CAND, "rf_model_path": a.out}.items()))


if __name__ == "__main__":
    main()
