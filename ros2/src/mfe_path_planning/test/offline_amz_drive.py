"""Offline kinematic drive of AmzPlanner around an EUFS track CSV (no ROS).
  python3 offline_amz_drive.py <track> [colors|unknown] [out.png]
Moves a point car 1.5 m along each planned path; reports laps, closest cone approach, stalls."""
import csv, sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mfe_path_planning"))
from amz_planner import AmzPlanner, LEFT, RIGHT, UNKNOWN
track = sys.argv[1]; mode = sys.argv[2] if len(sys.argv) > 2 else "colors"
rows = list(csv.DictReader(open(f"/home/neil/Develop/MFE26-eufs-sim-main/eufs_sim/eufs_tracks/csv/{track}.csv")))
cmap = {"blue": LEFT, "yellow": RIGHT}
cones = np.array([[float(r["x"]), float(r["y"])] for r in rows if r["tag"] in cmap])
cols = np.array([cmap[r["tag"]] for r in rows if r["tag"] in cmap])
if mode == "unknown": cols[:] = UNKNOWN
cs = [r for r in rows if r["tag"] == "car_start"][0]
pos = np.array([float(cs["x"]), float(cs["y"])]); yaw = float(cs["direction"])
head = np.array([np.cos(yaw), np.sin(yaw)])
pl = AmzPlanner()
traj = [pos.copy()]; stalls = 0; turn = 0.0
for step in range(400):
    res = pl.plan(cones, cols, pos, head)
    if res is None or len(res[0]) < 3:
        stalls += 1; print("stall", step, pl.last_debug); break
    path = res[0]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    tgt = np.array([np.interp(1.5, s, path[:, 0]), np.interp(1.5, s, path[:, 1])])
    nh = tgt - pos; nh /= np.linalg.norm(nh)
    turn += np.arctan2(head[0] * nh[1] - head[1] * nh[0], head @ nh)
    pos, head = tgt, nh; traj.append(pos.copy())
    if abs(turn) > 2 * np.pi and np.linalg.norm(pos - traj[0]) < 3: break
traj = np.array(traj)
d = np.min(np.linalg.norm(traj[:, None, :] - cones[None], axis=2), axis=1)
dist = np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
print(f"{track}/{mode}: steps={len(traj)-1} driven={dist:.0f} m  net_turn={np.degrees(turn):.0f} deg  "
      f"back_at_start={np.linalg.norm(pos-traj[0]) < 3}  closest_cone={d.min():.2f} m  "
      f"pts<0.8m={int((d<0.8).sum())}  stalls={stalls}")
if len(sys.argv) > 3:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    for c, col in ((LEFT, "tab:blue"), (RIGHT, "gold")):
        m = np.array([cmap[r["tag"]] == c for r in rows if r["tag"] in cmap])
        ax.scatter(*cones[m].T, s=12, c=col)
    ax.plot(*traj.T, "r-", lw=1.5); ax.plot(*traj[0], "ko"); ax.set_aspect("equal"); ax.set_title(f"{track} {mode}")
    fig.savefig(sys.argv[3], dpi=90)
