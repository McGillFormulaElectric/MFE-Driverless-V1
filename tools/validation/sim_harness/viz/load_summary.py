"""Summarise host CPU and GPU load recorded by run_batch.sh with LOAD=1.

  python3 load_summary.py [--dir DIR] [run_id ...]

Reads DIR/<id>.pidstat (pidstat -u -r -h -l 1) and DIR/<id>.gpu (nvidia-smi csv: timestamp, util %, mem MiB,
power W). DIR defaults to $RESULTS/load (RESULTS defaults to ~/mfe_sim_results). Without run ids, every run in
DIR is summarised.

Per run: mean and max of the total CPU % of the sim stack (summed over matched processes; 100 = one core), the
same split by group (gzserver, lidar_perception_node, slam_node, path_planner_node, pure_pursuit, other
ros2/python), then GPU utilisation mean/max, GPU memory max (MiB) and power mean (W). Processes are matched by a
substring of the full command line (pidstat -l).
"""
import argparse, glob, os, sys
from collections import defaultdict

GROUPS = [
    ("gzserver", ("gzserver",)),
    ("lidar_perception_node", ("lidar_perception_node",)),
    ("slam_node", ("slam_node",)),
    ("path_planner_node", ("path_planner_node",)),
    ("pure_pursuit", ("pure_pursuit",)),
    ("other ros2/python", ("ros2", "python")),   # matched last: the catch-all for the rest of the stack
]
DEFAULT_DIR = os.path.join(os.environ.get("RESULTS", os.path.expanduser("~/mfe_sim_results")), "load")


def classify(cmd):
    for name, keys in GROUPS:
        if any(k in cmd for k in keys):
            return name
    return None


def parse_pidstat(path):
    """(time, command, %CPU) per process sample. Header lines start with '#'; 'Average' lines are skipped."""
    header, rows = None, []
    with open(path, errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if not line.strip() or line.lstrip().startswith("Average"):
                continue
            if line.startswith("#"):
                header = line.lstrip("#").split()
                continue
            if header is None:
                continue
            fields = line.split(None, len(header) - 1)      # the command is the last field and may contain spaces
            if len(fields) != len(header):
                continue
            rec = dict(zip(header, fields))
            try:
                cpu = float(rec["%CPU"])
            except (KeyError, ValueError):
                continue
            rows.append((rec.get("Time", ""), rec.get("Command", ""), cpu))
    return rows


def parse_gpu(path):
    util, mem, power = [], [], []
    with open(path, errors="replace") as fh:
        for line in fh:
            f = [x.strip() for x in line.split(",")]
            if len(f) < 4:
                continue
            vals = []
            for x in f[1:4]:
                try:
                    vals.append(float(x))
                except ValueError:
                    vals.append(None)           # e.g. "[N/A]" for power on some GPUs
            if vals[0] is not None: util.append(vals[0])
            if vals[1] is not None: mem.append(vals[1])
            if vals[2] is not None: power.append(vals[2])
    return util, mem, power


def mean_max(xs):
    return (sum(xs) / len(xs), max(xs)) if xs else (float("nan"), float("nan"))


def cpu_summary(path):
    per_t = defaultdict(lambda: defaultdict(float))     # time -> group -> summed %CPU
    for t, cmd, cpu in parse_pidstat(path):
        g = classify(cmd)
        if g:
            per_t[t][g] += cpu
    total = [sum(d.values()) for d in per_t.values()]
    groups = {name: [d.get(name, 0.0) for d in per_t.values()] for name, _ in GROUPS}
    return len(per_t), total, groups


def report(run_id, directory):
    lines = [f"run {run_id}"]
    pidstat = os.path.join(directory, run_id + ".pidstat")
    gpu = os.path.join(directory, run_id + ".gpu")
    if os.path.exists(pidstat):
        n, total, groups = cpu_summary(pidstat)
        tm, tx = mean_max(total)
        lines.append(f"  CPU sim stack: mean {tm:7.1f}%  max {tx:7.1f}%  over {n} samples (100% = one core)")
        for name, _ in GROUPS:
            gm, gx = mean_max(groups[name])
            lines.append(f"    {name:22} mean {gm:7.1f}%  max {gx:7.1f}%")
    else:
        lines.append("  CPU: no " + run_id + ".pidstat")
    if os.path.exists(gpu):
        util, mem, power = parse_gpu(gpu)
        um, ux = mean_max(util); _, mx = mean_max(mem); pm, _ = mean_max(power)
        lines.append(f"  GPU util: mean {um:5.1f}%  max {ux:5.1f}%   mem max {mx:7.0f} MiB   power mean {pm:6.1f} W")
    else:
        lines.append("  GPU: no " + run_id + ".gpu")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="Summarise CPU/GPU load recorded by run_batch.sh with LOAD=1.")
    ap.add_argument("--dir", default=DEFAULT_DIR, help="directory with <id>.pidstat and <id>.gpu (default: $RESULTS/load)")
    ap.add_argument("runs", nargs="*", help="run ids (default: every run in --dir)")
    args = ap.parse_args()
    ids = args.runs
    if not ids:
        found = set()
        for ext in ("pidstat", "gpu"):
            for p in glob.glob(os.path.join(args.dir, "*." + ext)):
                found.add(os.path.basename(p)[: -len(ext) - 1])
        ids = sorted(found)
    if not ids:
        print(f"no load files in {args.dir}", file=sys.stderr)
        sys.exit(1)
    for run_id in ids:
        print(report(run_id, args.dir))


if __name__ == "__main__":
    main()
