"""Give LiDAR cone landmarks a colour from the camera's YOLO detections.

The camera only supplies pixel positions and a colour class per detection (the vision node runs without depth). A landmark
(position in the SLAM frame) is projected into the image taken at the detection's own timestamp, using the car pose at that
moment, and the nearest detection decides one vote for the landmark's colour. Votes accumulate over frames.
"""
import bisect
import math
import numpy as np

BLUE, YELLOW, ORANGE_BIG, ORANGE_SMALL, UNKNOWN = 0, 1, 2, 3, 4


def _ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class PoseHistory:
    """Time-ordered car poses (t, x, y, yaw), interpolated at a requested time."""

    def __init__(self, keep=400):
        self.t, self.p, self.keep = [], [], keep

    def add(self, t, x, y, yaw):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t); self.p.append((x, y, yaw))
        if len(self.t) > self.keep:
            del self.t[:len(self.t) - self.keep]; del self.p[:len(self.p) - self.keep]

    def at(self, t, max_gap=0.5):
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t):
            j = 0 if i == 0 else len(self.t) - 1
            return self.p[j] if self.t and abs(self.t[j] - t) <= max_gap else None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > max_gap:
            return None
        a = (t - t0) / (t1 - t0); p0, p1 = self.p[i - 1], self.p[i]
        dyaw = (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi
        return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), p0[2] + a * dyaw)


class CamFusion:
    def __init__(self, hfov_deg=87.0, width=640, height=480, cam_xyz=(0.12, 0.0, 0.76), cam_pitch_deg=10.15,
                 cone_z=0.16, du=0.0, dv=0.0, match_px=40.0, min_range=2.0, max_range=18.0):
        self.fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.cx, self.cy, self.w, self.h = width / 2.0, height / 2.0, width, height
        self.r_cam, self.t_cam = _ry(cam_pitch_deg), np.array(cam_xyz, float)
        self.cone_z, self.du, self.dv = cone_z, du, dv
        self.match_px, self.min_range, self.max_range = match_px, min_range, max_range

    def project(self, lm_xy, pose):
        """Landmark (x, y in the SLAM frame) -> (u, v, range) in the image for a car pose, or None if out of view."""
        x, y, yaw = pose
        dx, dy = lm_xy[0] - x, lm_xy[1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        base = np.array([c * dx + s * dy, -s * dx + c * dy, self.cone_z])
        rng = math.hypot(base[0], base[1])
        if rng < self.min_range or rng > self.max_range:
            return None
        pc = self.r_cam.T @ (base - self.t_cam)          # camera link frame: x forward, y left, z up
        xo, yo, zo = -pc[1], -pc[2], pc[0]               # optical frame: x right, y down, z forward
        if zo < 0.3:
            return None
        u, v = self.fx * xo / zo + self.cx + self.du, self.fx * yo / zo + self.cy + self.dv
        if not (0 <= u < self.w and 0 <= v < self.h):
            return None
        return u, v, rng

    def vote(self, detections, landmarks, pose):
        """detections: [(u, v, colour)]; landmarks: {id: (x, y)}. Returns [(landmark id, colour)] for confident matches."""
        proj = {}
        for j, lm in landmarks.items():
            r = self.project(lm, pose)
            if r is not None:
                proj[j] = r
        if not proj or not detections:
            return []
        ids = list(proj)
        pu = np.array([proj[j][0] for j in ids]); pv = np.array([proj[j][1] for j in ids])
        cand = []                                         # (distance, detection index, landmark index)
        for di, (u, v, col) in enumerate(detections):
            if col == UNKNOWN:
                continue
            d = np.hypot(pu - u, 0.5 * (pv - v))
            order = np.argsort(d)
            if d[order[0]] > self.match_px:
                continue
            if len(order) > 1 and d[order[1]] < 1.5 * d[order[0]] and d[order[1]] < self.match_px:
                continue                                  # two landmarks equally close: skip rather than guess
            cand.append((float(d[order[0]]), di, int(order[0])))
        out, used = [], set()
        for dist, di, li in sorted(cand):                 # each landmark takes only its closest detection
            if li in used:
                continue
            used.add(li); out.append((ids[li], int(detections[di][2])))
        return out


def decide(votes, min_votes=2, share=0.6, big_share=0.3):
    """votes: length-5 count array -> colour, or None when there is not enough agreement.

    The two orange classes count as one colour group: YOLO calls the big gate cones "big" and "small" about equally often
    (measured on the sim), so a landmark is big orange when at least big_share of its orange votes were "big".
    """
    v = np.asarray(votes, float)
    groups = [v[BLUE], v[YELLOW], v[ORANGE_BIG] + v[ORANGE_SMALL]]
    total = float(sum(groups))
    if total < min_votes:
        return None
    k = int(np.argmax(groups))
    if groups[k] / total < share:
        return None
    if k == 2:
        return ORANGE_BIG if v[ORANGE_BIG] >= big_share * groups[2] else ORANGE_SMALL
    return k
