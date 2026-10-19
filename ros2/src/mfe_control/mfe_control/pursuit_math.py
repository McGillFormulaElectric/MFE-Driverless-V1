"""Pure math for the pure pursuit controller (no ROS dependencies)."""

import math


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def smooth_path(path, w):
    """Centred moving average over w points; the window shrinks at the ends so length and endpoints are kept."""
    if w <= 1 or len(path) < 3:
        return path
    h = w // 2
    n = len(path)
    out = []
    for i in range(n):
        k = min(h, i, n - 1 - i)
        seg = path[i - k:i + k + 1]
        out.append((sum(p[0] for p in seg) / len(seg), sum(p[1] for p in seg) / len(seg)))
    return out


def menger_curvature(p0, p1, p2):
    """Menger curvature through three points, or None if either segment is degenerate."""
    dx1 = p1[0] - p0[0];  dy1 = p1[1] - p0[1]
    dx2 = p2[0] - p1[0];  dy2 = p2[1] - p1[1]
    cross = abs(dx1 * dy2 - dy1 * dx2)
    l1 = math.hypot(dx1, dy1)
    l2 = math.hypot(dx2, dy2)
    if l1 < 0.01 or l2 < 0.01:
        return None
    return cross / (l1 * l2 * (l1 + l2) / 2.0)


def local_curvature(path, idx):
    """Mean Menger curvature of the triplets centred within +-2 waypoints of idx (0.0 if none usable)."""
    if len(path) < 3:
        return 0.0
    n = len(path)
    kappas = []
    for centre in range(max(1, idx - 2), min(n - 1, idx + 3)):
        k = menger_curvature(path[centre - 1], path[centre], path[centre + 1])
        if k is not None:
            kappas.append(k)
    return sum(kappas) / len(kappas) if kappas else 0.0


def corner_speed(kappa, a_lat, v_max):
    """Speed that keeps lateral acceleration a_lat on a corner of curvature kappa, capped at v_max."""
    R = 1.0 / kappa if kappa > 1e-4 else float('inf')
    return min(v_max, math.sqrt(a_lat * R))


def curvature_speed_target(path, idx, scan_n, v_now, v_max, a_lat, decel):
    """Scan scan_n points ahead for the tightest corner and ramp speed down to it before it is reached."""
    n = len(path)
    scan_end = min(n, idx + scan_n)

    v_target = v_max
    dist_to_target = float('inf')
    cum_dist = 0.0

    for i in range(idx, scan_end - 2):
        if i > idx:
            cum_dist += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        kappa = menger_curvature(path[i], path[i + 1], path[i + 2])
        if kappa is None:
            continue
        v_corner = corner_speed(kappa, a_lat, v_max)
        if v_corner < v_target:
            v_target = v_corner
            dist_to_target = cum_dist

    d_brake = max(0.0, (v_now ** 2 - v_target ** 2) / (2.0 * decel))
    ramp_dist = max(20.0, d_brake * 5.0)

    if v_target < v_max and dist_to_target <= ramp_dist:
        t = dist_to_target / ramp_dist
        return max(0.0, v_target + t * (v_max - v_target))
    return v_max


def remaining_length(path, idx):
    """Polyline length from waypoint idx to the end of path."""
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
               for i in range(idx, len(path) - 1))


def path_end_speed_cap(remaining, decel, margin):
    """Max speed from which the car can stop within `remaining - margin` metres at decel."""
    return math.sqrt(2.0 * decel * max(0.0, remaining - margin))


def find_lookahead(path, start_idx, car_x, car_y, ld, local_window=60, relocate_m=10.0):
    """
    Return (lookahead_point, closest_idx).

    The nearest waypoint is searched near start_idx, or over the whole path if the car is more than
    relocate_m away from it. The lookahead is the first waypoint at or beyond ld from the car.
    """
    n = len(path)
    min_dist, closest = _scan_nearest(path, range(start_idx, min(n, start_idx + local_window)),
                                      car_x, car_y, float('inf'), start_idx)
    if min_dist > relocate_m:
        _, closest = _scan_nearest(path, range(n), car_x, car_y, min_dist, closest)

    for i in range(closest, n):
        wx, wy = path[i]
        if math.hypot(wx - car_x, wy - car_y) >= ld:
            return (wx, wy), closest
    return path[-1], closest


def _scan_nearest(path, indices, car_x, car_y, best_d, best_i):
    for i in indices:
        d = math.hypot(path[i][0] - car_x, path[i][1] - car_y)
        if d < best_d:
            best_d, best_i = d, i
    return best_d, best_i


def pure_pursuit_steering(car_x, car_y, yaw, target, wheelbase, max_steer_rad):
    """Normalised steering in [-1, 1] that drives the car to target on an arc."""
    dx = target[0] - car_x
    dy = target[1] - car_y
    alpha = math.atan2(dy, dx) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    # Actual distance to the target, floored: dividing by ld over-steered on sparse paths.
    d_la = max(math.hypot(dx, dy), 0.5)
    steering_rad = math.atan2(2.0 * wheelbase * math.sin(alpha), d_la)
    return max(-1.0, min(1.0, steering_rad / max_steer_rad))


def pi_throttle_brake(v_target, v_now, integral, kp, ki, kb, deadband, dt):
    """
    Closed-loop PI speed control. Returns (throttle, brake, integral).

    Integral is clamped to [-1, 1]. Brake is proportional to overspeed beyond the deadband and
    zeroes throttle and the positive integral while active.
    """
    v_error = v_target - v_now

    integral += v_error * dt
    integral = max(-1.0, min(1.0, integral))

    throttle = kp * v_error + ki * integral
    throttle = max(0.0, min(1.0, throttle))

    overspeed = -v_error - deadband
    if overspeed > 0.0:
        integral = min(integral, 0.0)
        return 0.0, max(0.0, min(1.0, kb * overspeed)), integral
    return throttle, 0.0, integral


class Pt2Anticipator:
    """
    Discrete PT2 inverse (backward Euler) feedforward that pre-emphasises the steering command so that
    a PT2 actuator model, G(s) = 1 / (tau^2 s^2 + 2 zeta tau s + 1), tracks the desired value.

    u[k] = A r[k] - B r[k-1] + C r[k-2], with ratio = tau/T:
        A = 1 + 2 zeta ratio + ratio^2,  B = 2 zeta ratio + 2 ratio^2,  C = ratio^2
    A two-stage low-pass (time constant filter_tau) on r keeps the inverse's high-frequency gain bounded.
    """

    def __init__(self, tau, zeta, filter_tau, dt):
        self.tau = tau
        self.zeta = zeta
        self.filter_tau = filter_tau
        self.dt = dt
        self.prev1 = 0.0
        self.prev2 = 0.0
        self.lp1 = 0.0
        self.lp2 = 0.0

    def reset_history(self):
        # The low-pass stages are intentionally not reset.
        self.prev1 = 0.0
        self.prev2 = 0.0

    def __call__(self, r_k):
        tau = self.tau
        zeta = self.zeta
        T = self.dt

        if self.filter_tau > 0.0 and tau > 0.0:
            a = T / (self.filter_tau + T)
            self.lp1 += a * (r_k - self.lp1)
            self.lp2 += a * (self.lp1 - self.lp2)
            r_k = self.lp2

        ratio = tau / T
        ratio2 = ratio * ratio

        A = 1.0 + 2.0 * zeta * ratio + ratio2
        B = 2.0 * zeta * ratio + 2.0 * ratio2
        C = ratio2

        u_k = A * r_k - B * self.prev1 + C * self.prev2
        self.prev2 = self.prev1
        self.prev1 = r_k
        return max(-1.0, min(1.0, u_k))
