"""
LTV-MPC lateral controller for MFE driverless.

Replaces pure pursuit steering with a model predictive controller.
Throttle/brake from pure_pursuit_node are passed through unchanged.

Model: 2-state linearised bicycle error model
    State:  e = [e_y, e_yaw]
        e_y   — cross-track error (signed; positive when car is left of path)
        e_yaw — heading error = car_yaw − path_tangent_yaw, wrapped to (−π, π]
    Input:  δ  — front-wheel steering angle (rad), linearised: tan(δ) ≈ δ

Discrete-time model (Forward Euler, dt = 0.05 s):
    e_y(k+1)   = e_y(k)   + v·e_yaw(k)·dt
    e_yaw(k+1) = e_yaw(k) + (v/L)·δ(k)·dt

    A = [[1, v·dt], [0, 1]]
    B = [[0], [v·dt/L]]

Note on limitations: the model omits the reference-path curvature feedforward
term (−v·κ_ref in the e_yaw dynamics). This is a deliberate simplification that
keeps the QP linear-time-varying but introduces lag on tight corners. The
sin/tan linearisations also degrade for large errors. A future extension can
add κ_ref feedforward once a reliable curvature estimate is available from the
planner.

QP objective (condensed horizon form):
    min  Σ_{k=1}^{N−1} x(k)^T Q x(k)  +  x(N)^T P x(N)  +  Σ_{k=0}^{N−1} δ(k)^T R δ(k)
    s.t. x(k+1) = A(k)·x(k) + B(k)·δ(k)
         |δ(k)| ≤ delta_max
         |δ(k) − δ(k−1)| ≤ delta_rate_max   (δ(−1) = last published δ)

Subscribes:
    /planning/centerline        (nav_msgs/Path)
    /ekf/output                 (nav_msgs/Odometry)
    /planning/mission_finished  (std_msgs/Bool)
    /control/command            (fs_msgs/ControlCommand)  — throttle/brake pass-through

Publishes:
    /control/command_mpc        (fs_msgs/ControlCommand)  — MPC steering + PP throttle/brake
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool, Header

# ---------------------------------------------------------------------------
# Optional OSQP + sparse
# ---------------------------------------------------------------------------
try:
    import osqp
    import scipy.sparse as sp
    _HAS_OSQP = True
except ImportError:
    _HAS_OSQP = False

if not _HAS_OSQP:
    from scipy.optimize import minimize


class MPCLateralNode(Node):
    """
    LTV-MPC lateral controller.

    Runs at 20 Hz. Computes an optimal steering angle via a condensed-horizon
    QP over N steps, then publishes that steering together with the throttle and
    brake values received from the pure-pursuit node.
    """

    def __init__(self):
        super().__init__('mpc_lateral_node')

        # ------------------------------------------------------------------
        # ROS parameters (all tunable without recompiling)
        # ------------------------------------------------------------------
        self.declare_parameter('wheelbase',        1.56)
        self.declare_parameter('max_steering_deg', 28.0)
        self.declare_parameter('max_lateral_accel', 8.0)   # unused by MPC directly, kept for parity
        self.declare_parameter('map_frame',        'map')

        # MPC horizon / timing
        self.declare_parameter('N',  10)       # prediction horizon steps
        self.declare_parameter('dt', 0.05)     # seconds per step (must match timer: 1/20)

        # Cost weights
        self.declare_parameter('Q_ey',   10.0)   # cross-track error weight
        self.declare_parameter('Q_eyaw',  5.0)   # heading error weight
        self.declare_parameter('R_delta', 1.0)   # steering effort weight
        self.declare_parameter('P_scale', 10.0)  # terminal cost = P_scale * Q

        # Steering constraints
        self.declare_parameter('delta_rate_max_deg', 5.0)   # max change per step

        # Minimum speed for MPC to be active (avoids near-zero model degeneracy)
        self.declare_parameter('v_min_active',  0.5)   # m/s — below this, publish δ=0
        self.declare_parameter('v_model_floor', 1.0)   # m/s — floor used in A/B matrices

        # ------------------------------------------------------------------
        # Cache parameters
        # ------------------------------------------------------------------
        self._L               = self.get_parameter('wheelbase').value
        self._max_steer_rad   = math.radians(self.get_parameter('max_steering_deg').value)
        self._map_frame       = self.get_parameter('map_frame').value
        self._N               = int(self.get_parameter('N').value)
        self._dt              = self.get_parameter('dt').value
        self._Q_ey            = self.get_parameter('Q_ey').value
        self._Q_eyaw          = self.get_parameter('Q_eyaw').value
        self._R_delta         = self.get_parameter('R_delta').value
        self._P_scale         = self.get_parameter('P_scale').value
        self._delta_rate_max  = math.radians(self.get_parameter('delta_rate_max_deg').value)
        self._v_min_active    = self.get_parameter('v_min_active').value
        self._v_model_floor   = self.get_parameter('v_model_floor').value

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._path             = None    # list of (x, y) tuples
        self._path_idx         = 0       # last-known closest waypoint index
        self._car_x            = None
        self._car_y            = None
        self._car_yaw          = None
        self._car_speed        = None
        self._mission_finished = False

        # Pass-through throttle/brake from pure pursuit (default 0 until first message)
        self._pp_throttle      = 0.0
        self._pp_brake         = 0.0

        # Last MPC output — used to enforce the delta-rate constraint for k=0
        self._last_delta_rad   = 0.0

        # OSQP solver instance (created lazily on first solve)
        self._osqp_solver      = None
        self._osqp_N_cache     = None   # horizon used when solver was set up

        # ------------------------------------------------------------------
        # QoS profiles
        # ------------------------------------------------------------------
        reliable_qos    = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Path,           '/planning/centerline',
                                 self._path_cb,     best_effort_qos)
        self.create_subscription(Odometry,        '/ekf/output',
                                 self._odom_cb,     best_effort_qos)
        self.create_subscription(Bool,            '/planning/mission_finished',
                                 self._finished_cb, reliable_qos)
        self.create_subscription(ControlCommand,  '/control/command',
                                 self._pp_cmd_cb,   reliable_qos)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(ControlCommand, '/control/command_mpc', reliable_qos)

        # ------------------------------------------------------------------
        # Control timer: 20 Hz
        # ------------------------------------------------------------------
        self.create_timer(1.0 / 20.0, self._control_loop)

        self.get_logger().info(
            f'mpc_lateral_node started | '
            f'N={self._N} dt={self._dt}s  L={self._L}m  '
            f'delta_max={math.degrees(self._max_steer_rad):.1f}deg  '
            f'rate_max={self.get_parameter("delta_rate_max_deg").value:.1f}deg/step  '
            f'Q=[{self._Q_ey},{self._Q_eyaw}] R={self._R_delta} P_scale={self._P_scale}  '
            f'OSQP={_HAS_OSQP}'
        )

    # ======================================================================
    # Callbacks
    # ======================================================================

    def _path_cb(self, msg: Path):
        if not msg.poses:
            return
        self._path     = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_idx = 0   # planner always regenerates from current position

    def _odom_cb(self, msg: Odometry):
        self._car_x   = msg.pose.pose.position.x
        self._car_y   = msg.pose.pose.position.y
        self._car_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._car_speed = math.hypot(vx, vy)

    def _finished_cb(self, msg: Bool):
        if msg.data and not self._mission_finished:
            self.get_logger().info('Mission finished — MPC lateral controller stopped.')
            self._mission_finished = True

    def _pp_cmd_cb(self, msg: ControlCommand):
        """Cache throttle/brake from pure pursuit for pass-through."""
        self._pp_throttle = float(msg.throttle)
        self._pp_brake    = float(msg.brake)

    # ======================================================================
    # Main control loop (20 Hz)
    # ======================================================================

    def _control_loop(self):
        if self._mission_finished:
            return

        if self._path is None or self._car_x is None:
            self.get_logger().warn('Waiting for path/odometry…', throttle_duration_sec=2.0)
            self._publish(0.0)
            return

        v = self._car_speed if self._car_speed is not None else 0.0

        # At very low speed the bicycle model has no meaningful lateral authority;
        # hold zero steering to avoid OSQP numerical issues.
        if v < self._v_min_active:
            self._publish(0.0)
            return

        e_y, e_yaw = self._compute_errors(self._car_x, self._car_y, self._car_yaw)
        if e_y is None:
            self._publish(0.0)
            return

        delta_rad = self._solve_mpc(e_y, e_yaw, v)

        # Clamp (safety net; OSQP constraints already enforce this)
        delta_rad = float(np.clip(delta_rad, -self._max_steer_rad, self._max_steer_rad))
        self._last_delta_rad = delta_rad
        self._publish(delta_rad)

    # ======================================================================
    # Error computation
    # ======================================================================

    def _compute_errors(self, car_x, car_y, car_yaw):
        """
        Compute lateral (cross-track) and heading errors relative to the path.

        Returns (e_y, e_yaw) or (None, None) if the path is too short.

        e_y   — positive when the car is to the LEFT of the path tangent
        e_yaw — car_yaw − tangent_yaw, wrapped to (−π, π]
        """
        path = self._path
        n    = len(path)
        if n < 2:
            return None, None

        # ------------------------------------------------------------------
        # Find the closest waypoint (search forward from last known index with
        # a bounded window; fall back to global search if nothing found nearby)
        # ------------------------------------------------------------------
        search_end  = min(n, self._path_idx + 60)
        min_dist    = float('inf')
        closest_idx = self._path_idx

        for i in range(self._path_idx, search_end):
            wx, wy = path[i]
            d = math.hypot(wx - car_x, wy - car_y)
            if d < min_dist:
                min_dist    = d
                closest_idx = i

        if min_dist > 10.0:
            for i, (wx, wy) in enumerate(path):
                d = math.hypot(wx - car_x, wy - car_y)
                if d < min_dist:
                    min_dist    = d
                    closest_idx = i

        self._path_idx = closest_idx

        # ------------------------------------------------------------------
        # Path tangent direction at closest waypoint
        # ------------------------------------------------------------------
        if closest_idx < n - 1:
            next_idx = closest_idx + 1
        else:
            next_idx = closest_idx
            closest_idx = closest_idx - 1

        wx0, wy0 = path[closest_idx]
        wx1, wy1 = path[next_idx]

        dx_path = wx1 - wx0
        dy_path = wy1 - wy0
        seg_len = math.hypot(dx_path, dy_path)
        if seg_len < 1e-6:
            return None, None

        tangent_yaw = math.atan2(dy_path, dx_path)

        # ------------------------------------------------------------------
        # Cross-track error: signed perpendicular distance.
        # Using the 2-D cross product of the tangent direction and the vector
        # from waypoint to car. Positive → car is to the left of the path.
        # ------------------------------------------------------------------
        # Unit tangent
        tx = dx_path / seg_len
        ty = dy_path / seg_len

        # Vector from closest waypoint to car
        ex = car_x - wx0
        ey_raw = car_y - wy0

        # Signed perpendicular distance: (tangent × car_vec) = tx*ey_raw − ty*ex
        e_y = tx * ey_raw - ty * ex

        # ------------------------------------------------------------------
        # Heading error
        # ------------------------------------------------------------------
        e_yaw = car_yaw - tangent_yaw
        # Wrap to (−π, π]
        e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))

        return e_y, e_yaw

    # ======================================================================
    # QP construction (condensed horizon)
    # ======================================================================

    def _build_condensed_qp(self, e_y: float, e_yaw: float, v: float):
        """
        Build the condensed-horizon LTV QP.

        Decision variable: u = [δ_0, δ_1, …, δ_{N−1}]  (N × 1)

        The state propagation is "rolled out" analytically:
            X̂ = S_x · x0 + S_u · u
        where X̂ = [x(1); x(2); …; x(N)]  (2N × 1)

        A_k and B_k vary with v (same v used for all steps — true LTV would
        re-linearise at each predicted state, but with a short horizon and
        slowly varying speed, this is a good approximation).

        Cost:  J = X̂^T Q̄ X̂ + u^T R̄ u  + (const)
              = 0.5 u^T H u + f^T u   (standard QP form; const dropped)

        where Q̄ = blockdiag(Q, Q, …, Q, P)  (stages 1..N−1 use Q; stage N uses P)
              R̄ = R · I_N

        Returns:
            H  (N×N, symmetric positive semi-definite)
            f  (N×1)
            A_ineq  constraint matrix  (2N × N)
            lb, ub  constraint bounds  (2N,)

        Constraints stacked as:
            [I_N]           lb_delta   ≤  u  ≤  ub_delta      (N box constraints)
            [D_N]   lb_rate ≤ D·u + d_prev ≤ ub_rate         (N rate constraints)

        where D_N is the first-difference matrix and d_prev accounts for
        δ(−1) = last_delta for the k=0 constraint.
        """
        N  = self._N
        dt = self._dt
        L  = self._L

        # Floor v in model matrices to avoid numerical degeneracy at low speed
        v_m = max(v, self._v_model_floor)

        # Time-invariant A and B (linearised at current speed)
        A = np.array([[1.0, v_m * dt],
                      [0.0, 1.0]])
        B = np.array([[0.0],
                      [v_m * dt / L]])

        # ------------------------------------------------------------------
        # Build S_x (2N × 2) and S_u (2N × N)
        # S_x[2k : 2k+2, :] = A^{k+1}
        # S_u[2k : 2k+2, j] = A^{k−j} · B  for j ≤ k, else 0
        # (indices k = 0…N−1 correspond to steps 1…N)
        # ------------------------------------------------------------------
        S_x = np.zeros((2 * N, 2))
        S_u = np.zeros((2 * N, N))

        Ak = np.eye(2)
        for k in range(N):
            Ak = A @ Ak   # A^{k+1}
            S_x[2*k : 2*k+2, :] = Ak

        # Precompute A powers: A_pow[k] = A^k (k = 0..N−1)
        A_pow = [np.eye(2)]
        for k in range(1, N):
            A_pow.append(A @ A_pow[-1])

        for k in range(N):          # row block (step k+1)
            for j in range(k + 1):  # column j (input δ_j)
                # A^{k−j} · B
                S_u[2*k : 2*k+2, j] = (A_pow[k - j] @ B).flatten()

        # ------------------------------------------------------------------
        # Cost matrices
        # ------------------------------------------------------------------
        Q = np.diag([self._Q_ey, self._Q_eyaw])
        P = self._P_scale * Q

        # Q̄: stage costs on steps 1..N−1 use Q; step N uses P
        Q_bar = np.zeros((2 * N, 2 * N))
        for k in range(N - 1):
            Q_bar[2*k : 2*k+2, 2*k : 2*k+2] = Q
        Q_bar[2*(N-1) : 2*N, 2*(N-1) : 2*N] = P

        # R̄: scalar R repeated N times
        R_bar = self._R_delta * np.eye(N)

        # ------------------------------------------------------------------
        # Hessian H and gradient f
        # ------------------------------------------------------------------
        x0 = np.array([e_y, e_yaw])

        H = S_u.T @ Q_bar @ S_u + R_bar
        # Symmetrise to ensure numerical PSD property
        H = 0.5 * (H + H.T)

        # Add a small ridge for numerical stability (especially with SLSQP)
        H += 1e-8 * np.eye(N)

        f = (S_u.T @ Q_bar @ S_x @ x0).flatten()

        # ------------------------------------------------------------------
        # Constraints:  lb ≤ A_con · u ≤ ub
        #
        # Block 1 (rows 0..N−1):  box on δ
        #   A_con[0:N, :] = I_N,   lb = −delta_max · 1,   ub = +delta_max · 1
        #
        # Block 2 (rows N..2N−1):  rate on δ(k) − δ(k−1)
        #   (D · u)[k] = δ(k) − δ(k−1)  for k ≥ 1
        #   (D · u)[0] = δ(0)             (δ(−1) is subtracted on the bound side)
        #   lb[N+k] = −delta_rate_max − offset,  ub[N+k] = +delta_rate_max − offset
        #   where offset = δ(−1) for k=0 and 0 for k≥1
        # ------------------------------------------------------------------
        delta_max      = self._max_steer_rad
        delta_rate_max = self._delta_rate_max

        # First-difference matrix D (N × N)
        D = np.zeros((N, N))
        D[0, 0] = 1.0                   # δ(0) − δ(−1): δ(−1) handled via bounds offset
        for k in range(1, N):
            D[k, k]   =  1.0
            D[k, k-1] = -1.0

        A_con = np.vstack([np.eye(N), D])

        # Box bounds (block 1)
        lb_box = -delta_max * np.ones(N)
        ub_box =  delta_max * np.ones(N)

        # Rate bounds (block 2): offset lb/ub for k=0 by last_delta
        lb_rate = -delta_rate_max * np.ones(N)
        ub_rate =  delta_rate_max * np.ones(N)
        # For k=0: constraint is δ(0) − last_delta ∈ [−rate_max, +rate_max]
        # i.e.  lb_rate[0] + last_delta ≤ (D·u)[0] ≤ ub_rate[0] + last_delta
        # Rewritten as: lb_rate[0] ≤ (D·u)[0] − last_delta ≤ ub_rate[0]
        # → shift bounds: lb[0] → lb_rate[0] + last_delta, ub[0] → ub_rate[0] + last_delta
        lb_rate[0] += self._last_delta_rad
        ub_rate[0] += self._last_delta_rad

        lb = np.concatenate([lb_box, lb_rate])
        ub = np.concatenate([ub_box, ub_rate])

        return H, f, A_con, lb, ub

    # ======================================================================
    # MPC solve
    # ======================================================================

    def _solve_mpc(self, e_y: float, e_yaw: float, v: float) -> float:
        """
        Solve the condensed QP and return the first steering angle (radians).

        Falls back gracefully on solver failure.
        """
        H, f, A_con, lb, ub = self._build_condensed_qp(e_y, e_yaw, v)
        N = self._N

        if _HAS_OSQP:
            return self._solve_osqp(H, f, A_con, lb, ub, N)
        else:
            return self._solve_slsqp(H, f, A_con, lb, ub, N)

    # ------------------------------------------------------------------
    # OSQP solver path
    # ------------------------------------------------------------------

    def _solve_osqp(self, H, f, A_con, lb, ub, N: int) -> float:
        """
        Solve via OSQP.

        The Hessian H changes every tick because A and B depend on v (LTV
        formulation). Therefore we update P (the cost matrix) on every call
        via solver.update(Px=...) rather than only updating f and bounds.

        OSQP ≥ 0.6 requires the upper-triangular form of P; we pass
        sp.triu(H_sp) at setup and supply only the upper-triangle data on
        updates. This is backward-compatible with older OSQP versions.
        """
        # Always use upper-triangular form for OSQP ≥ 0.6 compatibility
        H_sp     = sp.triu(sp.csc_matrix(H), format='csc')
        A_sp     = sp.csc_matrix(A_con)

        if self._osqp_solver is None or self._osqp_N_cache != N:
            # First call or horizon changed: full setup
            solver = osqp.OSQP()
            solver.setup(
                H_sp, f, A_sp, lb, ub,
                warm_starting=True,
                verbose=False,
                eps_abs=1e-5,
                eps_rel=1e-5,
                max_iter=1000,
                polish=True,
            )
            self._osqp_solver  = solver
            self._osqp_N_cache = N
        else:
            # Warm update: H changes every tick (v-dependent), so update Px too.
            # The sparsity pattern of triu(H) is fixed (dense NxN upper triangle),
            # so we can safely pass only the data array.
            self._osqp_solver.update(Px=H_sp.data, q=f, l=lb, u=ub)

        result = self._osqp_solver.solve()

        if result.info.status_val in (1, 2):   # optimal or optimal_inaccurate
            return float(result.x[0])
        else:
            self.get_logger().warn(
                f'OSQP status: {result.info.status} — returning 0', throttle_duration_sec=1.0
            )
            return 0.0

    # ------------------------------------------------------------------
    # SLSQP fallback
    # ------------------------------------------------------------------

    def _solve_slsqp(self, H, f, A_con, lb, ub, N: int) -> float:
        """Fallback solver using scipy SLSQP (no OSQP available)."""
        # Build bounds list for scipy: list of (lb_i, ub_i) per variable
        # Box constraints (first N rows of A_con = I_N) are already per-variable bounds
        bounds = [(-self._max_steer_rad, self._max_steer_rad)] * N

        def objective(u):
            return 0.5 * float(u @ H @ u) + float(f @ u)

        def gradient(u):
            return (H @ u + f).flatten()

        u0 = np.zeros(N)
        result = minimize(
            objective,
            u0,
            jac=gradient,
            method='SLSQP',
            bounds=bounds,
            constraints=[{'type': 'ineq',
                          'fun':  lambda u: (A_con[N:, :] @ u - lb[N:]).flatten(),
                          'jac':  lambda u: A_con[N:, :]},
                         {'type': 'ineq',
                          'fun':  lambda u: (ub[N:] - A_con[N:, :] @ u).flatten(),
                          'jac':  lambda u: -A_con[N:, :]}],
            options={'maxiter': 200, 'ftol': 1e-8},
        )

        if result.success:
            return float(result.x[0])
        else:
            self.get_logger().warn(
                f'SLSQP failed: {result.message} — returning 0', throttle_duration_sec=1.0
            )
            return 0.0

    # ======================================================================
    # Publishing
    # ======================================================================

    def _publish(self, delta_rad: float):
        """
        Publish ControlCommand with MPC steering (normalised) and PP throttle/brake.

        steering is normalised: steering_norm = delta_rad / max_steer_rad ∈ [−1, 1].
        This matches the convention used by pure_pursuit_node.
        """
        steering_norm = float(np.clip(delta_rad / self._max_steer_rad, -1.0, 1.0))

        cmd = ControlCommand()
        cmd.header           = Header()
        cmd.header.stamp     = self.get_clock().now().to_msg()
        cmd.header.frame_id  = self._map_frame
        cmd.steering         = steering_norm
        cmd.throttle         = self._pp_throttle
        cmd.brake            = self._pp_brake
        self._cmd_pub.publish(cmd)

    # ======================================================================
    # Utilities
    # ======================================================================

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Convert a ROS quaternion message to yaw (radians)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MPCLateralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
