"""
Planner parameters: names, defaults and meaning (ROS-free).

The node declares every field of PlannerParams as a ROS parameter with its default, then reads the values back
with the same types the planner used before.

Groups: mission and backend; colourless and colour-lap planning; the pipeline latch; the jump gate; the accel midline;
cone-map hygiene; the lap-return reset; profiling and debug; the offline race line.
"""
from dataclasses import dataclass, fields


@dataclass
class PlannerParams:
    map_frame: str = 'map'
    mission: str = 'autocross'               # autocross | trackdrive | acceleration | skidpad | peanut
    skidpad_mode: str = 'cone_based'         # cone_based | hardcoded
    # Local (non-global) backend: fasttube = fsd_path_planning.
    planner_backend: str = 'fasttube'
    # Uncoloured cones (LiDAR-only), local missions:
    #   trace         - boundary_tracer labels the walls; its gate-midpoint centreline is used directly
    #   trace_backend - same wall labels fed to the backend as left/right
    #   heading       - legacy: side of the car's heading (fails in corners)
    #   fasttube      - uncoloured cones go to ft-fsd unchanged; its own sorting picks the sides
    #   colour_hint   - trace always (colours stripped); cone colours only vote which traced wall is left/right
    unknown_cone_mode: str = 'trace'
    # Racing-line optimiser on colourless (traced) paths. Off by default: traced paths are short and the
    # optimiser's ~0.27 s per cycle made the path lag the car.
    colourless_raceline: bool = False
    # Colour laps (local missions, unknown_cone_mode colour_hint): from lap colour_after_lap on (laps counted on
    # /planning/laps_completed), plan with the camera's BLUE/YELLOW cones through the library backend instead of
    # the tracer. UNKNOWN cones take the side the lap-1 tracer gave their map cell. 0 = off.
    colour_after_lap: int = 0
    colour_lap_raceline: bool = False        # racing-line optimiser on the colour-lap library path
    # Latch between the trace and library pipelines (local missions). Off: the pipeline is picked per cycle from the
    # visible coloured count, which flip-flops when YOLO colours appear/disappear at the window edge.
    #   min_coloured_each - trace -> library once the window holds this many BLUE and YELLOW cones each
    #   revert_after      - library -> trace after this many consecutive cycles with no coloured cone
    pipeline_hysteresis: bool = False
    min_coloured_each: int = 2
    revert_after: int = 20
    # Jump gate on library / colourless centrelines (before publish). A candidate whose heading (first 5 m) turns more
    # than jump_heading_deg from the last published path, or whose points at 5 m and 10 m are more than jump_lateral_m
    # from it, is replaced by the last published path for up to jump_hold_max consecutive cycles.
    # jump_min_path_m: the gate is skipped when the candidate or the held path is shorter than this.
    # jump_remaining_guard: never hold a stale path whose remaining length ahead of the car is shorter than the fresh
    # candidate's (only with odometry).
    jump_gate: bool = False
    jump_heading_deg: float = 25.0
    jump_lateral_m: float = 1.5
    jump_hold_max: int = 5
    jump_min_path_m: float = 10.0
    jump_remaining_guard: bool = True
    # accel_refit_midline (acceleration only): publish a straight fitted to the midline of the mapped left/right cone
    # pairs ahead of the car, re-fitted every cycle. accel_midline_alpha: low-pass gain on heading and offset.
    accel_refit_midline: bool = True
    accel_midline_alpha: float = 0.3
    # Cone-map colour votes and expiry (see core.cone_map). All off by default: behaviour is unchanged.
    colour_vote: bool = False
    vote_min: int = 3                        # coloured votes needed before a cell takes a colour
    vote_margin: int = 2                     # winner's lead over the other colour
    cone_expiry: bool = False
    expiry_min_obs: int = 3                  # cells seen at least this often never expire
    expiry_cycles: int = 30                  # unseen for this many merge cycles (inside sensing range)
    # Lap-return map reset (local missions, boundary tracer input). On the return to the start after a lap, the cone
    # cells mapped within lap_return_radius_m of the START POSE are excluded from the tracer's map for the rest of the
    # run. Centred on the start pose, not the car: the trigger fires ~start_r away.
    #   lap_return_min_travel_m - travel since leaving the start area before a return counts (capped at
    #                             lap_return_len_frac * the previous lap length once a lap is known)
    #   lap_return_start_r_m    - the start area: the return is inside it, and leaving beyond 2x it re-arms
    lap_return_reset: bool = True
    lap_return_radius_m: float = 25.0
    lap_return_min_travel_m: float = 250.0
    lap_return_start_r_m: float = 15.0
    lap_return_len_frac: float = 0.6
    # Planner-cycle timing: logs mean/p95/max of whole _cones_cb every 50 cycles. Off by default.
    planner_profile: bool = False
    planner_debug_log: str = ''              # per-cycle planner debug log (JSON lines). '' = off
    # Offline race line for laps >= raceline_after_lap (core.raceline). Built once from the lap-1 cone union and trail,
    # then frozen: each cycle publishes raceline_window_m of it from the car's projection, with target speeds on
    # /planning/target_speeds. On by default, but only for the closed-loop missions trackdrive, autocross and peanut
    # (core.raceline_runner RACELINE_MISSIONS); acceleration, skidpad and every other mission never run it.
    raceline_enable: bool = True
    raceline_after_lap: int = 1
    raceline_extra_margin_m: float = 0.25
    raceline_a_lat: float = 6.0
    raceline_v_max: float = 8.0
    raceline_window_m: float = 35.0

    def __post_init__(self):
        """Coerce every value to its field type (ROS may hand back ints for float parameters)."""
        for f in fields(self):
            setattr(self, f.name, f.type(getattr(self, f.name)))


DEFAULTS = {f.name: f.default for f in fields(PlannerParams)}
