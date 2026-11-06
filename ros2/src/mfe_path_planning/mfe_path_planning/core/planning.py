"""
One planning cycle's decisions (pure, no ROS): which pipeline runs (library, colourless tracer.

colour laps, skidpad local fallback), with the outputs as a Plan that the node publishes and logs.

The library planner is the fsd_path_planning / ft_fsd_path_planning object (duck-typed: calculate_path_in_global_frame).
"""
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from mfe_path_planning.core.cone_map import CONE_BEHIND_M, bucket_counts
from mfe_path_planning.core.paths import cone_arrays
from mfe_path_planning.core.racing_line import _min_curvature_path

OFF_MAP_M = 20.0          # car this far from every mapped cone: off track, publish nothing
COLOUR_LOG_EVERY = 50     # colour laps: summary log period (cycles)


@dataclass
class PlanContext:
    """Per-cycle inputs from the node."""

    car_pos: np.ndarray
    car_dir: np.ndarray
    laps_done: int
    mission: str
    global_planning: bool
    now_s: float = 0.0


@dataclass
class Plan:
    """
    What the node does with one cycle.

    publish: centreline (jump gate applied by the node), left, right. horizon: value for /planning/path_horizon
    (None: not published). pre_cycles / cycles: planner_debug_log records (logged before / after the publish).
    events: (level, text[, kwargs]) log lines.
    """

    publish: bool = False
    centerline: Optional[np.ndarray] = None
    left: object = None
    right: object = None
    horizon: Optional[float] = None
    pre_cycles: List[dict] = field(default_factory=list)
    cycles: List[dict] = field(default_factory=list)
    events: list = field(default_factory=list)


def _horizon(xy) -> float:
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


class LocalPlanner:
    """
    Owns the per-cycle pipeline choice and the colour-lap counters.

    The cone map, lap ban, side votes, colour hint, pipeline latch and accel midline are shared with the node and
    passed in.
    """

    def __init__(self, types, planner, tracer, cones, lap_ban, side_votes, hint, pipeline, accel, *, unknown_mode,
                 colour_after_lap, pipeline_hysteresis, accel_refit):
        self.types = types
        self.planner = planner
        self.tracer = tracer
        self.cones = cones
        self.lap_ban = lap_ban
        self.side_votes = side_votes
        self.hint = hint
        self.pipeline = pipeline
        self.accel = accel
        self.unknown_mode = unknown_mode
        self.colour_after_lap = colour_after_lap
        self.pipeline_hysteresis = pipeline_hysteresis
        self.accel_refit = accel_refit
        self.colour_lap_active = False      # colour planning has started (logged once)
        self.colour_cycles = 0
        self.colour_fallbacks = 0
        self.colour_cam_sum = 0             # camera-coloured cones per colour-lap cycle (for the mean)
        self.colour_fill_sum = 0            # side-vote-filled UNKNOWN cones per colour-lap cycle
        self.last_lib_error = None          # reason set by _run_library when it returns None on an exception

    # ------------------------------------------------------------------ cycle dispatch

    def plan(self, ctx: PlanContext) -> Plan:
        """Local planning for one cone message (after the lap-1 lock-in and the race line)."""
        events: list = []
        use_full_map = ctx.global_planning or ctx.mission == 'acceleration'
        buckets = self.cones.bucket(not use_full_map, ctx.car_pos, ctx.car_dir)
        n_left, n_right, n_unknown = bucket_counts(buckets, self.types)

        # Colour laps: from colour_after_lap completed laps on, the camera colours plan through the library backend
        # (UNKNOWN cones side-filled from the lap-1 tracer votes). Before the colour_hint check; off with 0.
        if self.colour_after_lap > 0 and ctx.laps_done >= self.colour_after_lap and not use_full_map:
            if not self.colour_lap_active:
                self.colour_lap_active = True
                events.append(('info', f'COLOUR LAPS ON: laps_completed={ctx.laps_done} '
                                       f'(colour_after_lap={self.colour_after_lap}) t={ctx.now_s:.1f}s: planning with '
                                       f'camera BLUE/YELLOW cones via the library backend; side votes='
                                       f'{len(self.side_votes)} cells'))
            return self._plan_colour_lap(ctx, buckets, events)

        # colour_hint: always trace the walls (colours stripped); the colours only pick the left/right labels.
        # Checked first so it wins over the pipeline_hysteresis latch, which is trace/trace_backend only.
        if not use_full_map and self.unknown_mode == 'colour_hint':
            return self._plan_colourless(ctx, True, events)

        # Colourless local planning: trace the walls instead of guessing sides from the heading.
        trace_ok = not use_full_map and self.unknown_mode in ('trace', 'trace_backend')
        if self.pipeline_hysteresis:
            if trace_ok:
                events += self.pipeline.update(n_left, n_right)
            trace_now = trace_ok and self.pipeline.trace and n_unknown > 0
        else:
            trace_now = trace_ok and n_left + n_right == 0 and n_unknown > 0
        if trace_now:
            return self._plan_colourless(ctx, False, events)

        events.append(('info', f'planner input: left={n_left} right={n_right} unknown={n_unknown} '
                               f'pos=({ctx.car_pos[0]:.1f},{ctx.car_pos[1]:.1f}) '
                               f'dir=({ctx.car_dir[0]:.2f},{ctx.car_dir[1]:.2f})', {'throttle_duration_sec': 2.0}))

        # fasttube: hand uncoloured cones to ft-fsd unchanged so its own sorting picks the sides.
        colourless_native = (n_left + n_right == 0 and n_unknown > 0 and self.unknown_mode == 'fasttube')
        cones_by_type = cone_arrays(buckets)
        if n_left + n_right == 0 and n_unknown > 0 and not colourless_native:
            self.cones.assign_unknown_sides(ctx.car_pos, ctx.car_dir, buckets)
            cones_by_type = cone_arrays(buckets)
            n_left, n_right = len(buckets[self.types.left]), len(buckets[self.types.right])
            events.append(('info', f'LiDAR-only: geometry L/R assignment → left={n_left} right={n_right}',
                           {'throttle_duration_sec': 2.0}))

        if n_left + n_right == 0 and not colourless_native:
            events.append(('warn', 'No colored or assignable cones — skipping planner.',
                           {'throttle_duration_sec': 5.0}))
            return Plan(events=events)

        result = self._run_library(cones_by_type, n_left, n_right, n_unknown, ctx, events)
        if result is None:
            return Plan(events=events)
        path_xy, left_xy, right_xy = result
        events.append(('info', f'planner OK: path_pts={len(path_xy) if path_xy is not None else 0}',
                       {'throttle_duration_sec': 2.0}))

        # final_path columns: (spline_parameter, x, y, curvature); boundaries are already (x, y).
        centerline = None
        if path_xy is not None and len(path_xy) > 0:
            centerline = np.array(path_xy)[:, 1:3]
            # Race line for local missions. Not during global exploration: those paths are ephemeral (the
            # lap-1 lock-in optimises once) and a per-cycle optimiser pushes the replan period past pure
            # pursuit's stale threshold.
            if (not ctx.global_planning
                    and not colourless_native
                    and len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                centerline = self._racing_line(centerline, np.array(left_xy)[:, :2], np.array(right_xy)[:, :2],
                                               events)
            if self.accel_refit and ctx.mission == 'acceleration' and len(centerline) > 1:
                centerline, (accepted, why) = self.accel.path(np.asarray(centerline), self.cones.xy(),
                                                              ctx.car_pos, ctx.car_dir)
                events.append(('info', f'accel midline {"accepted" if accepted else "rejected"}: {why}',
                               {'throttle_duration_sec': 1.0}))
        return Plan(publish=True, centerline=centerline, left=left_xy, right=right_xy, events=events)

    def skidpad_local(self, ctx: PlanContext, has_cones: bool) -> Plan:
        """Local planning for skidpad states without a locked circle (APPROACH, EXIT, incomplete)."""
        events: list = []
        if self.planner is None or not has_cones:
            return Plan(events=events)
        buckets = self.cones.bucket(True, ctx.car_pos, ctx.car_dir)
        cones_by_type = cone_arrays(buckets)
        n_left, n_right, n_unknown = bucket_counts(buckets, self.types)
        if n_left + n_right == 0 and n_unknown > 0:
            self.cones.assign_unknown_sides(ctx.car_pos, ctx.car_dir, buckets)
            cones_by_type = cone_arrays(buckets)
            n_left, n_right = len(buckets[self.types.left]), len(buckets[self.types.right])

        if n_left + n_right == 0:
            events.append(('warn', 'No colored or assignable cones — skipping planner.',
                           {'throttle_duration_sec': 5.0}))
            return Plan(events=events)

        result = self._run_library(cones_by_type, n_left, n_right, n_unknown, ctx, events)
        if result is None:
            return Plan(events=events)
        path_xy, left_xy, right_xy = result
        centerline = None
        if path_xy is not None and len(path_xy) > 0:
            centerline = np.array(path_xy)[:, 1:3]
            if (len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                try:
                    centerline = _min_curvature_path(
                        centerline, np.array(left_xy)[:, :2], np.array(right_xy)[:, :2], safety=0.7)
                except Exception as opt_e:
                    events.append(('debug', f'Optimizer failed: {opt_e}'))
        return Plan(publish=True, centerline=centerline, left=left_xy, right=right_xy, events=events)

    # ------------------------------------------------------------------ pipelines

    def _plan_colourless(self, ctx: PlanContext, colour_hint: bool, events: list) -> Plan:
        """
        No cone colours: the boundary tracer labels each wall from the driven trajectory and grows both.

        walls ahead together. The path ends where the paired walls end. colour_hint: the traced walls keep the
        tracer's sides unless the cone colours vote otherwise (ColourHint).
        """
        xy = self.tracer_cone_xy()
        if len(xy) == 0 or self._off_map(xy, ctx, events):
            reason = 'no cones' if len(xy) == 0 else 'off map'
            return Plan(events=events, pre_cycles=[dict(n_cones=len(xy), reason=reason, dbg=None)])
        res = self.tracer.plan(xy, ctx.car_pos, ctx.car_dir)
        if res is None:
            reason = self.tracer.debug.get('reason')
            events.append(('warn', f'boundary tracer: no path ({reason})', {'throttle_duration_sec': 2.0}))
            return Plan(events=events, pre_cycles=[dict(n_cones=len(xy), reason=reason, dbg=self.tracer.debug)])
        path, left, right = res.path, res.left, res.right
        swapped = False
        if colour_hint:
            left, right, hint_events = self.hint.sides(left, right, self.cones)
            events += hint_events
            swapped = self.hint.swapped
        self.side_votes.record(xy, res.labels, swapped)
        if self.unknown_mode == 'trace_backend':
            path = self._backend_on_labels(xy, res, ctx, events)
            if path is None:
                return Plan(events=events)
        horizon = _horizon(path)
        events.append(('info', f'boundary tracer: left={len(left)} right={len(right)} '
                               f'gates={res.debug.get("n_gates")} labelled={res.debug.get("n_labelled")} '
                               f'horizon={horizon:.1f} m', {'throttle_duration_sec': 2.0}))
        return Plan(publish=True, centerline=path, left=left, right=right, horizon=horizon,
                    cycles=[dict(n_cones=len(xy), reason='ok', dbg=self.tracer.debug, horizon=horizon)],
                    events=events)

    def _plan_colour_lap(self, ctx: PlanContext, buckets, events: list) -> Plan:
        """
        Colour laps: plan with the camera's BLUE/YELLOW cones (UNKNOWN ones side-filled from the lap-1.

        votes) through the library backend. Falls back to the colour_hint tracer for the cycle when the library fails.
        """
        n_cam, n_fill, n_unk = self.side_votes.fill(buckets)
        n_left, n_right = len(buckets[self.types.left]), len(buckets[self.types.right])
        n_total = n_left + n_right + n_unk
        reason = None
        result = None
        if n_left + n_right == 0:
            reason = 'no blue/yellow cones after side fill'
        else:
            self.last_lib_error = None
            try:
                result = self._run_library(cone_arrays(buckets), n_left, n_right, n_unk, ctx, events)
            except Exception as e:  # noqa: BLE001 - a library failure must fall back, not stop planning
                result = None
                reason = f'{type(e).__name__}: {e}'
            if reason is None and result is None:
                reason = self.last_lib_error or 'library returned None'
            elif reason is None and (result[0] is None or len(result[0]) == 0):
                reason = 'empty path'
        if reason is not None:
            self.colour_fallbacks += 1
            events.append(('warn', f'colour lap: library fallback to colour_hint tracer ({reason}); '
                                   f'fallbacks={self.colour_fallbacks}', {'throttle_duration_sec': 5.0}))
            fallback = dict(n_cones=n_total, reason=f'fallback: {reason}', dbg=None, mode='colour_lib')
            sub = self._plan_colourless(ctx, True, events)
            sub.pre_cycles.insert(0, fallback)
            return sub
        path_xy, left_xy, right_xy = result
        centerline = np.array(path_xy)[:, 1:3]
        horizon = _horizon(centerline)
        self.colour_cycles += 1
        self.colour_cam_sum += n_cam
        self.colour_fill_sum += n_fill
        if self.colour_cycles % COLOUR_LOG_EVERY == 0:
            events.append(('info', f'colour laps: colour_cycles={self.colour_cycles} fallbacks={self.colour_fallbacks} '
                                   f'mean_camera_coloured={self.colour_cam_sum / self.colour_cycles:.1f} '
                                   f'mean_side_filled={self.colour_fill_sum / self.colour_cycles:.1f} '
                                   f'laps_done={ctx.laps_done}'))
        return Plan(publish=True, centerline=centerline, left=left_xy, right=right_xy,
                    cycles=[dict(n_cones=n_total, reason='ok', dbg=None, horizon=horizon, mode='colour_lib')],
                    events=events)

    # ------------------------------------------------------------------ helpers

    def _off_map(self, xy: np.ndarray, ctx: PlanContext, events: list) -> bool:
        """
        Return True (and warn) when the car is far from every mapped cone: publish nothing, so pure pursuit's.

        stale-path failsafe brakes.
        """
        d_nearest = float(np.min(np.hypot(xy[:, 0] - ctx.car_pos[0], xy[:, 1] - ctx.car_pos[1])))
        if d_nearest > OFF_MAP_M:
            events.append(('warn', f'Car is {d_nearest:.0f} m from the nearest mapped cone — off track? '
                                   'Skipping planner so the stale-path failsafe stops the car.',
                           {'throttle_duration_sec': 2.0}))
            return True
        return False

    def _run_library(self, cones_by_type, n_left, n_right, n_unknown, ctx: PlanContext, events: list):
        """
        ft-fsd call.

        Returns (path, left, right) or None when skipped or failed.
        """
        if self._off_map(self.cones.xy(), ctx, events):
            return None
        try:
            out = self.planner.calculate_path_in_global_frame(
                cones_by_type, ctx.car_pos, ctx.car_dir,
                return_intermediate_results=True,
            )
        except Exception as e:
            self.last_lib_error = f'{type(e).__name__}: {e}'
            events.append(('error', f'Path calculation failed ({type(e).__name__}): {e} | '
                                    f'left={n_left} right={n_right} unknown={n_unknown}',
                           {'throttle_duration_sec': 2.0}))
            return None
        return out[0], out[1], out[2]

    def _racing_line(self, centerline, left, right, events: list):
        """Race line through the corridor: the minimum-curvature path; the raw centerline if it fails."""
        try:
            out = _min_curvature_path(centerline, left, right, safety=0.7)
            events.append(('info', 'Racing line optimizer applied.', {'throttle_duration_sec': 5.0}))
            return out
        except Exception as opt_e:
            events.append(('warn', f'Racing line optimizer failed ({type(opt_e).__name__}): {opt_e} — '
                                   'falling back to raw centerline.', {'throttle_duration_sec': 5.0}))
            return centerline

    def tracer_cone_xy(self) -> np.ndarray:
        """Map cells handed to the boundary tracer: the cone map minus the lap-return exclusions."""
        if not self.lap_ban.banned:
            return self.cones.xy()
        keys = self.lap_ban.visible(self.cones.keys())
        return np.array(keys, dtype=np.float64).reshape(-1, 2)

    def _backend_on_labels(self, xy, res, ctx: PlanContext, events: list):
        """
        Run the configured backend on the tracer's left/right labels, then cut its path where the traced.

        walls end (backends extrapolate past the last cones).
        """
        rel = xy - ctx.car_pos
        ahead = rel @ ctx.car_dir >= -CONE_BEHIND_M
        Lm = ahead & (res.labels > 0)
        Rm = ahead & (res.labels < 0)
        cones = [np.zeros((0, 2)) for _ in range(self.types.n)]
        cones[self.types.left] = xy[Lm]
        cones[self.types.right] = xy[Rm]
        try:
            path = np.array(self.planner.calculate_path_in_global_frame(
                cones, ctx.car_pos, ctx.car_dir))[:, 1:3]
        except Exception as e:
            events.append(('error', f'Path calculation failed: {type(e).__name__}: {e}',
                           {'throttle_duration_sec': 2.0}))
            return None
        if len(path) < 2:
            return None
        end = res.path[-1]
        j = int(np.argmin(np.linalg.norm(path - end, axis=1)))
        return path[:j + 1] if j >= 1 else None
