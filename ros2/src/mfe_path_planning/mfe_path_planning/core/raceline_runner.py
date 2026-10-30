"""
Per-cycle race-line state machine around core.raceline (pure numpy, no ROS).

'off' (disabled) -> 'collect' (lap-1 cone union and trail) -> 'built' (window published each cycle), or 'failed'
(the tracer runs for the rest of the run). The node publishes the windows and logs the returned events.
"""
import time
from typing import Optional

import numpy as np

from mfe_path_planning.core import raceline as _raceline

RL_MAX_OFF_M = 2.0     # car further than this from the line -> tracer path for that cycle
RL_MOVE_M = 0.5        # the lap-1 trail starts once the car is this far from its first pose
RACELINE_MISSIONS = ('trackdrive', 'autocross', 'peanut')   # closed-loop missions; the only ones the race line runs in


def raceline_gate(mission: str, enabled: bool) -> bool:
    """Return True when raceline_enable is set and the mission is closed-loop (trackdrive, autocross, peanut)."""
    return bool(enabled) and mission in RACELINE_MISSIONS


class RaceLineRunner:
    def __init__(self, enabled, after_lap, extra_margin_m, a_lat, v_max, window_m):
        self.after = int(after_lap)
        self.margin = float(extra_margin_m)
        self.a_lat = float(a_lat)
        self.v_max = float(v_max)
        self.window_m = float(window_m)
        self.state = 'collect' if enabled else 'off'
        self.p0 = None                           # first odometry position (movement is measured from it)
        self.trail: list = []                    # car trail from first movement, until the build
        self.union = _raceline.ConeUnion() if enabled else None
        self.rl = None                           # the built race line
        self.idx: Optional[int] = None           # race-line index of the last published window
        self.cycles = 0
        self.fallbacks = 0
        self.build_s: Optional[float] = None

    @property
    def enabled(self) -> bool:
        return self.state != 'off'

    def collect_wanted(self, laps_done: int) -> bool:
        """Return True while the lap-1 trail is still being collected from odometry."""
        return self.state == 'collect' and laps_done < self.after

    def collect_odom(self, pos) -> None:
        """Lap-1 trail: from the first odometry point more than RL_MOVE_M from the first pose, until the build."""
        if self.p0 is None:
            self.p0 = pos.copy()
        if self.trail or float(np.hypot(*(pos - self.p0))) > RL_MOVE_M:
            self.trail.append((float(pos[0]), float(pos[1])))

    def cycle(self, car_pos, laps_done: int, cones):
        """
        One planning cycle.

        cones() returns (xy Nx2, colours N) of the current message. Returns (window, events): window = (xy, speeds) of
        the race-line window to publish, or None when the tracer runs this cycle.
        """
        events: list = []
        if self.state == 'off':
            return None, events
        if self.state == 'collect':
            if laps_done < self.after:
                if self.trail:   # union of the cones seen since first movement (lap 1)
                    xy, col = cones()
                    ok = np.isfinite(xy).all(axis=1)
                    self.union.add(xy[ok], col[ok])
                return None, events
            self._build(laps_done, events)
        if self.state != 'built':
            return None, events
        return self._follow(car_pos, events), events

    def _build(self, laps_done: int, events: list) -> None:
        """Build the race line once, from the union and trail frozen at laps_completed = raceline_after_lap."""
        n_cones, n_trail = self.union.n, len(self.trail)
        params = _raceline.RaceLineParams(extra_margin_m=self.margin, a_lat=self.a_lat, v_max=self.v_max)
        t0 = time.perf_counter()
        rl, err = None, None
        try:
            rl = _raceline.build(self.union.xy, np.array(self.trail, float), params, colours=self.union.colours)
        except Exception as e:  # noqa: BLE001 - a failed build must not stop the tracer
            err = f'{type(e).__name__}: {e}'
        self.build_s = time.perf_counter() - t0
        self.union = None
        self.trail = []
        if rl is None:
            self.state = 'failed'
            events.append(('warn', f'race line: build failed ({err or "degenerate input"}) after '
                                   f'{self.build_s:.2f} s; {n_cones} union cones, {n_trail} trail points. '
                                   f'Tracer path for the rest of the run.'))
            return
        self.rl = rl
        self.state = 'built'
        events.append(('info', f'RACE LINE built at laps_completed={laps_done}: length {rl.length_m:.1f} m, '
                               f'predicted lap {rl.lap_s:.2f} s, min side width {rl.min_side_width_m:.2f} m, '
                               f'build {self.build_s:.2f} s ({n_cones} union cones, {n_trail} trail points, '
                               f'a_lat {self.a_lat:g}, margin +{self.margin:g} m)'))

    def _follow(self, car_pos, events: list):
        """
        Return the race-line window of window_m from the car's projection, with target speeds.

        None (tracer this cycle, logged) when the car is more than RL_MAX_OFF_M from the line.
        """
        rl = self.rl
        idx, dist = _raceline.project(rl, car_pos, self.idx)
        if dist > RL_MAX_OFF_M:
            self.fallbacks += 1
            events.append(('warn', f'race line: car {dist:.2f} m from the line, tracer path this cycle '
                                   f'(fallbacks={self.fallbacks})', {'throttle_duration_sec': 2.0}))
            return None
        self.idx = idx
        ii = _raceline.segment_indices(rl, idx, self.window_m)
        self.cycles += 1
        if self.cycles % 100 == 0:
            events.append(('info', f'race line: {self.cycles} windows published, {self.fallbacks} tracer fallbacks'))
        return rl.xy[ii], [float(v) for v in rl.v[ii]]
