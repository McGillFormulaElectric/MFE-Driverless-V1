"""
Jump gate on the library / colourless centreline: holds the last published path when a candidate.

jumps (pure).
"""
import math

from mfe_common.geometry import arc_length_guarded, wrap_angle
from mfe_path_planning.core.paths import dist_to_polyline, path_point_at, remaining_ahead

JUMP_HEAD_M = 5.0          # heading is taken over the first 5 m of the path (as the diag 'jumps' metric)
JUMP_LAT_M = (5.0, 10.0)   # lateral offsets checked at these arc lengths ahead (m)
JUMP_WARN_S = 1.0          # throttle for the rejection WARN
JUMP_LOG_EVERY = 50        # counter log period (cycles)


class JumpGate:
    """
    A candidate whose heading (first 5 m) turns more than heading_deg from the last published path, or.

    whose points at 5 m and 10 m are more than lateral_m from it, is replaced by the last published path for up to
    hold_max consecutive cycles; after that the candidate is accepted so a genuine change gets through.

    apply() returns (path to publish, log events). `last` is the decision of the last call: 'off', 'skip', 'hold',
    'ok' or 'accept_after_hold'.
    """

    def __init__(self, enabled, heading_deg, lateral_m, hold_max, min_path_m, remaining_guard):
        self.enabled = bool(enabled)
        self.heading_deg = float(heading_deg)
        self.lateral_m = float(lateral_m)
        self.hold_max = int(hold_max)
        self.min_path_m = float(min_path_m)
        self.remaining_guard = bool(remaining_guard)
        self.prev = None           # last published centreline
        self.hold = 0             # consecutive held cycles
        self.cycles = 0
        self.last = 'off'
        self.rejects = 0
        self.accepted_after_hold = 0

    def _accept(self, xy):
        self.prev = xy
        self.hold = 0
        return xy

    def apply(self, xy, car_pos, odom_received):
        """Return (centreline to publish, events) for the candidate xy at car_pos."""
        events: list = []
        self.last = 'skip'
        if not self.enabled or self.prev is None or len(xy) < 2 or len(self.prev) < 2:
            return self._accept(xy), events
        prev = self.prev
        # Short paths: the 5 m / 10 m probes clamp to the path end, so the heading and lateral test is meaningless.
        if arc_length_guarded(xy) < self.min_path_m or arc_length_guarded(prev) < self.min_path_m:
            return self._accept(xy), events
        h_new = math.atan2(*(path_point_at(xy, JUMP_HEAD_M) - xy[0])[::-1])
        h_prev = math.atan2(*(path_point_at(prev, JUMP_HEAD_M) - prev[0])[::-1])
        dh = math.degrees(abs(wrap_angle(h_new - h_prev)))
        lat = [dist_to_polyline(path_point_at(xy, s), prev) for s in JUMP_LAT_M]
        bad = dh > self.heading_deg or max(lat) > self.lateral_m
        self.cycles += 1
        # Never hold a stale path that has less road ahead of the car than the fresh candidate: that would trade
        # a long path for a shorter one.
        hold_ok = self.hold < self.hold_max
        if bad and hold_ok and self.remaining_guard and odom_received:
            if remaining_ahead(prev, car_pos) < remaining_ahead(xy, car_pos):
                hold_ok = False

        if bad and hold_ok:
            self.last = 'hold'
            self.hold += 1
            self.rejects += 1
            events.append(('warn', f'jump gate: rejected centreline (dheading={dh:.1f} deg, lat5={lat[0]:.2f} m, '
                                   f'lat10={lat[1]:.2f} m; hold {self.hold}/{self.hold_max}) — republishing '
                                   f'previous path', {'throttle_duration_sec': JUMP_WARN_S}))
            xy = prev
        else:
            if bad:
                self.accepted_after_hold += 1
                events.append(('warn', f'jump gate: accepting jump after {self.hold} held cycles '
                                       f'(dheading={dh:.1f} deg, lat5={lat[0]:.2f} m, lat10={lat[1]:.2f} m)',
                               {'throttle_duration_sec': JUMP_WARN_S}))
            self.last = 'accept_after_hold' if bad else 'ok'
            self._accept(xy)

        if self.cycles % JUMP_LOG_EVERY == 0:
            events.append(('info', f'jump_rejects={self.rejects} accepted_after_hold={self.accepted_after_hold} '
                                   f'cycles={self.cycles}'))
        return xy, events
