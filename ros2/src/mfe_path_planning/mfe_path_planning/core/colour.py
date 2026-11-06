"""
Colour-lap and colour-hint helpers (pure, no ROS): the lap-1 side votes, the hint that picks the.

traced walls' sides from cone colours, and the pipeline latch between the tracer and the library backends.
"""
from mfe_path_planning.core.cone_map import FSD_TYPES

HINT_LOG_EVERY = 50   # colour hint: counter log period (cycles)


class SideVotes:
    """Colour laps: which side (+1 left, -1 right) the lap-1 tracer gave each map cell, summed over the laps."""

    def __init__(self):
        self.votes: dict = {}

    def __len__(self):
        return len(self.votes)

    def record(self, xy, labels, swapped: bool) -> None:
        """
        Record the labels, aligned with xy; swapped means colour_hint exchanged the walls, so the published left is.

        the traced right.
        """
        sign = -1 if swapped else 1
        for (x, y), lab in zip(xy, labels):
            if lab == 0:
                continue
            key = (float(x), float(y))
            self.votes[key] = self.votes.get(key, 0) + sign * (1 if lab > 0 else -1)

    def fill(self, buckets):
        """
        UNKNOWN cones take the side the lap-1 tracer gave their cell (vote > 0 LEFT, < 0 RIGHT).

        Camera colours are never changed. Returns (camera-coloured count, filled count, unfilled count).
        """
        n_cam = len(buckets[FSD_TYPES.left]) + len(buckets[FSD_TYPES.right])
        keep = []
        n_fill = 0
        for rx, ry in buckets[FSD_TYPES.unknown]:
            v = self.votes.get((rx, ry), 0)
            if v > 0:
                buckets[FSD_TYPES.left].append([rx, ry])
                n_fill += 1
            elif v < 0:
                buckets[FSD_TYPES.right].append([rx, ry])
                n_fill += 1
            else:
                keep.append([rx, ry])
        buckets[FSD_TYPES.unknown] = keep
        return n_cam, n_fill, len(keep)


class ColourHint:
    """
    colour_hint: keep the tracer's geometric left/right unless both walls vote confidently and.

    oppositely (one blue-majority, one yellow-majority) against it.
    """

    def __init__(self):
        self.cycles = 0
        self.counts = {'override': 0, 'agree': 0, 'fallback': 0}
        self.swapped = False

    def sides(self, left, right, cones):
        """Return (left, right, events) for the traced walls; cones is the ConeMap that holds the colours."""
        events: list = []
        self.cycles += 1
        self.swapped = False
        v_left, v_right = cones.wall_colour_vote(left), cones.wall_colour_vote(right)
        if v_left != 0 and v_right != 0 and v_left != v_right:
            if v_left == 1:
                self.counts['agree'] += 1
            else:
                self.counts['override'] += 1
                self.swapped = True
                events.append(('info', f'colour hint overrides geometric sides: traced left wall ({len(left)} cones) '
                                       f'votes YELLOW, traced right wall ({len(right)} cones) votes BLUE — swapping '
                                       f'walls', {'throttle_duration_sec': 5.0}))
                left, right = right, left
        else:
            self.counts['fallback'] += 1
        if self.cycles % HINT_LOG_EVERY == 0:
            events.append(('info', f'colour hint: hint_overrides={self.counts["override"]} '
                                   f'hint_agree={self.counts["agree"]} hint_fallback={self.counts["fallback"]}'))
        return left, right, events


class PipelineLatch:
    """
    pipeline_hysteresis latch (local missions).

    Trace -> library once both walls hold min_coloured_each coloured cones; library -> trace after revert_after
    consecutive cycles with no coloured cone at all.
    """

    def __init__(self, min_coloured_each, revert_after):
        self.min_each = int(min_coloured_each)
        self.revert_after = int(revert_after)
        self.trace = True           # True = trace pipeline, False = library
        self.cycles = 0
        self.switches = 0
        self.absent = 0             # consecutive cycles with no BLUE/YELLOW cone in the window

    def update(self, n_left: int, n_right: int) -> list:
        """
        One cycle with the coloured counts in the window.

        Returns the log events.
        """
        events: list = []
        self.cycles += 1
        self.absent = self.absent + 1 if n_left + n_right == 0 else 0
        if self.trace:
            if n_left >= self.min_each and n_right >= self.min_each:
                self._set(False, n_left, n_right, events)
        elif self.absent >= self.revert_after:
            self._set(True, n_left, n_right, events)
        events.append(('debug', f'pipeline mode={"trace" if self.trace else "library"} '
                                f'left={n_left} right={n_right} absent={self.absent}'))
        if self.cycles % 50 == 0:
            events.append(('info', f'pipeline_switches={self.switches}'))
        return events

    def _set(self, trace: bool, n_left: int, n_right: int, events: list) -> None:
        self.trace = trace
        self.switches += 1
        self.absent = 0
        events.append(('info', f'pipeline: {"library->trace" if trace else "trace->library"} '
                               f'(left={n_left} right={n_right})', {'throttle_duration_sec': 1.0}))
