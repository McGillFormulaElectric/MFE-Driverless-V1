"""
Unit tests for Hungarian + Kalman cone tracking in boundary_extractor.

Tests the _update_tracks() method and ConeTrack dataclass added in
neil/feature/cone-tracking.

No ROS2 runtime needed (conftest.py mocks all imports).
"""

import sys
import os
import math
import numpy as np
import pytest

# Append so PYTHONPATH (worktree override) takes priority.
_local = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
if _local not in sys.path:
    sys.path.append(_local)

try:
    from mfe_path_planning.boundary_extractor import BoundaryExtractor, ConeTrack
    from mfe_msgs.msg import Cone as _Cone
    _TRACKING_AVAILABLE = True
except Exception:
    _TRACKING_AVAILABLE = False


def _make_cone(x, y, color=None):
    """Create a stub Cone at map-frame (x, y)."""
    from mfe_msgs.msg import Cone
    c = Cone()
    c.location.x = x
    c.location.y = y
    c.location.z = 0.0
    c.color = color if color is not None else Cone.UNKNOWN
    return c


def _make_extractor():
    """Instantiate BoundaryExtractor with minimal param stubs."""
    be = object.__new__(BoundaryExtractor)
    be._tracks        = []
    be._next_track_id = 0
    be._min_hits      = 3
    be._max_misses    = 5
    be._match_r       = 1.0
    return be


pytestmark = pytest.mark.skipif(
    not _TRACKING_AVAILABLE,
    reason='ConeTrack/BoundaryExtractor not available — run on neil/feature/cone-tracking branch'
)


# ---------------------------------------------------------------------------
# Test group 1: track lifecycle
# ---------------------------------------------------------------------------

class TestTrackLifecycle:

    def test_new_detection_creates_tentative_track(self):
        """First detection of a cone creates a track with hits=1."""
        be = _make_extractor()
        cones = [_make_cone(5.0, 3.0)]
        be._update_tracks(cones)
        assert len(be._tracks) == 1
        assert be._tracks[0].hits == 1

    def test_track_confirmed_after_min_hits(self):
        """Track becomes confirmed (returned in output) after min_hits frames."""
        be = _make_extractor()
        cone = _make_cone(5.0, 3.0)
        confirmed = []
        for _ in range(be._min_hits):
            confirmed = be._update_tracks([cone])
        # After exactly min_hits observations, track should be confirmed
        assert len(confirmed) == 1, \
            f"Track not confirmed after {be._min_hits} hits (got {len(confirmed)})"

    def test_track_deleted_after_max_misses(self):
        """Track disappears after max_misses consecutive missed frames."""
        be = _make_extractor()
        cone = _make_cone(5.0, 3.0)
        # Confirm the track
        for _ in range(be._min_hits + 1):
            be._update_tracks([cone])
        assert len(be._tracks) == 1
        # Now miss it for max_misses frames
        for _ in range(be._max_misses + 1):
            be._update_tracks([])
        assert len(be._tracks) == 0, \
            f"Track not deleted after {be._max_misses} misses"

    def test_confirmed_track_not_deleted_on_single_miss(self):
        """One missed frame should not delete a confirmed track."""
        be = _make_extractor()
        cone = _make_cone(5.0, 3.0)
        for _ in range(be._min_hits + 2):
            be._update_tracks([cone])
        # One miss
        be._update_tracks([])
        assert len(be._tracks) == 1, "Track deleted after single miss"


# ---------------------------------------------------------------------------
# Test group 2: Hungarian data association
# ---------------------------------------------------------------------------

class TestDataAssociation:

    def test_same_cone_same_track(self):
        """Same cone detected in 10 consecutive frames → 1 track, not 10."""
        be = _make_extractor()
        cone = _make_cone(10.0, 5.0)
        for _ in range(10):
            be._update_tracks([cone])
        assert len(be._tracks) == 1, \
            f"Expected 1 track, got {len(be._tracks)}"

    def test_two_distinct_cones_two_tracks(self):
        """Two cones far apart → two distinct tracks."""
        be = _make_extractor()
        c1 = _make_cone(0.0,  0.0)
        c2 = _make_cone(10.0, 0.0)
        for _ in range(5):
            be._update_tracks([c1, c2])
        assert len(be._tracks) == 2, \
            f"Expected 2 tracks, got {len(be._tracks)}"

    def test_cone_within_match_radius_associates(self):
        """Cone moved by 0.3m (< match_radius=1.0m) should stay on same track."""
        be = _make_extractor()
        c1 = _make_cone(5.0, 5.0)
        be._update_tracks([c1])
        initial_id = be._tracks[0].id

        # Move cone slightly — still within match_radius
        c2 = _make_cone(5.3, 5.1)
        be._update_tracks([c2])
        assert len(be._tracks) == 1
        assert be._tracks[0].id == initial_id, \
            "Cone within match_radius spawned a new track"

    def test_cone_outside_match_radius_new_track(self):
        """Cone farther than match_radius=1.0m should create a new track."""
        be = _make_extractor()
        c1 = _make_cone(5.0, 5.0)
        be._update_tracks([c1])
        n_tracks_before = len(be._tracks)

        c2 = _make_cone(10.0, 10.0)  # far away — should not match
        be._update_tracks([c2])
        assert len(be._tracks) == n_tracks_before + 1, \
            "No new track created for cone outside match_radius"


# ---------------------------------------------------------------------------
# Test group 3: Kalman position filtering
# ---------------------------------------------------------------------------

class TestKalmanFiltering:

    def test_kalman_smooths_noisy_detections(self):
        """
        Cone at true position (5, 3) observed with noise.
        Kalman-filtered output should be closer to truth than raw observations.
        """
        be = _make_extractor()
        true_x, true_y = 5.0, 3.0
        noise_std = 0.3

        rng = np.random.default_rng(42)
        for _ in range(20):
            nx = true_x + rng.normal(0, noise_std)
            ny = true_y + rng.normal(0, noise_std)
            be._update_tracks([_make_cone(nx, ny)])

        track = be._tracks[0]
        kalman_x = float(track.state[0])
        kalman_y = float(track.state[1])
        err = math.hypot(kalman_x - true_x, kalman_y - true_y)
        assert err < noise_std, \
            f"Kalman estimate ({kalman_x:.3f}, {kalman_y:.3f}) worse than noise std {noise_std}"

    def test_published_position_is_kalman_not_raw(self):
        """
        Confirmed track's published position should be the Kalman state,
        not the last raw detection.
        """
        be = _make_extractor()
        true_x, true_y = 8.0, 4.0

        rng = np.random.default_rng(0)
        confirmed = []
        last_raw_x = last_raw_y = 0.0
        for _ in range(50):
            nx = true_x + rng.normal(0, 0.5)
            ny = true_y + rng.normal(0, 0.5)
            last_raw_x, last_raw_y = nx, ny
            confirmed = be._update_tracks([_make_cone(nx, ny)])

        assert len(confirmed) > 0, "No confirmed tracks after 50 frames"
        c = confirmed[0]
        kx = float(c.state[0])
        ky = float(c.state[1])
        kalman_err = math.hypot(kx - true_x, ky - true_y)
        raw_err    = math.hypot(last_raw_x - true_x, last_raw_y - true_y)
        # Kalman estimate must be better than last single raw observation
        assert kalman_err < raw_err, (
            f"Kalman ({kx:.3f},{ky:.3f}) err={kalman_err:.3f}m "
            f"not better than raw err={raw_err:.3f}m")


# ---------------------------------------------------------------------------
# Test group 4: color refinement
# ---------------------------------------------------------------------------

class TestColorRefinement:

    def test_unknown_color_updated_when_camera_provides_color(self):
        """
        Track starts UNKNOWN (LiDAR only). When camera provides color,
        track color should update.
        """
        from mfe_msgs.msg import Cone
        be = _make_extractor()

        # First few frames: UNKNOWN (LiDAR only)
        for _ in range(2):
            be._update_tracks([_make_cone(5.0, 3.0, color=Cone.UNKNOWN)])

        # Camera now provides color
        be._update_tracks([_make_cone(5.0, 3.0, color=Cone.BLUE)])

        assert be._tracks[0].color == Cone.BLUE, \
            f"Track color not updated: {be._tracks[0].color}"

    def test_confirmed_color_not_overwritten_by_unknown(self):
        """Once color is confirmed, UNKNOWN observations should not overwrite it."""
        from mfe_msgs.msg import Cone
        be = _make_extractor()

        # Establish YELLOW track
        for _ in range(5):
            be._update_tracks([_make_cone(5.0, 3.0, color=Cone.YELLOW)])

        # UNKNOWN detection should not overwrite
        be._update_tracks([_make_cone(5.0, 3.0, color=Cone.UNKNOWN)])

        assert be._tracks[0].color == Cone.YELLOW, \
            "Known color overwritten by UNKNOWN"
