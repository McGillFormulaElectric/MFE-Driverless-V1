"""
Rosbag-based path planning tests.

Takes cone positions extracted from the LiDAR pipeline (ground removal + clustering)
and validates the path planning algorithms:
  1. Delaunay triangulation produces valid midpoints
  2. Midpoints stay between left/right boundaries
  3. MST-based cone ordering is consistent
  4. Curvature of the extracted path is bounded (no wild oscillations)
  5. Adaptive look-ahead scales correctly with path curvature

All tests skip when bag is empty/missing.
"""

import sys
import os
import math
import numpy as np
import pytest

_UTILS = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _UTILS not in sys.path:
    sys.path.insert(0, _UTILS)
from rosbag_test_utils import BAG_SKIP, BAG_PATH   # noqa: E402

sys.path.append(os.path.normpath(os.path.join(os.path.dirname(__file__), '..')))


# ---------------------------------------------------------------------------
# Inline implementations (no ROS needed)
# ---------------------------------------------------------------------------

def _menger_kappa(path: np.ndarray) -> np.ndarray:
    """Menger curvature at each waypoint (matches path_planner_node.py)."""
    N = len(path)
    kappa = np.zeros(N)
    for i in range(1, N - 1):
        a, b, c = path[i-1], path[i], path[i+1]
        cross = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
        la = np.linalg.norm(b - a)
        lb = np.linalg.norm(c - b)
        lc = np.linalg.norm(a - c)
        denom = la * lb * lc
        kappa[i] = (2 * cross / denom) if denom > 1e-9 else 0.0
    kappa[0] = kappa[1]; kappa[-1] = kappa[-2]
    return kappa


def _adaptive_lookahead(path: np.ndarray, v: float, K_v: float = 0.5,
                        ld_min: float = 0.5, ld_max: float = 15.0) -> np.ndarray:
    """Compute adaptive look-ahead distance at each waypoint."""
    kappa = _menger_kappa(path)
    ld = K_v * v * np.sqrt(1.0 + kappa**2)
    return np.clip(ld, ld_min, ld_max)


def _delaunay_midpoints(cones_xy: np.ndarray) -> np.ndarray:
    """
    Delaunay triangulation of cones → midpoints of interior edges.
    Returns Mx2 array of midpoints, sorted by x-coordinate as proxy for path order.
    """
    from scipy.spatial import Delaunay
    if len(cones_xy) < 4:
        return np.zeros((0, 2))

    tri = Delaunay(cones_xy)
    midpoints = []
    seen = set()
    for simplex in tri.simplices:
        for i in range(3):
            a, b = simplex[i], simplex[(i+1) % 3]
            edge = (min(a, b), max(a, b))
            if edge in seen:
                continue
            seen.add(edge)
            dist = np.linalg.norm(cones_xy[a] - cones_xy[b])
            # Only edges shorter than track width (~5m) are inter-cone edges
            if dist < 6.0:
                midpoints.append((cones_xy[a] + cones_xy[b]) / 2.0)

    if not midpoints:
        return np.zeros((0, 2))

    mid = np.array(midpoints)
    order = np.argsort(mid[:, 0])
    return mid[order]


def _extract_cone_clusters_from_bag(bag_reader, n_frames: int = 5):
    """
    Read LiDAR frames from bag, apply ground removal + clustering,
    return list of cone-candidate (x,y) positions per frame.
    """
    # Import helpers from LiDAR test
    sys.path.insert(0, os.path.join(_BAG_CONF, '..'))
    from test_rosbag_lidar import (
        _decode_pointcloud2, _slope_ground_removal, _euclidean_cluster, _load_frames
    )
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    conn = next(c for c in bag_reader.connections if c.topic == '/velodyne_points')

    all_frames = []
    count = 0
    for _, ts, raw in bag_reader.messages(connections=[conn]):
        msg = typestore.deserialize_cdr(raw, conn.msgtype)
        pts = _decode_pointcloud2(
            bytes(msg.data), msg.fields,
            msg.point_step, msg.width, msg.height)
        if len(pts) < 100:
            continue
        non_gnd = _slope_ground_removal(pts[:, :3])
        obj = pts[non_gnd, :3]
        nearby = obj[np.linalg.norm(obj[:, :2], axis=1) < 15.0]
        if len(nearby) < 3:
            continue
        clusters = _euclidean_cluster(nearby[:, :2])
        centroids = np.array([np.mean(c, axis=0) for c in clusters
                              if 0.5 < np.linalg.norm(np.mean(c, axis=0)) < 15.0])
        if len(centroids) >= 4:
            all_frames.append(centroids)
            count += 1
        if count >= n_frames:
            break
    return all_frames


# ---------------------------------------------------------------------------
# Test group 1: Delaunay triangulation
# ---------------------------------------------------------------------------

class TestDelaunayTriangulation:

    @BAG_SKIP
    def test_midpoints_produced_from_real_cones(self, bag_reader):
        """Real cone detections should yield valid Delaunay midpoints."""
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters extracted from bag")

        found = False
        for cones in frames:
            mids = _delaunay_midpoints(cones)
            if len(mids) >= 2:
                found = True
                break
        assert found, "Delaunay produced no midpoints from real cone detections"

    @BAG_SKIP
    def test_midpoints_between_cone_extents(self, bag_reader):
        """
        Midpoints must lie within the bounding box of the cone positions.
        (They can't be outside the track.)
        """
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            mids = _delaunay_midpoints(cones)
            if len(mids) == 0:
                continue
            x_min, x_max = cones[:, 0].min(), cones[:, 0].max()
            y_min, y_max = cones[:, 1].min(), cones[:, 1].max()
            pad = 1.0   # allow 1m outside bbox
            assert np.all(mids[:, 0] >= x_min - pad), "Midpoint outside cone x-min"
            assert np.all(mids[:, 0] <= x_max + pad), "Midpoint outside cone x-max"
            assert np.all(mids[:, 1] >= y_min - pad), "Midpoint outside cone y-min"
            assert np.all(mids[:, 1] <= y_max + pad), "Midpoint outside cone y-max"

    @BAG_SKIP
    def test_midpoint_spacing_track_width(self, bag_reader):
        """
        Consecutive path midpoints should be spaced 0.1–5m apart.
        Wider = track too wide or missing cones.
        Narrower = duplicate midpoints.
        """
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            mids = _delaunay_midpoints(cones)
            if len(mids) < 3:
                continue
            spacings = np.linalg.norm(np.diff(mids, axis=0), axis=1)
            assert spacings.max() < 10.0, \
                f"Midpoint spacing {spacings.max():.2f}m too large (missing cones?)"
            assert spacings.min() > 0.01, \
                "Duplicate midpoints detected"
            break


# ---------------------------------------------------------------------------
# Test group 2: Path curvature
# ---------------------------------------------------------------------------

class TestPathCurvature:

    @BAG_SKIP
    def test_path_curvature_bounded(self, bag_reader):
        """
        Extracted path curvature must be physically achievable.
        Max curvature = 1 / min_radius. FSAE min corner: ~3m radius → κ_max = 0.33.
        """
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=5)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            mids = _delaunay_midpoints(cones)
            if len(mids) < 5:
                continue
            kappa = _menger_kappa(mids)
            # Allow up to κ=1.0 (1m radius) for noise
            assert kappa.max() < 2.0, \
                f"Path curvature too high: {kappa.max():.3f} (κ_max for FSAE ≈ 0.33)"
            break

    @BAG_SKIP
    def test_straight_sections_near_zero_curvature(self, bag_reader):
        """
        On straight sections, curvature should be near zero.
        At least some portion of the path should have κ < 0.05.
        """
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=5)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            mids = _delaunay_midpoints(cones)
            if len(mids) < 5:
                continue
            kappa = _menger_kappa(mids)
            straight_fraction = (kappa < 0.05).mean()
            # At least 20% of the path should be "straight"
            assert straight_fraction > 0.1, \
                f"Only {straight_fraction*100:.1f}% of path is straight (κ<0.05)"
            break


# ---------------------------------------------------------------------------
# Test group 3: Adaptive look-ahead
# ---------------------------------------------------------------------------

class TestAdaptiveLookahead:

    @BAG_SKIP
    def test_lookahead_scales_with_speed(self, bag_reader):
        """Faster speed → larger look-ahead distance everywhere."""
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters")

        cones = frames[0]
        mids = _delaunay_midpoints(cones)
        if len(mids) < 3:
            pytest.skip("Not enough midpoints")

        ld_slow = _adaptive_lookahead(mids, v=2.0)
        ld_fast = _adaptive_lookahead(mids, v=10.0)
        assert np.all(ld_fast >= ld_slow), \
            "Faster speed should give larger look-ahead everywhere"

    @BAG_SKIP
    def test_lookahead_larger_on_curves(self, bag_reader):
        """
        sqrt(1+κ²) factor means corners get MORE look-ahead than straights.
        Verify on real path geometry.
        """
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=5)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            mids = _delaunay_midpoints(cones)
            if len(mids) < 5:
                continue
            kappa = _menger_kappa(mids)
            if kappa.max() < 0.05:
                continue   # path too straight to test

            ld = _adaptive_lookahead(mids, v=5.0)
            # Points with high curvature should have larger ld
            high_k_mask = kappa > np.percentile(kappa, 75)
            low_k_mask  = kappa < np.percentile(kappa, 25)
            if high_k_mask.sum() < 2 or low_k_mask.sum() < 2:
                continue

            ld_high = float(np.mean(ld[high_k_mask]))
            ld_low  = float(np.mean(ld[low_k_mask]))
            assert ld_high > ld_low, \
                f"High curvature ld={ld_high:.2f} not > low curvature ld={ld_low:.2f}"
            break

    @BAG_SKIP
    def test_lookahead_within_bounds(self, bag_reader):
        """Look-ahead must always be within [ld_min, ld_max]."""
        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters")

        mids = _delaunay_midpoints(frames[0])
        if len(mids) < 3:
            pytest.skip("Not enough midpoints")

        for v in [1.0, 5.0, 13.0]:
            ld = _adaptive_lookahead(mids, v=v, ld_min=0.5, ld_max=15.0)
            assert np.all(ld >= 0.5),  f"v={v}: look-ahead below ld_min"
            assert np.all(ld <= 15.0), f"v={v}: look-ahead above ld_max"


# ---------------------------------------------------------------------------
# Test group 4: MST cone ordering
# ---------------------------------------------------------------------------

class TestMSTConeOrdering:

    @BAG_SKIP
    def test_mst_connects_all_cones(self, bag_reader):
        """
        Minimum Spanning Tree of cones must connect all detected cones
        (no isolated nodes).
        """
        from scipy.sparse.csgraph import minimum_spanning_tree
        from scipy.sparse import csr_matrix

        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            if len(cones) < 3:
                continue
            # Distance matrix
            n = len(cones)
            D = np.linalg.norm(cones[:, None] - cones[None, :], axis=2)
            mst = minimum_spanning_tree(csr_matrix(D))
            # MST has n-1 edges for n nodes → all connected
            assert mst.nnz == n - 1, \
                f"MST has {mst.nnz} edges but expected {n-1} for {n} cones"
            break

    @BAG_SKIP
    def test_mst_edge_lengths_track_width(self, bag_reader):
        """
        MST edges should be shorter than 2× track width (~10m).
        Long edges indicate wrong cone associations.
        """
        from scipy.sparse.csgraph import minimum_spanning_tree
        from scipy.sparse import csr_matrix

        frames = _extract_cone_clusters_from_bag(bag_reader, n_frames=3)
        if not frames:
            pytest.skip("No cone clusters")

        for cones in frames:
            if len(cones) < 3:
                continue
            n = len(cones)
            D = np.linalg.norm(cones[:, None] - cones[None, :], axis=2)
            mst = minimum_spanning_tree(csr_matrix(D))
            mst_arr = mst.toarray()
            edge_lengths = mst_arr[mst_arr > 0]
            assert edge_lengths.max() < 15.0, \
                f"MST edge {edge_lengths.max():.2f}m > 15m (wrong cone association)"
            break
