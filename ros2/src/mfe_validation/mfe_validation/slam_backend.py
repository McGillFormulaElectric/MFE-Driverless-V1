"""SLAM optimiser interface, plain SE(2) helpers and the backend registry.

A backend owns the factor graph and its solver. Poses are (x, y, theta) and landmarks (x, y) in the SLAM frame, as plain
floats. To add an optimiser, subclass SlamBackend in its own module and add one entry to BACKENDS.
"""
import importlib
import math
from abc import ABC, abstractmethod

BACKENDS = {'gtsam': ('mfe_validation.gtsam_backend', 'GtsamBackend')}


class SlamBackend(ABC):
    """obs_sigmas = (bearing_rad, range_m) noise of landmark observations, owned by the backend."""

    @abstractmethod
    def add_pose_prior(self, i, pose, sigmas):
        """Anchor pose i to pose (x, y, theta) with sigmas (x, y, theta)."""

    @abstractmethod
    def add_odometry(self, i, j, dx, dy, dtheta, sigmas):
        """Relative-motion factor: pose j = pose i composed with (dx, dy, dtheta)."""

    @abstractmethod
    def add_landmark_observation(self, pose_index, landmark_id, bearing_rad, range_m):
        """Landmark seen from pose_index at bearing_rad (sensor frame) and range_m."""

    @abstractmethod
    def init_pose(self, i, pose):
        """Initial guess for pose i (used when it is first added)."""

    @abstractmethod
    def init_landmark(self, landmark_id, xy):
        """Initial guess for a new landmark."""

    @abstractmethod
    def update(self):
        """Incorporate everything added since the last update and re-estimate."""

    @abstractmethod
    def pose(self, i):
        """Estimated pose i as (x, y, theta) after the last update."""

    @abstractmethod
    def landmark(self, landmark_id):
        """Estimated landmark (x, y) after the last update."""


def make_backend(name, **kwargs):
    if name not in BACKENDS:
        raise ValueError(f"backend must be one of {sorted(BACKENDS)}, got {name!r}")
    module, cls = BACKENDS[name]
    return getattr(importlib.import_module(module), cls)(**kwargs)


def compose(a, b):
    """SE(2) a * b for (x, y, theta) tuples; theta comes out as atan2, like gtsam Pose2."""
    x1, y1, t1 = a
    x2, y2, t2 = b
    c1, s1 = math.cos(t1), math.sin(t1)
    c2, s2 = math.cos(t2), math.sin(t2)
    return (x1 + c1 * x2 - s1 * y2, y1 + s1 * x2 + c1 * y2, math.atan2(s1 * c2 + c1 * s2, c1 * c2 - s1 * s2))


def transform_from(pose, p):
    """Point p (x, y) in the pose's frame -> world (x, y)."""
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return (x + c * p[0] - s * p[1], y + s * p[0] + c * p[1])
