"""GTSAM backend: cone-landmark pose graph solved incrementally with iSAM2."""
import gtsam
import numpy as np
from mfe_validation.slam_backend import SlamBackend

X, L = gtsam.symbol_shorthand.X, gtsam.symbol_shorthand.L


def _pose2(p):
    return gtsam.Pose2(float(p[0]), float(p[1]), float(p[2]))


class GtsamBackend(SlamBackend):
    def __init__(self, obs_sigmas):
        # Huber-robust bearing-range noise: a bad LiDAR association cannot drag the whole graph.
        self._obs_noise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),
                                                         gtsam.noiseModel.Diagonal.Sigmas(np.array(obs_sigmas)))
        self._isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self._graph, self._init = gtsam.NonlinearFactorGraph(), gtsam.Values()
        self._est = None

    def add_pose_prior(self, i, pose, sigmas):
        self._graph.add(gtsam.PriorFactorPose2(X(i), _pose2(pose), gtsam.noiseModel.Diagonal.Sigmas(np.array(sigmas))))

    def add_odometry(self, i, j, dx, dy, dtheta, sigmas):
        self._graph.add(gtsam.BetweenFactorPose2(X(i), X(j), _pose2((dx, dy, dtheta)),
                                                 gtsam.noiseModel.Diagonal.Sigmas(np.array(sigmas))))

    def add_landmark_observation(self, pose_index, landmark_id, bearing_rad, range_m):
        self._graph.add(gtsam.BearingRangeFactor2D(X(pose_index), L(landmark_id), gtsam.Rot2.fromAngle(bearing_rad),
                                                   range_m, self._obs_noise))

    def init_pose(self, i, pose):
        self._init.insert(X(i), _pose2(pose))

    def init_landmark(self, landmark_id, xy):
        self._init.insert(L(landmark_id), gtsam.Point2(float(xy[0]), float(xy[1])))

    def update(self):
        self._isam.update(self._graph, self._init)
        self._graph, self._init = gtsam.NonlinearFactorGraph(), gtsam.Values()
        self._est = self._isam.calculateEstimate()

    def pose(self, i):
        p = self._est.atPose2(X(i))
        return (p.x(), p.y(), p.theta())

    def landmark(self, landmark_id):
        q = self._est.atPoint2(L(landmark_id))
        return (q[0], q[1])
