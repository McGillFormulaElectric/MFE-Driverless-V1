"""
Unit tests for the ExtendedKalmanFilter base class.

Covers the two bugs fixed in neil/feature/ekf-sigma-fix:
  1. G Jacobian must be evaluated at prior mu, not post-prediction mu.
  2. Covariance update must use the Joseph form (I-KH)S(I-KH)^T + KQK^T.

No ROS imports — runs with plain `pytest` or `colcon test`.
"""

import math
import numpy as np
import pytest
import sys
import os

# Allow PYTHONPATH to control which source tree is used (e.g. a specific worktree).
# Only fall back to the local package if nothing is already on sys.path for this module.
# PYTHONPATH takes priority (lets you test a specific worktree).
# Fall back to the package root next to this test directory.
# Append (not prepend) so PYTHONPATH takes priority when testing a specific worktree.
# e.g. PYTHONPATH=/path/to/MFE-V1-p2-ekf-fix/ros2/src/mfe_state_estimation pytest ...
_pkg_root = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
if _pkg_root not in sys.path:
    sys.path.append(_pkg_root)

from mfe_state_estimation.filters.extended_kalman_filter import ExtendedKalmanFilter


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _make_identity_models(n=3):
    """Identity motion + observation models for basic sanity checks."""
    def motion():
        g  = lambda mu, u, dt: mu.copy()
        G  = lambda mu, u, dt: np.eye(n)
        V  = None
        return g, G, V

    def observation():
        h = lambda mu: mu[:2]
        H = lambda mu: np.eye(2, n)
        return h, H

    return motion, observation


def _make_unicycle_models():
    """
    Simplified unicycle motion model: [x, y, theta].
    G has off-diagonal terms — exercises the prior-mu bug path.
    """
    def motion():
        def g(mu, u, dt):
            v, w = u
            theta = mu[2]
            return np.array([
                mu[0] + v * math.cos(theta) * dt,
                mu[1] + v * math.sin(theta) * dt,
                mu[2] + w * dt,
            ])

        def G(mu, u, dt):
            v, _ = u
            theta = mu[2]
            return np.array([
                [1, 0, -v * math.sin(theta) * dt],
                [0, 1,  v * math.cos(theta) * dt],
                [0, 0,  1],
            ])

        return g, G, None

    def observation():
        h = lambda mu: mu[:2]
        H = lambda mu: np.eye(2, 3)
        return h, H

    return motion, observation


def _make_ekf(motion, observation, n=3, proc_std=None, obs_std=None):
    proc_std = proc_std or [0.1] * n
    obs_std  = obs_std  or [0.1, 0.1]
    return ExtendedKalmanFilter(
        np.zeros(n), np.eye(n),
        motion, observation,
        proc_noise_std=proc_std,
        obs_noise_std=obs_std,
    )


# ---------------------------------------------------------------------------
# Test group 1: covariance stays positive semi-definite
# ---------------------------------------------------------------------------

class TestCovariancePSD:

    def test_identity_model_stays_psd(self):
        """Sigma must stay PSD through 100 predict+update cycles."""
        motion, observation = _make_identity_models()
        ekf = _make_ekf(motion, observation)
        for _ in range(100):
            ekf.predict(np.zeros(2), 0.1)
            ekf.update(np.array([0.05, -0.05]), 0.1)
            eigvals = np.linalg.eigvalsh(ekf.Sigma)
            assert eigvals.min() > -1e-9, \
                f"Sigma went non-PSD: min eigenvalue = {eigvals.min():.2e}"

    def test_unicycle_model_stays_psd(self):
        """Non-trivial off-diagonal Jacobian — covariance must stay PSD."""
        motion, observation = _make_unicycle_models()
        ekf = _make_ekf(motion, observation)
        u = np.array([1.0, 0.1])  # forward speed + slight turn
        for i in range(200):
            ekf.predict(u, 0.05)
            if i % 5 == 0:
                z = ekf.mu[:2] + np.random.randn(2) * 0.05
                ekf.update(z, 0.05)
            eigvals = np.linalg.eigvalsh(ekf.Sigma)
            assert eigvals.min() > -1e-9, \
                f"Step {i}: Sigma non-PSD, min eig = {eigvals.min():.2e}"

    def test_sigma_stays_symmetric(self):
        """Sigma must remain symmetric after Joseph-form update."""
        motion, observation = _make_unicycle_models()
        ekf = _make_ekf(motion, observation)
        for _ in range(50):
            ekf.predict(np.array([2.0, 0.3]), 0.05)
            ekf.update(ekf.mu[:2] + 0.1, 0.05)
        sym_err = np.max(np.abs(ekf.Sigma - ekf.Sigma.T))
        assert sym_err < 1e-12, f"Sigma not symmetric: max asymmetry = {sym_err:.2e}"


# ---------------------------------------------------------------------------
# Test group 2: state estimate converges
# ---------------------------------------------------------------------------

class TestConvergence:

    def test_stationary_position_converges(self):
        """
        Stationary car observed 200 times — estimated position should
        converge to within 0.05 m of truth.
        """
        motion, observation = _make_identity_models()
        ekf = _make_ekf(motion, observation, proc_std=[0.01, 0.01, 0.001])
        true_pos = np.array([3.0, -1.5])
        for _ in range(200):
            ekf.predict(np.zeros(2), 0.1)
            ekf.update(true_pos + np.random.randn(2) * 0.1, 0.1)
        err = np.linalg.norm(ekf.mu[:2] - true_pos)
        assert err < 0.05, f"State did not converge: error = {err:.3f} m"

    def test_covariance_shrinks_with_observations(self):
        """Uncertainty must decrease as more GPS measurements arrive."""
        motion, observation = _make_identity_models()
        ekf = _make_ekf(motion, observation)
        initial_trace = np.trace(ekf.Sigma)
        for _ in range(30):
            ekf.predict(np.zeros(2), 0.1)
            ekf.update(np.array([0.0, 0.0]), 0.1)
        assert np.trace(ekf.Sigma) < initial_trace, \
            "Covariance did not decrease with observations"


# ---------------------------------------------------------------------------
# Test group 3: prior-mu bug regression
# ---------------------------------------------------------------------------

class TestPriorMuJacobian:

    def test_jacobian_evaluated_at_prior(self):
        """
        With a velocity-dependent Jacobian, evaluating G at post-prediction mu
        would produce a different covariance matrix than evaluating at prior mu.
        This test verifies the implementation uses the prior.

        We capture G(prior) and G(post) and verify the implemented Sigma
        is consistent with G(prior), not G(post).
        """
        motion, observation = _make_unicycle_models()
        g, G_fn, _ = motion()
        h_fn, H_fn = observation()

        mu0   = np.array([0.0, 0.0, 0.0])
        Sigma0 = np.eye(3) * 0.5
        u  = np.array([2.0, 0.5])
        dt = 0.1

        # Reference: what the CORRECT update should produce
        G_prior = G_fn(mu0, u, dt)
        mu_pred = g(mu0, u, dt)
        R = np.diag(np.array([0.1, 0.1, 0.01]) ** 2)
        Sigma_correct = G_prior @ Sigma0 @ G_prior.T + R

        # What a buggy implementation (G at post-prediction mu) would produce
        G_post  = G_fn(mu_pred, u, dt)
        Sigma_buggy = G_post @ Sigma0 @ G_post.T + R

        # The EKF should match Sigma_correct, not Sigma_buggy
        motion_model_fn, obs_model_fn = _make_unicycle_models(), observation
        ekf = ExtendedKalmanFilter(
            mu0, Sigma0,
            _make_unicycle_models()[0], observation,
            proc_noise_std=[0.1, 0.1, 0.01],
            obs_noise_std=[0.1, 0.1],
        )
        ekf.predict(u, dt)

        err_correct = np.max(np.abs(ekf.Sigma - Sigma_correct))
        err_buggy   = np.max(np.abs(ekf.Sigma - Sigma_buggy))

        # Sigma should be closer to the correct version
        assert err_correct < err_buggy or err_correct < 1e-10, (
            f"EKF may be using post-prediction mu for Jacobian. "
            f"err_correct={err_correct:.2e}  err_buggy={err_buggy:.2e}"
        )


# ---------------------------------------------------------------------------
# Test group 4: no stdout spam
# ---------------------------------------------------------------------------

class TestNoStdoutSpam:

    def test_predict_produces_no_stdout(self, capsys):
        """predict() must not print anything (removed 100Hz print calls)."""
        motion, observation = _make_identity_models()
        ekf = _make_ekf(motion, observation)
        ekf.predict(np.zeros(2), 0.1)
        captured = capsys.readouterr()
        assert captured.out == '', f"predict() printed: {captured.out!r}"

    def test_update_produces_no_stdout(self, capsys):
        """update() must not print anything."""
        motion, observation = _make_identity_models()
        ekf = _make_ekf(motion, observation)
        ekf.predict(np.zeros(2), 0.1)
        ekf.update(np.array([0.0, 0.0]), 0.1)
        captured = capsys.readouterr()
        assert captured.out == '', f"update() printed: {captured.out!r}"
