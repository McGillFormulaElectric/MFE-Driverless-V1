import numpy as np

from mfe_state_estimation.extended_kalman_filter_node import (
    _DEFAULT_PROC_NOISE,
    _build_8state_motion_model,
    _H_wheelspeed,
    process_noise_for_dt,
)
from mfe_state_estimation.filters.extended_kalman_filter import ExtendedKalmanFilter

SPEED = 3
WHEELSPEED_VAR = 0.0025


def _make_filter():
    return ExtendedKalmanFilter(
        np.zeros(8),
        np.diag([5.0, 5.0, 0.1, 1.0, 0.1, 1.0, 1.0, 0.01]),
        _build_8state_motion_model,
        lambda: (None, None),
        proc_noise_std=_DEFAULT_PROC_NOISE,
        obs_noise_std=[1.0, 1.0],
    )


def test_accel_variance_only_adds_to_speed_process_noise():
    base_R = np.diag(np.array(_DEFAULT_PROC_NOISE) ** 2)

    Q = process_noise_for_dt(base_R, 0.04, 0.1)

    expected = base_R.copy()
    expected[SPEED, SPEED] += 0.04 * 0.1**2
    np.testing.assert_allclose(Q, expected)
    np.testing.assert_allclose(base_R, np.diag(np.array(_DEFAULT_PROC_NOISE) ** 2))


def test_larger_accel_variance_grows_predicted_speed_covariance():
    dt = 0.1
    speed_var = []
    for var_acc in (0.01, 1.0):
        ekf = _make_filter()
        ekf.predict(np.zeros(2), dt, process_noise_for_dt(ekf.R, var_acc, dt))
        speed_var.append(ekf.Sigma[SPEED, SPEED])

    assert speed_var[1] > speed_var[0]
    np.testing.assert_allclose(speed_var[1] - speed_var[0], (1.0 - 0.01) * dt**2)


def test_larger_accel_variance_increases_wheelspeed_gain():
    # Speed is observed directly by wheelspeed, so K[speed] = P_vv / (P_vv + R).
    # A larger prior speed variance therefore gives a larger gain, not a smaller one.
    dt = 0.1
    H = _H_wheelspeed()
    R_ws = np.array([[WHEELSPEED_VAR]])
    gains = []
    for var_acc in (0.01, 1.0):
        ekf = _make_filter()
        ekf.predict(np.zeros(2), dt, process_noise_for_dt(ekf.R, var_acc, dt))
        K = ekf.Sigma @ H.T @ np.linalg.inv(H @ ekf.Sigma @ H.T + R_ws)
        gains.append(K[SPEED, 0])

    assert gains[1] > gains[0]
