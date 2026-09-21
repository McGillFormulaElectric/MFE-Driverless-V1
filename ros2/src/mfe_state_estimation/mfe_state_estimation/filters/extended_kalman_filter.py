# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Credits: https://github.com/carlos-argueta/rse_prob_robotics/blob/main/rse_gaussian_filters/rse_gaussian_filters/filters/ekf.py

import numpy as np

class ExtendedKalmanFilter:

	def __init__(self, initial_state, initial_covariance, motion_model, observation_model, **kwargs) -> None :
		"""Non-linear state estimation via iterative linearization (Jacobian-based)."""

		proc_noise_std = kwargs.get('proc_noise_std', [0.02, 0.02, 0.01])
		obs_noise_std = kwargs.get('obs_noise_std', [0.02, 0.02, 0.01])

		self.mu = initial_state
		self.Sigma = initial_covariance

		self.g, self.G, self.V = motion_model()
		self.proc_noise_std = np.array(proc_noise_std)
		self.R = np.diag(self.proc_noise_std ** 2)

		self.h, self.H = observation_model()
		self.obs_noise_std = np.array(obs_noise_std)
		self.Q = np.diag(self.obs_noise_std ** 2)


		
	def predict(self, u, dt):
		"""Prediction step: propagate state and covariance via motion model."""
		self.mu = self.g(self.mu, u, dt)
		self.Sigma = self.G(self.mu, u, dt) @ self.Sigma @ self.G(self.mu, u, dt).T + self.R
		return self.mu, self.Sigma

	def update(self, z, dt):
		"""Correction step: fuse measurement via Kalman gain."""
		K = self.Sigma @ self.H(self.mu).T @ np.linalg.inv(self.H(self.mu) @ self.Sigma @ self.H(self.mu).T + self.Q)
		innovation = z - self.h(self.mu)
		self.mu = self.mu + (K @ innovation).reshape((self.mu.shape[0],))
		self.Sigma = (np.eye(len(K)) - K @ self.H(self.mu)) @ self.Sigma
		return self.mu, self.Sigma