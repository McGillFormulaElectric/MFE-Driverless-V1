# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Credits: https://github.com/carlos-argueta/rse_prob_robotics/blob/main/rse_gaussian_filters/rse_gaussian_filters/filters/ekf.py

import numpy as np

import time

class ExtendedKalmanFilter:

	def __init__(self, initial_state, initial_covariance, motion_model, observation_model, **kwargs) -> None :
		"""
		Constructor for the Extended Kalman Filter class. 
		
		The Extended Kalman Filter is the
		generalized version of the Kalman Filter for non-linear systems. It utilizes a Jacobian 
		matrix to linear the state space model at every given moment, and applied a Gaussian
		covariance matrix to probabilistically estimate a state vector repeatedly.

		Args:	
			np.ndarray (initial_state): The starting state vector
			np.ndarray (initial_covariance): The starting covariance matrix for the state vector
			np.ndarray  (motion_model): A list of three matrices in array form of the kinematic 
			motion model.
			# TODO: Add documentation for observation_model
			Optional<List>, Optional<List> (**kwargs): Process noise (noise of the motion model), 
			Observation Noise (noise of the measurement)

		Returns:
			None
		"""
		
		# Process arguments
		proc_noise_std = kwargs.get('proc_noise_std', [0.02, 0.02, 0.01])
		obs_noise_std = kwargs.get('obs_noise_std', [0.02, 0.02, 0.01])

		self.mu = initial_state # Initial state estimate 
		self.Sigma = initial_covariance # Initial uncertainty

		self.g, self.G, self.V = motion_model() # The action model to use.
		
		# Standard deviations of the process or action model noise
		self.proc_noise_std = np.array(proc_noise_std)
		# Process noise covariance (R)
		self.R = np.diag(self.proc_noise_std ** 2)

		self.h, self.H = observation_model() # The observation model to use

		# Standard deviations for the observation or sensor model noise
		self.obs_noise_std = np.array(obs_noise_std)
		# Observation noise covariance (Q)
		self.Q = np.diag(self.obs_noise_std ** 2)

		self.exec_times_pred = []
		self.exec_times_upd = []

		
	def predict(self, u, dt):
		"""
		Executes the prediction step of the Extended Kalman Filter
		"""
		start_time = time.time()
		# Predict state estimate (mu) 
		self.mu = self.g(self.mu, u, dt)
		# Predict covariance (Sigma)
		self.Sigma = self.G(self.mu, u, dt) @ self.Sigma @ self.G(self.mu, u, dt).T + self.R 

		end_time = time.time()
		execution_time = end_time - start_time
		self.exec_times_pred.append(execution_time)
		print(f"Execution time prediction: {execution_time} seconds")

		print("Average exec time pred: ", sum(self.exec_times_pred) / len(self.exec_times_pred))


		return self.mu, self.Sigma

	def update(self, z, dt):
		"""
		Updates the Extended Kalman filter with the inputted measurement using the correction step
		"""
		start_time = time.time()

		# Compute the Kalman gain (K)
		K = self.Sigma @ self.H(self.mu).T @ np.linalg.inv(self.H(self.mu) @ self.Sigma @ self.H(self.mu).T + self.Q)
		
		# Update state estimate (mu) 
		innovation = z - self.h(self.mu)
		self.mu = self.mu + (K @ innovation).reshape((self.mu.shape[0],))

		# Update covariance (Sigma)
		I = np.eye(len(K))
		self.Sigma = (I - K @ self.H(self.mu)) @ self.Sigma

		end_time = time.time()
		execution_time = end_time - start_time
		self.exec_times_upd.append(execution_time)
		print(f"Execution time update: {execution_time} seconds")

		print("Average exec time update: ", sum(self.exec_times_upd) / len(self.exec_times_upd))

		return self.mu, self.Sigma