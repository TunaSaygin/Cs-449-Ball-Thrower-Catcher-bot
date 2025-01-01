import numpy as np

class KalmanFilter:
    def __init__(self, dim=3, dt=0.01, process_noise=1e-3, measurement_noise=1e-2):
        """
        Initialize the Kalman filter.

        Args:
            dim (int): Dimensionality of the state (e.g., 3 for 3D positions).
            dt (float): Time step.
            process_noise (float): Process noise covariance.
            measurement_noise (float): Measurement noise covariance.
        """
        self.dt = dt
        self.dim = dim

        # State vector [position, velocity]
        self.x = np.zeros(dim * 2)

        # State covariance matrix
        self.P = np.eye(dim * 2) * 1e-1

        # State transition matrix
        self.F = np.eye(dim * 2)
        for i in range(dim):
            self.F[i, dim + i] = dt

        # Process noise covariance matrix
        self.Q = np.eye(dim * 2) * process_noise

        # Measurement matrix
        self.H = np.zeros((dim, dim * 2))
        for i in range(dim):
            self.H[i, i] = 1

        # Measurement noise covariance matrix
        self.R = np.eye(dim) * measurement_noise

        # Identity matrix
        self.I = np.eye(dim * 2)

    def update(self, measurement):
        """
        Update the filter with a new measurement.

        Args:
            measurement (array-like): Observed position [x, y, z].
        """
        z = np.array(measurement)
        y = z - self.H @ self.x  # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R  # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        # Update state estimate and covariance matrix
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P

    def predict(self):
        """
        Predict the next state.

        Returns:
            np.array: Predicted position [x, y, z].
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:self.dim]  # Return predicted position
