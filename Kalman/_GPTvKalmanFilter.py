import numpy as np

class KalmanFilter:
    def __init__(self, F, H, Q, R, P, x0):
        """
        Initialize the Kalman Filter
        :param F: State Transition matrix
        :param H: Measurement matrix
        :param Q: Process Noise Covariance
        :param R: Measurement Noise Covariance
        :param P: Initial estimation error covariance
        :param x0: Initial state estimate
        """
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x0

    def predict(self):
        """
        Predict the next state
        """
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        """
        Update the state estimate using measurement z
        :param z: Measurement
        """
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    def filter(self, measurements):
        """
        Apply Kalman Filter over a series of measurements
        :param measurements: Series of measurements
        :return: List of state estimates
        """
        estimates = []
        for z in measurements:
            self.predict()
            self.update(z)
            estimates.append(self.x)
        return estimates

if __name__ == "__main__":
    F = np.array([[1, 1], [0, 1]])
    H = np.array([[1, 0]])
    Q = np.array([[0.1, 0], [0, 0.1]])
    R = np.array([[1]])
    P = np.array([[1, 0], [0, 1]])
    x0 = np.array([0, 0])

    kalman = KalmanFilter(F, H, Q, R, P, x0)

    measurements = [np.array([z]) for z in [1, 1.1, 2.9, 4.2, 5.1]]
    estimates = kalman.filter(measurements)
    print(estimates)
