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

    def predict(self):
        """
        Predict the next state
        """

    def update(self, z):
        """
        Update the state estimate using measurement z
        :param z: Measurement
        """

    def filter(self, measurements):
        """
        Apply Kalman Filter over a series of measurements
        :param measurements: Series of measurements
        :return: List of state estimates
        """
