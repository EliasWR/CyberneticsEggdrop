# VERY MUCH WORK IN PROGRESS


import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter:

    def __init__(self, F, H, R, Q, P0, x0):
        """
        Initialize the Kalman Filter
        :param F: State Transition matrix (A)
        :param H: Observation model (B)
        :param R: Observation Noise Covariance
        :param Q: Process Noise Covariance
        :param P0: Initial estimation error covariance
        :param X0: Initial state estimate
        """
        self.F = F
        self.H = H
        self.R = R
        self.Q = Q
        self.P = P0
        self.x = x0

    def predict(self):
        """
        Predictions based on the model ("A priori")
        """
        print("x shape pred1: ", self.x.shape)
        print("F shape pred1: ", self.F.shape)
        print("H shape pred1: ", self.H.shape)
        print("x", self.x)
        print("F", self.F)
        self.x = self.F @ self.x + self.H
        print("x", self.x)
        print("x shape pred2: ", self.x.shape)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        """
        Corrections based on the measurements ("A posteriori")
        """
        K = (self.P @ self.H.T) @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        estimate = self.H @ self.x
        print("x shape upd1: ", self.x.shape)
        self.x = self.x + K @ (measurement - estimate)
        self.P = self.P - K @ self.H @ self.P
        print("x shape upd2: ", self.x.shape)
        

    def filter(self, measurements):
        """
        Apply Kalman Filter over a series of measurements
        """
        filtered = np.zeros((len(measurements), 2))
        
        for i in range(filtered.shape[0]):
            self.predict()
            self.update(measurements[i])
            print(filtered[i].shape)
            print("x shape: ", self.x.shape)
            filtered[i] = self.x
        return filtered

    

if __name__ == "__main__":
    F = np.array([[1, 1], [0, 1]])
    H = np.array([[1, 0]])
    Q = np.array([[0.1, 0], [0, 0.1]])
    R = np.array([[1]])
    P = np.array([[1, 0], [0, 1]])
    x0 = np.array([0,0])

    kalman = KalmanFilter(F, H, Q, R, P, x0)

    measurements = [np.array([z]) for z in [1, 1.1, 2.9, 4.2, 5.1, 8.6, 5.2, 4.8 , 4.9, 5.1, 0.2, 5.2]]
    estimates = kalman.filter(measurements)

    # Plot the estimates over 10 cycles of measurements
    
    plt.plot([e[0] for e in estimates])
    plt.plot([m for m in measurements])
    plt.show()


    print(estimates)
        


    