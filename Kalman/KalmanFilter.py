# VERY MUCH WORK IN PROGRESS


import numpy as np
import matplotlib.pyplot as plt
import time

class KalmanFilter:

    def __init__(self, A, H, R, Q, P0, x0, B = None, u = None, sigma = 0.1):
        """
        Initialize the Kalman Filter
        :param F: State Transition matrix (A)
        :param H: Observation model (B)
        :param R: Observation Noise Covariance
        :param Q: Process Noise Covariance
        :param P0: Initial estimation error covariance
        :param X0: Initial state estimate
        """
        self.A = A
        self.B = B
        self.H = H
        self.R = R
        self.Q = Q
        self.P = P0
        self.x = x0

        self.sigma = sigma

        self.prev_time = time.time()

    def predict(self):
        """
        Predictions based on the model ("A priori")
        """
        delta_t = time.time() - self.prev_time
        self.prev_time = time.time()
        self._calculate_A(delta_t)
        self._calculate_Q(delta_t)
        
        control = self.B @ self.u if self.B is not None and self.u is not None else 0
        self.x = self.A @ self.x + control              # x- = Ax + Bu
        self.P = self.A @ self.P @ self.A.T + self.Q    # P- = AP_ A.T + Q
      


    def update(self, measurement):
        """
        Corrections based on the measurements ("A posteriori")
        """
        self.predict()

        K = (self.P @ self.H.T) @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        estimate = self.H @ self.x

        self.x = self.x + K @ (measurement - estimate)
        self.P = self.P - K @ self.H @ self.P

        

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

    def filter_single(self, measurement):
        """
        Apply Kalman Filter over a single measurement
        """
        
        self.update(measurement)
        return self.x
    

    def _calculate_A(self, delta_t):
        """
        Calculate the state transition matrix
        """
        A = np.array([  [1, delta_t, 0.5 * delta_t**2],
                        [0, 1, delta_t],
                        [0, 0, 1]] )

        self.A = A

    def _calculate_Q(self, delta_t):
        """
        Calculate the process noise covariance matrix
        """
        Q = np.array([  [1/36*delta_t**6, 1/12*delta_t**5, 1/6*delta_t**4],
                        [1/12*delta_t**5, 1/6*delta_t**4, 1/2*delta_t**3],
                        [1/6*delta_t**4, 1/2*delta_t**3, delta_t**2]] )

        self.Q = self.sigma**2 * Q  
    

if __name__ == "__main__":

    A = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
    H = np.array([[0, 0, 1]])
    Q = np.eye(3) # Adjust as needed
    R = np.array([[1]])
    P = np.eye(3)
    x0 = np.array([0, 0, 0])

    kalman = KalmanFilter(A, H, R, Q, P, x0, sigma=15)

    # Generate n number of perlin noise samples for testing
    n = 100
    z = (np.random.randn(n) + 1) * 5
    measurements = np.zeros((n, 1))
    for i in range(n):
        measurements[i] = np.array([z[i]])


    #measurements = [np.array([z]) for z in [1, 1.1, 2.9, 4.2, 5.1, 8.6, 5.2, 4.8 , 4.9, 5.1, 0.2, 5.2]]
    #estimates = [kalman.filter_single(m) for m in measurements]
    estimates = []
    for m in measurements:
        est = kalman.filter_single(m)
        estimates.append(est)
        print(m, est)
        #time.sleep(0.1)
    # Plot the estimates over 10 cycles of measurements
    
    plt.plot([e[2] for e in estimates])
    plt.plot([m for m in measurements])
    plt.show()


    