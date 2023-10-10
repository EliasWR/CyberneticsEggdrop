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
        self.sigma = sigma

        self.A = A
        self.B = B
        self.H = H
        self.R = R
        self.Q = Q
        self.P = P0
        self.x = x0

        

        self.prev_time = time.time()

    def predict(self):
        """
        Predictions based on the model ("A priori")
        """
        delta_t = time.time() - self.prev_time
        self.prev_time = time.time()
        self._calculate_A(delta_t)
        self._calculate_Q(delta_t)
        
        control = self.B @ self.u if (self.B is not None and self.u is not None) else 0
        self.x = self.A @ self.x + control              # x- = Ax + Bu
        self.P = self.A @ self.P @ self.A.T + self.Q    # P- = AP_ A.T + Q
      

    def correct(self, measurement):
        """
        Corrections based on the measurements ("A posteriori")
        """
        K = (self.P @ self.H.T) @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        estimate = self.H @ self.x

        self.x = self.x + K @ (measurement - estimate)
        self.P = self.P - K @ self.H @ self.P


    def update(self, measurement):

        self.predict()              # A priori
        self.correct(measurement)   # A posteriori


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
    

def load_measurements_from_file(filename):
    """
    Load measurements from a file
    Returned as list 
    """
    measurements = []
    with open(filename, "r") as f:
        
        f.readline()
        for line in f:
            arr = line.strip()
            arr = arr.split(",")
            measurements.append(arr)

    measurements = np.array(measurements, dtype=np.float64)
    return measurements


def generate_random_samples(n = 100):
    """
    Generate n number of random samples for testing
    """
    z = (np.random.randn(n) + 1) * 5
    samples = np.zeros((n, 1))
    for i in range(n):
        samples[i] = np.array([z[i]])
    return samples


def testing():
    # m = 3 (states), n = 2 (measurements)
    A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) # State transition matrix (m x m)
    H = np.array([[1, 0, 0],
                  [0, 0, 1]])         # Observation model (n x m)

    R = np.array([[1, 0],
                  [0, 1]])            # Observation Noise Covariance (n x n)
    Q = np.array([[0.1, 0, 0], [0, 0, 0], [0, 0, 0.001]])     # Process Noise Covariance (m x m)
    P0 = np.array([[1, 0, 0], [0, 0, 0], [0, 0, 1]]) # Initial estimation error covariance (m x m)
    x0 = np.array([[0], [0], [0]])       # Initial state estimate (m x 1)
                                         # measurement (n x 1)


    kalman = KalmanFilter(A, H, R, Q, P0, x0)

    # Load measurements from file
    measurements = load_measurements_from_file("Project\\SensorDataKalmanTimestamped.csv")
    accX, accY, accZ, dist = measurements[:, 0], measurements[:, 1], measurements[:, 2], measurements[:, 3]

    # Filter the measurements treating each measurement as a single sample
    zero = np.zeros((len(accZ), ))
    use_measurements = np.array([[accZ], [dist]]).T


    estimates = []
    for m in use_measurements:
        est = kalman.filter_single(m.T)
        estimates.append(est)
        #time.sleep(1) # Simulate measurements coming in

    # Extract the estimates
    estimates = np.array(estimates)
    accZ_hat = estimates[:, 0]
    vel_hat = estimates[:, 1]
    dist_hat = estimates[:, 2]

    print("accZ_hat: ", accZ_hat.shape)
    print("vel_hat: ", vel_hat.shape)
    print("dist_hat: ", dist_hat.shape)


    # Plot the the acceleration, velocity and distance in the three different plots
    plt.figure(1)
    plt.title("Acceleration")
    plt.plot(accZ, label="Actual")
    plt.plot(accZ_hat, label="Estimation")
    plt.legend()

    plt.figure(2)
    plt.title("Velocity")
    plt.plot(vel_hat, label="Estimation")
    plt.legend()
    
    plt.figure(3)
    plt.title("Distance")
    plt.plot(dist, label="Actual")
    plt.plot(dist_hat, label="Estimation")
    plt.legend()

    plt.show()



if __name__ == "__main__":
    testing()




    