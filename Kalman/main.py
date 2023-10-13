from lib.kalman_filter import KalmanFilter
from lib.communication import UDP_communication

import numpy as np

def get_from_arduino():
    print("Getting from Arduino")
    return np.array([[50],[0.3]])

def send_to_arduino(values = []):
    print("Sending to Arduino")
    print(values)
    pass    
    
def setup_kalman():
    # m = 3 (states), n = 2 (measurements)
    # States: x = [dist, vel, accZ]
    # Measurements: z = [accZ, dist]
    
    A = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])       # State transition matrix (m x m)
    H = np.array([[0, 0, 1],
                  [1, 0, 0]])       # Measurement model (n x m)
    R = np.array([[0.1, 0],
                  [0, 0.08]])         # Measurement Noise Covariance (n x n)
    Q = np.array([[0.05, 0, 0],
                  [0, 1, 0],
                  [0, 0, 0.002]])      # Process Noise Covariance (m x m)
    P0 = np.array([[1, 0, 0],
                   [0, 0, 0],
                   [0, 0, 1]])      # Initial estimation error covariance (m x m)
    x0 = np.array([[25],
                   [0],
                   [1]])         # Initial state estimate (m x 1)
    # measurement (n x 1)

    kalman = KalmanFilter(A, H, R, Q, P0, x0)

    return kalman



def main():
    filter = setup_kalman()
    arduino = UDP_communication()

    try:
        arduino.send(1)
        while True:
            measurements = arduino.receive()
            if (measurements is not None):
                measurements = measurements.reshape(2,1)
                estimate = filter.filter_single(measurements)
            arduino.send_values(estimate)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()