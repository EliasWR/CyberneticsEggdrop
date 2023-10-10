import time
from socket import *
import numpy as np
import csv

BiasZNeg = 1.066219512  # =AVERAGE(C2:C247)
BiasZPos = -0.934452174 # =AVERAGE(C286:C860)
Bias10CM = 1.2676471    # 101.2676471  # =AVERAGE(D2:D860)
Bias20CM = 5.5929368    # 205.5929368  # =AVERAGE(D2:D860)
Bias30CM = 7.2714617    # 307.2714617  # =AVERAGE(D2:D860)

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress=True)

arduino_ip = '192.168.10.240'
arduino_port = 8888


def arduino_send_receive(estimate):
    udp_socket.sendto(str(estimate).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        return np.array(inbound_message.decode('ascii').split(',')).astype(float)
    except Exception as e:
        print(e)


def use_sensor_values_for_something(sensor_values):
    print(sensor_values)


def sensor_values_as_struct(sensor_values):
    return {
        'x': sensor_values[0],
        'y': sensor_values[1],
        'z': sensor_values[2],
        'd': sensor_values[3]
    }


def arduino_has_been_reset():
    print("Arduino is offline.. Resetting")

def writeSensorDataToFile ():
    with open('SensorDataKalmanTimestampedOscillating.csv', 'w', newline='') as f:
        writer = csv.writer(f)

        # Write the header row
        writer.writerow(['AccX', 'AccY', 'AccZ', 'Dist', 'Time'])

        timeStart = time.time()
        estimate = 0.0
        delta = 1.0
        while (True):
            # Kalman filter data
            estimate = estimate + delta
            if (estimate > 100.0):
                delta = -1
            elif (estimate < -100):
                delta = 1
            # Kalman filter data
            sensor_values = arduino_send_receive(estimate)

            if (sensor_values is not None):
                # x, y, z, d = sensor_values_as_struct(sensor_values)
                currentTime = time.time() - timeStart
                sensor_values = arduino_send_receive(estimate)
                sensor_values = np.append(sensor_values, currentTime)
                writer.writerow(sensor_values)
                print(sensor_values)

            else:
                arduino_has_been_reset()

def printSensorData ():
    estimate = 0.0
    delta = 1.0
    timeStart = time.time()
    while (True):
        estimate = estimate + delta
        if (estimate > 100.0):
            delta = -1
        elif (estimate < -100):
            delta = 1

        currentTime = time.time()-timeStart
        sensor_values = arduino_send_receive(estimate)
        sensor_values = np.append(sensor_values, currentTime)
        print(sensor_values)

# printSensorData()
writeSensorDataToFile()