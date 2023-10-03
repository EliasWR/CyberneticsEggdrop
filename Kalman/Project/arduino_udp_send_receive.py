import time
from socket import *
import numpy as np
import csv

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress=True)

arduino_ip = '192.168.10.240'
arduino_port = 8888


def arduino_send_receive(estimate):
    udp_socket.sendto(str(estimate).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an a values
        # [accel_x, accel_y, accel_z, range_senrray with the followingsor]
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
    with open('SensorDataKalman30CM.csv', 'w', newline='') as f:
        writer = csv.writer(f)

        # Write the header row
        writer.writerow(['AccX', 'AccY', 'AccZ', 'Dist'])

        estimate = 0.0
        delta = 1.0
        while (True):
            estimate = estimate + delta
            if (estimate > 100.0):
                delta = -1
            elif (estimate < -100):
                delta = 1

            sensor_values = arduino_send_receive(estimate)

            if (sensor_values is not None):
                x, y, z, d = sensor_values_as_struct(sensor_values)
                writer.writerow(sensor_values)
                print(sensor_values)

            else:
                arduino_has_been_reset()

def printSensorData ():
    estimate = 0.0
    delta = 1.0
    while (True):
        estimate = estimate + delta
        if (estimate > 100.0):
            delta = -1
        elif (estimate < -100):
            delta = 1

        sensor_values = arduino_send_receive(estimate)
        print(sensor_values)

#printSensorData()
writeSensorDataToFile()