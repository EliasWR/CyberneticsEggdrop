import time
from socket import *
import numpy as np

ARDUINO_IP = '192.168.10.240'
ARDUINO_PORT = 8888


class UDP_communication:

    def __init__(self, ip = ARDUINO_IP, port = ARDUINO_PORT):
        self.udp_socket = socket(AF_INET, SOCK_DGRAM)
        self.udp_socket.settimeout(1)

        self.arduino_ip = ip
        self.arduino_port = port

    def send_values(self, values=None):
        if values is None:
            values = []
        # Convert all elements to strings before joining them
        # string_values = ','.join(str(value) for value in values)
        msg = float(values[0][0])
        msg = format(msg, '.7f')
        print (msg)
        msg = str(msg)
        self.send(msg)


    def send(self, string):
        self.udp_socket.sendto(str(string).encode(), (self.arduino_ip, self.arduino_port))
        
        
    def receive(self):
        try:
            inbound_message, remote_address = self.udp_socket.recvfrom(24)
            # returns an a values
            # [accel_x, accel_y, accel_z, range_senrray with the followingsor]
            return np.array(inbound_message.decode('ascii').split(',')).astype(float)
        except Exception as e:
            print(e)
            return None

