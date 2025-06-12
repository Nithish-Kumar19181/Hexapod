#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial 


class SimpleSerialTransmitter(Node):

    def __init__(self):
        super().__init__("simple_serial_transmitter")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200) 

        self.port_ = self.get_parameter("port").value 
        self.baudrate_ = self.get_parameter("baudrate").value 
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        self.sub_ = self.create_subscription(Vector3, "joystick_direction", self.msgCallback, 10)
        self.sub_

    def msgCallback(self, msg):
        # Format the Vector3 message data as a string and transmit it
        data = f"{msg.x},{msg.y},{msg.z}\n"
        self.arduino_.write(data.encode("utf-8"))

def main():
    rclpy.init()

    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
