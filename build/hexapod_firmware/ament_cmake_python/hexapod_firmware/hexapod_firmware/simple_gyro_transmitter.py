#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3
import serial 


class SimpleGyroTransmitter(Node):

    def __init__(self):
        super().__init__("simple_gyro_transmitter")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200) 

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.port_ = self.get_parameter("port").value 
        self.baudrate_ = self.get_parameter("baudrate").value 
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        self.sub_ = self.create_subscription(Vector3, "orientation", self.msgCallback, qos_profile)
        self.sub_


    def msgCallback(self, msg):
        # Format the Vector3 message data as a string and transmit it
        data = f"{msg.x},{msg.y},{msg.z}\n"
        self.arduino_.write(data.encode("utf-8"))

def main():
    rclpy.init()

    simple_gyro_transmitter = SimpleGyroTransmitter()
    rclpy.spin(simple_gyro_transmitter)
    
    simple_gyro_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
