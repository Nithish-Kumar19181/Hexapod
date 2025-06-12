#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
import serial


class InfoTransmitter(Node):

    def __init__(self):
        super().__init__('info_transmitter')

        # Serial communication setup
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value

        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=0.1)

        # Initialize placeholders for data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.joystick_data = [0.0, 0.0, 0.0]
        self.orientation_data = [0.0, 0.0, 0.0]

        # Subscribers
        self.create_subscription(Vector3, 'joystick_direction', self.joystick_callback, 10)
        self.create_subscription(Vector3, 'orientation', self.orientation_callback, qos_profile)

        # Publisher (optional for ROS communication)
        self.info_publisher = self.create_publisher(Float64MultiArray, 'info_transmitter', 10)

        # Timer to publish data
        self.create_timer(0.1, self.publish_and_transmit_data)  # Publish at 10 Hz

    def joystick_callback(self, msg):
        """Callback to update joystick direction data."""
        self.joystick_data = [msg.x, msg.y, msg.z]

    def orientation_callback(self, msg):
        """Callback to update orientation data."""
        self.orientation_data = [msg.x, msg.y, msg.z]

    def publish_and_transmit_data(self):
        """Combine data, publish to ROS, and transmit serially."""
        # Combine joystick and orientation data
        combined_data = self.joystick_data + self.orientation_data
        
        # Publish data to ROS topic
        msg = Float64MultiArray(data=combined_data)
        self.info_publisher.publish(msg)

        # Serialize the combined data as a comma-separated string
        data_string = ','.join(f"{value:.2f}" for value in combined_data) + '\n'

        # Transmit serially to ESP32
        self.serial_connection.write(data_string.encode('utf-8'))
        self.get_logger().info(f"Published to ROS and transmitted: {data_string.strip()}")


def main():
    rclpy.init()
    node = InfoTransmitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial connection on shutdown
        if node.serial_connection.is_open:
            node.serial_connection.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
