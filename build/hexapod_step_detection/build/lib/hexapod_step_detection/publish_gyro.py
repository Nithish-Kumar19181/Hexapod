import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import serial  # For serial communication

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    """
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class OrientationNode(Node):
    def __init__(self):
        super().__init__('orientation_node')

        # Initialize serial communication
        # self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # Adjust the port and baudrate

        # QoS profile for best-effort communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscriber to the /imu/data topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile
        )

        # Publisher for the /orientation topic
        self.publisher = self.create_publisher(
            Vector3,
            '/orientation',
            qos_profile
        )

    def imu_callback(self, msg):
        # Extract quaternion from IMU message
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(x, y, z, w)

        # Create and publish Vector3 message with Euler angles
        orientation_msg = Vector3()
        orientation_msg.x = math.degrees(roll)
        orientation_msg.y = math.degrees(pitch)
        orientation_msg.z = math.degrees(yaw)

        self.publisher.publish(orientation_msg)

        # # Transmit orientation data over serial
        # data_to_send = f'{orientation_msg.x:.2f},{orientation_msg.y:.2f},{orientation_msg.z:.2f}\n'
        # self.serial_port.write(data_to_send.encode('utf-8'))

        # # Log the values for debugging
        # self.get_logger().info(f'Published Orientation: roll={math.degrees(roll):.2f}, pitch={math.degrees(pitch):.2f}, yaw={math.degrees(yaw):.2f}')
        # self.get_logger().info(f'Sent Data: {data_to_send.strip()}')

    def destroy_node(self):
        # Close the serial port when the node is destroyed
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
