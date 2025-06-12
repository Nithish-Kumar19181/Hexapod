import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import LaserScan
from depthimage_to_laserscan.laserscan import DepthImageToLaserScan
from cv_bridge import CvBridge
import cv2

class DepthToLaserScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_laserscan_node')

        # Declare parameters
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('scan_topic', '/scan')                          # Remap the output scan topic
            

        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        # Create a CvBridge to convert ROS Image message to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the depth image and camera info
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # LaserScan publisher
        self.scan_publisher = self.create_publisher(LaserScan, self.scan_topic, 10)

        # Initialize DepthImageToLaserScan converter
        self.depth_to_laserscan = DepthImageToLaserScan()
        self.camera_info = None
        self.depth_image = None

    def depth_callback(self, msg):
        self.depth_image = msg
        if self.camera_info:
            self.convert_depth_to_laserscan()

    def camera_info_callback(self, msg):
        self.camera_info = msg
        if self.depth_image:
            self.convert_depth_to_laserscan()

    def convert_depth_to_laserscan(self):
        # Convert depth image to LaserScan
        try:
            # Convert the depth image to a CV2 format
            depth_cv = self.bridge.imgmsg_to_cv2(self.depth_image, "32FC1")

            # Perform depth image to laser scan conversion
            scan_msg = self.depth_to_laserscan.convert(depth_cv, self.camera_info)

            # Publish LaserScan message
            self.scan_publisher.publish(scan_msg)
            self.get_logger().info(f'Published LaserScan data to {self.scan_topic}')
        except Exception as e:
            self.get_logger().error(f'Error in converting depth image to laser scan: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
