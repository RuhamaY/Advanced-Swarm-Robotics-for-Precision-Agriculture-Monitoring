import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RustDetectionNode(Node):
    def __init__(self, robot_name):
        super().__init__('rust_detection_node')
        self.robot_name = robot_name  # Name for this robot (detector_1 or detector_2)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.rust_location_pub = self.create_publisher(Point, f'/{self.robot_name}/rust_location', 10)
        self.bridge = CvBridge()

        self.robot_status = "idle"  # Initial robot status
        self.handling_rust = False

    def image_callback(self, msg):
        if self.handling_rust:
            return
        
        # Detect rust logic
        rust_detected, rust_centroid = self.detect_rust(msg)
        
        if rust_detected:
            rust_location = self.calculate_rust_location(rust_centroid)
            self.rust_location_pub.publish(rust_location)
            self.robot_status = f"handling rust at x={rust_location.x}, y={rust_location.y}"
            self.get_logger().info(f"Rust detected at {rust_location.x}, {rust_location.y}")
    
    def detect_rust(self, msg):
        # Simplified example: if image is predominantly red, assume rust detected
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([100, 100, 255])
        mask = cv2.inRange(cv_image, lower_red, upper_red)
        if np.sum(mask) > 1000:  # arbitrary threshold for rust detection
            rust_centroid = np.mean(np.where(mask > 0), axis=1)
            return True, rust_centroid
        return False, (0, 0)

    def calculate_rust_location(self, rust_centroid):
        # Convert image centroid to world coordinates (simplified)
        rust_location = Point()
        rust_location.x = rust_centroid[0]
        rust_location.y = rust_centroid[1]
        rust_location.z = 0.0
        return rust_location

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'detector_1'  # Change to 'detector_2' for the second robot
    rust_detection_node = RustDetectionNode(robot_name)
    rclpy.spin(rust_detection_node)
    rust_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

