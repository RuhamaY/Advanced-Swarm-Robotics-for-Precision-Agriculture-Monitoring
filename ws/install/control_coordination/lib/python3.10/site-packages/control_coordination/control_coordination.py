import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CentralCoordinationNode(Node):
    def __init__(self):
        super().__init__('central_coordination_node')
        self.detector_1_subscription = self.create_subscription(
            Point,
            '/detector_1/rust_location',
            self.rust_location_callback,
            10)
        self.detector_2_subscription = self.create_subscription(
            Point,
            '/detector_2/rust_location',
            self.rust_location_callback,
            10)
        self.rust_location_pub = self.create_publisher(Point, '/central_coordination/rust_location', 10)

    def rust_location_callback(self, msg):
        self.get_logger().info(f"Received rust location from detector: x={msg.x}, y={msg.y}")
        # Here we simply forward the rust location to intervention robots
        self.rust_location_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    central_coordination_node = CentralCoordinationNode()
    rclpy.spin(central_coordination_node)
    central_coordination_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

