import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String

class InterventionNode(Node):
    def __init__(self, robot_name):
        super().__init__('intervention_node')
        self.robot_name = robot_name
        self.status_pub = self.create_publisher(String, f'/{self.robot_name}/status', 10)
        self.rust_location_sub = self.create_subscription(Point, '/central_coordination/rust_location', self.rust_location_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

    def rust_location_callback(self, msg):
        rust_location = msg
        self.get_logger().info(f"Received rust location at x={rust_location.x}, y={rust_location.y}")
        self.intervene_at_location(rust_location)
    
    def intervene_at_location(self, rust_location):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.status_pub.publish(String(data=f"Intervening at x={rust_location.x}, y={rust_location.y}"))
        self.get_logger().info(f"Intervening at {rust_location.x}, {rust_location.y}")

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'intervention_1'
    intervention_node = InterventionNode(robot_name)
    rclpy.spin(intervention_node)
    intervention_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

