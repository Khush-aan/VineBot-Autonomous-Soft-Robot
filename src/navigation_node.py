import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.sub = self.create_subscription(LaserScan, '/lidar_scan', self.lidar_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Navigation node started')

    def lidar_callback(self, msg):
        twist = Twist()
        if min(msg.ranges) < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
