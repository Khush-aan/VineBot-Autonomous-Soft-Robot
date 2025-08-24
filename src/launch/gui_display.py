import tkinter as tk
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node

class GUIDisplay(Node):
    def __init__(self):
        super().__init__('gui_display')
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar_scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.bridge = CvBridge()
        self.root = tk.Tk()
        self.root.title('VineBot Sensor Data')
        self.label = tk.Label(self.root, text='Distance: N/A')
        self.label.pack()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.get_logger().info('GUI display started')

    def lidar_callback(self, msg):
        self.label.config(text=f'Distance: {min(msg.ranges):.2f} m')

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)

    def on_closing(self):
        cv2.destroyAllWindows()
        self.root.destroy()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GUIDisplay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
