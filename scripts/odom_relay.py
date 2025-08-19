#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/diffbot_base_controller/odom',
            self.listener_callback,
            10)
        self.get_logger().info('Odom Bridge node started, remapping /diffbot_base_controller/odom to /odom')

    def listener_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    odom_bridge = OdomBridge()
    rclpy.spin(odom_bridge)
    odom_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()