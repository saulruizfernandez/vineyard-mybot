#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped')

        # Subscriber al topic original de Twist
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_aux',
            self.twist_callback,
            10)

        # Publisher al topic de salida TwistStamped
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10)

    def twist_callback(self, msg: Twist):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_footprint'  # Frame correcto para comandos de velocidad
        stamped_msg.twist = msg
        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
