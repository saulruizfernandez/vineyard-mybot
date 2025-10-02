#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Suscriptor a cmd_vel_nav (lo que publica Nav2)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher a cmd_vel como TwistStamped (lo que espera el robot)
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('CMD Vel Relay started: /cmd_vel_nav -> /cmd_vel (TwistStamped)')

    def cmd_vel_callback(self, msg):
        # Convertir Twist a TwistStamped
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist = msg
        
        # Publicar el mensaje convertido
        self.publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()