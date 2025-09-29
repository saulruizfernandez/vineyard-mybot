#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SlamDiagnostics(Node):
    def __init__(self):
        super().__init__('slam_diagnostics')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Timers for diagnostics
        self.create_timer(2.0, self.tf_diagnostics)
        
        self.scan_count = 0
        self.last_odom_to_map = None
        
        self.get_logger().info("SLAM Diagnostics Node Started")
        
    def scan_callback(self, msg):
        self.scan_count += 1
        
        # Check scan quality
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid laser ranges!")
            return
            
        # Log scan statistics every 50 scans
        if self.scan_count % 50 == 0:
            self.get_logger().info(f"Scan {self.scan_count}: "
                                 f"Valid points: {len(valid_ranges)}/{len(ranges)}, "
                                 f"Min: {np.min(valid_ranges):.2f}, "
                                 f"Max: {np.max(valid_ranges):.2f}, "
                                 f"Mean: {np.mean(valid_ranges):.2f}")
    
    def tf_diagnostics(self):
        try:
            # Check odom to base_link
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            
            # Check map to odom
            map_to_odom = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            
            # Check for jumps in map->odom transform
            if self.last_odom_to_map is not None:
                dx = map_to_odom.transform.translation.x - self.last_odom_to_map.transform.translation.x
                dy = map_to_odom.transform.translation.y - self.last_odom_to_map.transform.translation.y
                
                # Calculate rotation difference
                q1 = self.last_odom_to_map.transform.rotation
                q2 = map_to_odom.transform.rotation
                
                # Simple angle difference calculation
                angle_diff = abs(self.quaternion_to_yaw(q2) - self.quaternion_to_yaw(q1))
                if angle_diff > np.pi:
                    angle_diff = 2*np.pi - angle_diff
                
                distance_jump = np.sqrt(dx**2 + dy**2)
                
                if distance_jump > 0.1 or angle_diff > 0.1:  # 10cm or ~6 degrees
                    self.get_logger().warn(f"SLAM JUMP DETECTED! "
                                         f"Distance: {distance_jump:.3f}m, "
                                         f"Angle: {angle_diff:.3f}rad ({np.degrees(angle_diff):.1f}°)")
                else:
                    self.get_logger().info(f"SLAM stable - Distance: {distance_jump:.3f}m, "
                                         f"Angle: {angle_diff:.3f}rad")
            
            self.last_odom_to_map = map_to_odom
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    
    node = SlamDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()