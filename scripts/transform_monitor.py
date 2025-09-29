#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import time

class TransformMonitor(Node):
    def __init__(self):
        super().__init__('transform_monitor')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.create_timer(0.1, self.monitor_transforms)  # 10Hz monitoring
        
        self.last_map_to_odom = None
        self.jump_count = 0
        
        self.get_logger().info("Transform Monitor Started - Watching for jumps...")
        
    def monitor_transforms(self):
        try:
            # Get current map->odom transform
            current_transform = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            
            if self.last_map_to_odom is not None:
                # Calculate position change
                dx = current_transform.transform.translation.x - self.last_map_to_odom.transform.translation.x
                dy = current_transform.transform.translation.y - self.last_map_to_odom.transform.translation.y
                dz = current_transform.transform.translation.z - self.last_map_to_odom.transform.translation.z
                
                # Calculate rotation change
                current_q = current_transform.transform.rotation
                last_q = self.last_map_to_odom.transform.rotation
                
                current_yaw = self.quaternion_to_yaw(current_q)
                last_yaw = self.quaternion_to_yaw(last_q)
                
                dyaw = abs(current_yaw - last_yaw)
                if dyaw > np.pi:
                    dyaw = 2*np.pi - dyaw
                
                distance_change = np.sqrt(dx**2 + dy**2 + dz**2)
                
                # Define thresholds for "jumps"
                DISTANCE_THRESHOLD = 0.05  # 5cm
                ANGLE_THRESHOLD = 0.087    # ~5 degrees
                
                if distance_change > DISTANCE_THRESHOLD or dyaw > ANGLE_THRESHOLD:
                    self.jump_count += 1
                    self.get_logger().warn(f"🚨 JUMP #{self.jump_count} DETECTED!")
                    self.get_logger().warn(f"   Position change: {distance_change:.4f}m (dx:{dx:.4f}, dy:{dy:.4f})")
                    self.get_logger().warn(f"   Angle change: {dyaw:.4f}rad ({np.degrees(dyaw):.2f}°)")
                    self.get_logger().warn(f"   Current pos: x:{current_transform.transform.translation.x:.3f}, y:{current_transform.transform.translation.y:.3f}")
                    self.get_logger().warn(f"   Current yaw: {np.degrees(current_yaw):.2f}°")
                    print("-" * 60)
                else:
                    # Only log small changes occasionally
                    if int(time.time()) % 5 == 0:  # Every 5 seconds
                        self.get_logger().info(f"✅ Stable - pos_change: {distance_change:.4f}m, angle_change: {np.degrees(dyaw):.2f}°")
            
            self.last_map_to_odom = current_transform
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform lookup failed: {str(e)}")
    
    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main():
    rclpy.init()
    
    monitor = TransformMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print(f"\nMonitoring stopped. Total jumps detected: {monitor.jump_count}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()