#!/usr/bin/env python3
"""Test script to verify frame transforms in py_uroc package."""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped


class FrameTestNode(Node):
    def __init__(self):
        super().__init__('frame_test_node')
        
        # TF buffer and listener to check available transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer to periodically check transforms
        self.timer = self.create_timer(2.0, self.check_transforms)
        
    def check_transforms(self):
        """Check if required transforms are available."""
        try:
            # Check if map -> base_link transform exists (from MAVROS)
            map_to_base = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            self.get_logger().info(f"✓ map -> base_link transform available")
            
            # Check if base_link -> gimbal_frame transform exists (from our gimbal nodes)
            base_to_gimbal = self.tf_buffer.lookup_transform(
                'base_link', 'gimbal_frame', rclpy.time.Time())
            self.get_logger().info(f"✓ base_link -> gimbal_frame transform available")
            
            # Check if map -> gimbal_frame full chain works
            map_to_gimbal = self.tf_buffer.lookup_transform(
                'map', 'gimbal_frame', rclpy.time.Time())
            self.get_logger().info(f"✓ Full chain map -> gimbal_frame works!")
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")


def main():
    rclpy.init()
    node = FrameTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
