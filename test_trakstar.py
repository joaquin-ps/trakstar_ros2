#!/usr/bin/env python3

"""
Simple test script to verify Trakstar ROS2 driver is working correctly
"""

import rclpy
from rclpy.node import Node
from trakstar.msg import TrakstarMsg
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs

class TrakstarTestNode(Node):
    def __init__(self):
        super().__init__('trakstar_test')
        
        # Subscribe to trakstar messages
        self.trakstar_sub = self.create_subscription(
            TrakstarMsg,
            'trakstar_msg',
            self.trakstar_callback,
            10
        )
        
        self.raw_sub = self.create_subscription(
            TrakstarMsg,
            'trakstar_raw_msg', 
            self.raw_callback,
            10
        )
        
        # TF buffer for transform queries
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Trakstar test node started")

    def trakstar_callback(self, msg):
        self.get_logger().info(f"Received processed data: {msg.n_tracker} trackers")
        for i in range(msg.n_tracker):
            t = msg.transform[i]
            self.get_logger().info(f"  Tracker {i}: pos=({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f})")

    def raw_callback(self, msg):
        self.get_logger().info(f"Received raw data: {msg.n_tracker} trackers")

def main(args=None):
    rclpy.init(args=args)
    node = TrakstarTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
