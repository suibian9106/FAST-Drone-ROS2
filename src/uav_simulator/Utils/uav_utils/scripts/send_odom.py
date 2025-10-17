#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import time

class OdomSender(Node):
    def __init__(self):
        super().__init__('odom_sender')
        
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.msg = Odometry()
        self.msg.header.frame_id = "world"
        
        q = quaternion_from_euler(0, 0, 0, "rzyx")
        
        self.msg.pose.pose.position.x = 0.0
        self.msg.pose.pose.position.y = 0.0
        self.msg.pose.pose.position.z = 0.0
        self.msg.twist.twist.linear.x = 0.0
        self.msg.twist.twist.linear.y = 0.0
        self.msg.twist.twist.linear.z = 0.0
        self.msg.pose.pose.orientation.x = q[0]
        self.msg.pose.pose.orientation.y = q[1]
        self.msg.pose.pose.orientation.z = q[2]
        self.msg.pose.pose.orientation.w = q[3]

    def timer_callback(self):
        self.counter += 1
        # Set timestamp with 0.2 seconds offset
        now = self.get_clock().now()
        offset_time = now - rclpy.duration.Duration(seconds=0.2)
        self.msg.header.stamp = offset_time.to_msg()
        
        self.pub.publish(self.msg)
        self.get_logger().info(f"Send {self.counter:3d} msg(s).")

def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()