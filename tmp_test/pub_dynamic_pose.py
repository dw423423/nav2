#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DynamicPosePub(Node):
    def __init__(self):
        super().__init__('dynamic_pose_test_pub')
        self.pub = self.create_publisher(PoseStamped, '/detected_dynamic_pose', 10)
        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz

    def on_timer(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = 2.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DynamicPosePub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
