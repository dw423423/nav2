#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class AutoRelocalizer(Node):
    def __init__(self):
        super().__init__('auto_relocalizer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('period_sec', 1.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.period_sec = float(self.get_parameter('period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10
        )

        self.timer = self.create_timer(self.period_sec, self.publish_initialpose)

        self.get_logger().info(
            f'Auto relocalizer started: every {self.period_sec:.1f}s publish current '
            f'{self.map_frame}->{self.base_frame} to {self.initialpose_topic}'
        )

    def publish_initialpose(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f'Waiting for TF {self.map_frame} -> {self.base_frame}: {e}'
            )
            return

        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published /initialpose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f} deg'
        )


def main():
    rclpy.init()
    node = AutoRelocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()