#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

class RobustInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('robust_initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # 控制发布的参数
        self.publish_count = 0
        self.max_count = 5  # 你想发布的总次数
        self.publish_period = 1.0  # 发布间隔，单位：秒 (频率不宜过高)
        
        # 创建一个定时器，每隔 publish_period 秒调用一次 timer_callback
        self.timer = self.create_timer(self.publish_period, self.timer_callback)
        self.get_logger().info('节点已启动，将开始周期性发布初始位姿...')
        
    def timer_callback(self):
        """定时器回调函数，每次被调用时发布一次位姿"""
        if self.publish_count < self.max_count:
            self.publish_initial_pose()
            self.publish_count += 1
            self.get_logger().info(f'发布第 {self.publish_count} 次初始位姿')
        else:
            # 达到发布次数后，取消定时器
            self.get_logger().info('已完成指定次数的发布，即将关闭节点。')
            self.timer.cancel()  # 停止定时器
            # 可以选择让节点自动退出，或保持运行但不发布
            # rclpy.shutdown()  # 直接关闭，但通常由外部控制
            
    def publish_initial_pose(self):
        # 构建消息内容 (与之前相同)
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.353
        msg.pose.pose.position.y = 0.619
        msg.pose.pose.position.z = 0.005
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.001
        msg.pose.pose.orientation.z = 1.0
        msg.pose.pose.orientation.w = -0.019
        # 协方差矩阵
        msg.pose.covariance[0] = 0.25   # x方差
        msg.pose.covariance[7] = 0.25   # y方差
        msg.pose.covariance[35] = 0.068 # yaw方差
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobustInitialPosePublisher()
    
    try:
        # 保持节点运行，直到定时器停止或收到终止信号
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()