#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import csv
from datetime import datetime
import math

class IMUValidator(Node):
    def __init__(self):
        super().__init__('imu_validator')
        
        # 订阅雷达发布的速度命令
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # 根据实际情况修改话题名
            self.cmd_vel_callback,
            10)
            
        # 订阅IMU数据
        self.imu_subscription = self.create_subscription(
            Imu,
            '/livox/imu',  # 根据配置文件中的topic
            self.imu_callback,
            10)
            
        # 存储最新数据
        self.latest_cmd_vel = None
        self.latest_imu = None
        
        # 用于积分计算的速度和位置变量
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.last_timestamp = None
        
        # CSV文件写入
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_file = open(f'velocity_estimation_{timestamp}.csv', 'w', newline='')
        self.data_writer = csv.writer(self.data_file)
        
        # 写入表头
        self.data_writer.writerow([
            'timestamp', 
            'cmd_linear_x', 'cmd_angular_z',
            'imu_angular_velocity_z',
            'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',
            'estimated_velocity_x', 'estimated_velocity_y', 'estimated_velocity_magnitude',
            'linear_diff', 'angular_diff'
        ])
        
        self.get_logger().info('IMU Validator Node has started.')
        self.get_logger().info('Estimating velocity from IMU data...')

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.compare_and_record()

    def imu_callback(self, msg):
        # 获取当前时间戳
        current_timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # 如果这是第一次接收IMU数据
        if self.last_timestamp is None:
            self.last_timestamp = current_timestamp
            self.latest_imu = msg
            return
            
        # 计算时间差
        dt = current_timestamp - self.last_timestamp
        
        # 获取加速度数据
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        
        # 积分加速度得到速度 (简单的欧拉积分)
        self.velocity_x += acc_x * dt
        self.velocity_y += acc_y * dt
        self.velocity_z += acc_z * dt
        
        # 更新时间戳
        self.last_timestamp = current_timestamp
        self.latest_imu = msg
        
        # 同时进行比较和记录
        self.compare_and_record()

    def compare_and_record(self):
        # 只有当两个数据都存在时才进行比较
        if self.latest_cmd_vel is not None and self.latest_imu is not None:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            # 获取cmd_vel数据
            cmd_linear_x = self.latest_cmd_vel.linear.x
            cmd_angular_z = self.latest_cmd_vel.angular.z
            
            # 获取IMU数据
            imu_angular_velocity_z = self.latest_imu.angular_velocity.z
            imu_linear_acceleration_x = self.latest_imu.linear_acceleration.x
            imu_linear_acceleration_y = self.latest_imu.linear_acceleration.y
            imu_linear_acceleration_z = self.latest_imu.linear_acceleration.z
            
            # 计算速度大小
            estimated_velocity_magnitude = math.sqrt(self.velocity_x**2 + self.velocity_y**2)
            
            # 计算差异
            linear_diff = abs(cmd_linear_x - estimated_velocity_magnitude)  # 比较命令线速度与估计速度大小
            angular_diff = abs(cmd_angular_z - imu_angular_velocity_z)
            
            # 写入数据
            self.data_writer.writerow([
                timestamp,
                cmd_linear_x, cmd_angular_z,
                imu_angular_velocity_z,
                imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z,
                self.velocity_x, self.velocity_y, estimated_velocity_magnitude,
                linear_diff, angular_diff
            ])
            self.data_file.flush()
            
            # 输出日志
            self.get_logger().info(
                f'Time: {timestamp:.3f} | '
                f'Cmd Linear X: {cmd_linear_x:6.3f} | '
                f'Estimated Vel Mag: {estimated_velocity_magnitude:6.3f} | '
                f'IMU Angular Z: {imu_angular_velocity_z:6.3f} | '
                f'Angular Diff: {angular_diff:6.3f}'
            )

    def destroy_node(self):
        self.get_logger().info('Shutting down IMU Validator Node...')
        if hasattr(self, 'data_file') and not self.data_file.closed:
            self.data_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    validator = IMUValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        # 确保节点被正确销毁
        validator.destroy_node()
        # 只有在ROS2上下文仍然活动时才调用shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()