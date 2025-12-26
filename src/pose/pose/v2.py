#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import math
import sys
import os

class VelocityAnalyzer(Node):
    def __init__(self):
        super().__init__('velocity_analyzer')
        self.get_logger().info('Velocity Analyzer Node has started.')
        
    def analyze_velocity_comparison(self, csv_file):
        """
        分析cmd_vel和IMU数据的对比
        """
        # 检查文件是否存在
        if not os.path.exists(csv_file):
            self.get_logger().error(f'File {csv_file} does not exist!')
            return None
            
        # 读取数据
        df = pd.read_csv(csv_file)
        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
        
        print("Velocity Comparison Statistics:")
        print("=" * 50)
        print(df.describe())
        
        # 计算统计数据
        angular_correlation = df['cmd_angular_z'].corr(df['imu_angular_velocity_z'])
        angular_rmse = np.sqrt(np.mean(df['angular_diff']**2))
        
        # 计算线速度相关性（如果数据中包含估计的线速度）
        linear_correlation = None
        linear_rmse = None
        if 'estimated_velocity_magnitude' in df.columns:
            linear_correlation = df['cmd_linear_x'].corr(df['estimated_velocity_magnitude'])
            linear_rmse = np.sqrt(np.mean((df['cmd_linear_x'] - df['estimated_velocity_magnitude'])**2))
            print(f"\nLinear Velocity Correlation: {linear_correlation:.4f}")
            print(f"Linear Velocity RMSE: {linear_rmse:.4f}")
        
        print(f"\nAngular Velocity Correlation: {angular_correlation:.4f}")
        print(f"Angular Velocity RMSE: {angular_rmse:.4f}")
        
        # 绘图
        if 'estimated_velocity_magnitude' in df.columns:
            # 如果有估计的线速度，创建4个子图
            fig, axes = plt.subplots(4, 1, figsize=(15, 16))
            
            # 线速度命令 vs 估计线速度
            axes[0].plot(df['timestamp'], df['cmd_linear_x'], 'b-', linewidth=1, label='Commanded Linear X')
            axes[0].plot(df['timestamp'], df['estimated_velocity_magnitude'], 'r-', linewidth=1, label='Estimated Linear Velocity')
            axes[0].set_title('Linear Velocity Comparison')
            axes[0].set_ylabel('Linear Velocity (m/s)')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            
            # 线速度差异
            linear_diff = abs(df['cmd_linear_x'] - df['estimated_velocity_magnitude'])
            axes[1].plot(df['timestamp'], linear_diff, 'g-', linewidth=1)
            axes[1].set_title('Linear Velocity Difference (RMSE: {:.4f})'.format(linear_rmse))
            axes[1].set_ylabel('Absolute Difference (m/s)')
            axes[1].grid(True, alpha=0.3)
        else:
            # 如果没有估计的线速度，创建3个子图
            fig, axes = plt.subplots(3, 1, figsize=(15, 12))
            
        # 角速度命令
        axes[-3].plot(df['timestamp'], df['cmd_angular_z'], 'b-', linewidth=1)
        axes[-3].set_title('Commanded Angular Velocity (Z)')
        axes[-3].set_ylabel('Angular Velocity (rad/s)')
        axes[-3].grid(True, alpha=0.3)
        
        # 角速度对比
        axes[-2].plot(df['timestamp'], df['cmd_angular_z'], 'b-', linewidth=1, label='Commanded Angular Z')
        axes[-2].plot(df['timestamp'], df['imu_angular_velocity_z'], 'r-', linewidth=1, label='IMU Angular Z')
        axes[-2].set_title('Angular Velocity Comparison')
        axes[-2].set_ylabel('Angular Velocity (rad/s)')
        axes[-2].legend()
        axes[-2].grid(True, alpha=0.3)
        
        # 角速度差异
        axes[-1].plot(df['timestamp'], df['angular_diff'], 'g-', linewidth=1)
        axes[-1].set_title('Angular Velocity Difference (RMSE: {:.4f})'.format(angular_rmse))
        axes[-1].set_ylabel('Absolute Difference (rad/s)')
        axes[-1].set_xlabel('Time')
        axes[-1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_filename = f'velocity_comparison_analysis_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
        plt.savefig(output_filename, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Saved analysis plot to {output_filename}')
        plt.show()
        
        # 线速度散点图（如果有估计的线速度）
        if 'estimated_velocity_magnitude' in df.columns:
            plt.figure(figsize=(8, 6))
            plt.scatter(df['cmd_linear_x'], df['estimated_velocity_magnitude'], alpha=0.5)
            plt.xlabel('Commanded Linear Velocity (m/s)')
            plt.ylabel('Estimated Linear Velocity (m/s)')
            plt.title(f'Linear Velocity Correlation (r={linear_correlation:.4f})')
            plt.grid(True, alpha=0.3)
            plt.plot([0, max(df['cmd_linear_x'].max(), df['estimated_velocity_magnitude'].max())], 
                     [0, max(df['cmd_linear_x'].max(), df['estimated_velocity_magnitude'].max())], 
                     'r--', linewidth=1)  # 对角线参考
            plt.xlim(0, max(df['cmd_linear_x'].max(), df['estimated_velocity_magnitude'].max()))
            plt.ylim(0, max(df['cmd_linear_x'].max(), df['estimated_velocity_magnitude'].max()))
            linear_correlation_filename = f'linear_velocity_correlation_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
            plt.savefig(linear_correlation_filename, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved linear correlation plot to {linear_correlation_filename}')
            plt.show()
        
        # 角速度散点图
        plt.figure(figsize=(8, 6))
        plt.scatter(df['cmd_angular_z'], df['imu_angular_velocity_z'], alpha=0.5)
        plt.xlabel('Commanded Angular Velocity (rad/s)')
        plt.ylabel('IMU Measured Angular Velocity (rad/s)')
        plt.title(f'Angular Velocity Correlation (r={angular_correlation:.4f})')
        plt.grid(True, alpha=0.3)
        plt.plot([-3, 3], [-3, 3], 'r--', linewidth=1)  # 对角线参考
        plt.xlim(-3, 3)
        plt.ylim(-3, 3)
        correlation_filename = f'angular_velocity_correlation_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
        plt.savefig(correlation_filename, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Saved correlation plot to {correlation_filename}')
        plt.show()
        
        return df

def main(args=None):
    # 检查是否有提供文件参数
    if len(sys.argv) < 2:
        print("Usage: ros2 run pose v2_node <velocity_comparison_file.csv>")
        print("   or: python3 v2.py <velocity_comparison_file.csv>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    # 初始化ROS2
    rclpy.init(args=args)
    analyzer = VelocityAnalyzer()
    
    try:
        # 分析数据
        df = analyzer.analyze_velocity_comparison(csv_file)
        if df is not None:
            analyzer.get_logger().info('Analysis completed successfully.')
    except Exception as e:
        analyzer.get_logger().error(f'Error during analysis: {str(e)}')
    finally:
        # 清理
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()