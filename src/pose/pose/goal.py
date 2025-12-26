#!/usr/bin/env python3
"""
基于NavigateToPose动作的单点导航节点
支持命令行参数指定目标，也支持在代码中硬编码目标
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
import math
import sys
import threading

class SinglePointNavigator(Node):
    def __init__(self):
        super().__init__('single_point_navigator')
        
        # 创建NavigateToPose动作客户端
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 状态变量
        self.goal_handle = None
        self.navigation_result = None
        self.feedback = None
        self.is_navigating = False
        
        self.get_logger().info('单点导航节点已初始化，等待动作服务器...')
        
        # 尝试连接服务器（带超时）
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('导航动作服务器不可用！')
            self.get_logger().info('请确保Nav2系统已启动，例如：')
            self.get_logger().info('ros2 launch nav2_bringup tb3_simulation_launch.py')
        else:
            self.get_logger().info('已连接到导航动作服务器')

    def destroy_node(self):
        """清理资源"""
        if self.is_navigating:
            self.cancel_navigation()
        super().destroy_node()

    def create_goal_pose(self, x, y, yaw_degrees=0.0):
        """
        创建导航目标位姿
        :param x: X坐标 (米)
        :param y: Y坐标 (米)
        :param yaw_degrees: 偏航角 (度)，默认0度（朝东）
        :return: PoseStamped消息
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # 固定使用map坐标系
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0  # 2D导航中Z通常为0
        
        # 将偏航角（度）转换为弧度，然后计算四元数
        yaw_rad = math.radians(float(yaw_degrees))
        
        # 手动计算四元数（避免依赖问题）
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(0.0)  # pitch为0
        sp = math.sin(0.0)  # pitch为0
        cr = math.cos(0.0)  # roll为0
        sr = math.sin(0.0)  # roll为0
        
        # 计算四元数 (绕Z轴旋转yaw)
        goal_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        goal_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        goal_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        goal_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        
        self.get_logger().info(f'创建目标: ({x}, {y}), 朝向: {yaw_degrees}°')
        return goal_pose

    def send_goal(self, x, y, yaw_degrees=0.0):
        """
        发送导航目标
        :return: 是否成功发送
        """
        if not self.action_client.server_is_ready():
            self.get_logger().error('动作服务器未就绪！')
            return False
        
        # 创建目标消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_goal_pose(x, y, yaw_degrees)
        
        # 设置行为树日志（可选）
        goal_msg.behavior_tree = ''
        
        self.get_logger().info(f'正在发送目标到 ({x}, {y})...')
        
        # 发送目标（异步）
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # 设置响应回调
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.is_navigating = True
        return True

    def goal_response_callback(self, future):
        """处理目标接受/拒绝响应"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('目标被导航服务器拒绝')
            self.is_navigating = False
            return
        
        self.get_logger().info('目标已被接受，机器人开始导航')
        
        # 获取结果（异步）
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        self.feedback = feedback_msg.feedback
        current_pose = self.feedback.current_pose
        
        # 提取当前位置和四元数
        pos = current_pose.pose.position
        orient = current_pose.pose.orientation
        
        # 将四元数转换回偏航角（度）用于显示
        # 四元数转欧拉角公式（仅计算yaw）
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        
        # 限制日志频率（每2秒打印一次）
        if hasattr(self, '_last_feedback_log'):
            if (self.get_clock().now() - self._last_feedback_log).nanoseconds > 2e9:
                self.get_logger().info(
                    f'导航中... 位置: ({pos.x:.2f}, {pos.y:.2f}), '
                    f'朝向: {yaw_deg:.1f}°'
                )
                self._last_feedback_log = self.get_clock().now()
        else:
            self._last_feedback_log = self.get_clock().now()
            self.get_logger().info('开始接收导航反馈...')

    def get_result_callback(self, future):
        """处理导航结果"""
        self.is_navigating = False
        
        try:
            result = future.result().result
            # 在NavigateToPose中，结果通常为空或包含一些状态信息
            self.get_logger().info('导航完成！')
            
            # 可以检查是否有错误信息
            if hasattr(result, 'result') and result.result:
                self.get_logger().info(f'结果代码: {result.result}')
                
        except Exception as e:
            self.get_logger().error(f'获取结果时出错: {e}')

    def cancel_navigation(self):
        """取消当前导航"""
        if self.goal_handle and self.is_navigating:
            self.get_logger().info('正在取消导航...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('没有正在进行的导航任务')

    def cancel_done_callback(self, future):
        """取消操作完成回调"""
        try:
            future.result()
            self.get_logger().info('导航已成功取消')
            self.is_navigating = False
        except Exception as e:
            self.get_logger().error(f'取消失败: {e}')

    def spin_until_complete(self, timeout_sec=300):
        """
        阻塞等待导航完成或超时
        :param timeout_sec: 超时时间（秒）
        :return: 是否成功完成（未超时）
        """
        self.get_logger().info(f'等待导航完成，超时时间: {timeout_sec}秒')
        
        # 创建定时器检查超时
        start_time = self.get_clock().now()
        
        while rclpy.ok() and self.is_navigating:
            # 检查超时
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warning(f'导航超时 ({timeout_sec}秒)')
                self.cancel_navigation()
                return False
            
            # 短暂休眠避免CPU占用过高
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return not self.is_navigating


def main(args=None):
    rclpy.init(args=args)
    
    # 创建导航节点
    navigator = SinglePointNavigator()
    
    # 检查命令行参数
    # 用法: ros2 run <package> <node> [x] [y] [yaw_degrees]
    if len(sys.argv) >= 3:
        try:
            target_x = float(sys.argv[1])
            target_y = float(sys.argv[2])
            target_yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
        except ValueError:
            navigator.get_logger().error('参数格式错误！请使用数字。')
            navigator.get_logger().info('示例: ros2 run my_package single_point_navigator 2.0 1.0 90.0')
            rclpy.shutdown()
            return
    else:
        # 使用默认目标（如果没有提供参数）
        navigator.get_logger().info('未提供目标坐标，使用默认值...')
        target_x = -1.3
        target_y = -1.4
        target_yaw = 90.0  # 朝向北方
    
    # 发送导航目标
    if navigator.send_goal(target_x, target_y, target_yaw):
        # 等待导航完成（带超时）
        navigator.spin_until_complete(timeout_sec=300)
    else:
        navigator.get_logger().error('无法发送导航目标')
    
    # 短暂等待确保所有回调处理完成
    import time
    time.sleep(1.0)
    
    # 清理
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()