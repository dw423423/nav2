#!/usr/bin/env python3
"""
PS2手柄ROS2节点
读取PS2手柄数据，发布到/cmd_vel话题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time
import threading

class PS2ControllerNode(Node):
    def __init__(self):
        super().__init__('ps2_controller')
        
        # 参数配置
        self.declare_parameter('linear_scale', 0.5)  # 线速度缩放因子 (m/s)
        self.declare_parameter('angular_scale', 1.0) # 角速度缩放因子 (rad/s)
        self.declare_parameter('publish_rate', 200.0) # 发布频率 (Hz)
        self.declare_parameter('deadzone', 0.1)      # 摇杆死区
        
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # 创建/cmd_vel发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 初始化手柄
        self.joystick = None
        self.init_joystick()
        
        # 手柄数据
        self.axis_data = {
            0: 0.0,  # 左摇杆左右
            1: 0.0,  # 左摇杆上下
            2: 0.0,  # 右摇杆左右
            3: 0.0,  # 右摇杆上下
        }
        self.button_data = {}
        
        # 运行标志
        self.running = True
        
        # 创建定时器发布cmd_vel
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        
        # 创建线程读取手柄数据
        self.read_thread = threading.Thread(target=self.read_joystick_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.get_logger().info(f'PS2手柄节点已启动，发布频率: {self.publish_rate}Hz')
        self.get_logger().info('左摇杆: 前后左右移动 | 右摇杆: 左右旋转 | 按钮X: 急停')
    
    def init_joystick(self):
        """初始化Pygame手柄"""
        try:
            pygame.init()
            pygame.joystick.init()
            
            # 等待手柄连接
            attempts = 0
            while pygame.joystick.get_count() == 0 and attempts < 50:
                self.get_logger().info('等待PS2手柄连接... (请确保已配对并打开电源)')
                pygame.event.pump()
                time.sleep(0.1)
                attempts += 1
            
            if pygame.joystick.get_count() == 0:
                self.get_logger().error('未检测到PS2手柄！')
                return False
            
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            self.get_logger().info(f'手柄已连接: {self.joystick.get_name()}')
            self.get_logger().info(f'摇杆数量: {self.joystick.get_numaxes()}')
            self.get_logger().info(f'按钮数量: {self.joystick.get_numbuttons()}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'初始化手柄失败: {e}')
            return False
    
    def read_joystick_data(self):
        """独立线程：持续读取手柄数据"""
        while self.running and rclpy.ok():
            try:
                # 处理Pygame事件
                pygame.event.pump()
                
                if self.joystick is None:
                    time.sleep(0.1)
                    continue
                
                # 读取摇杆数据
                for axis in range(self.joystick.get_numaxes()):
                    if axis < 4:  # 只保存前4个轴的数据
                        value = self.joystick.get_axis(axis)
                        # 应用死区
                        if abs(value) < self.deadzone:
                            value = 0.0
                        self.axis_data[axis] = value
                
                # 读取按钮数据（用于急停等特殊功能）
                for button in range(self.joystick.get_numbuttons()):
                    self.button_data[button] = self.joystick.get_button(button)
                
                time.sleep(0.01)  # 100Hz读取频率
                
            except Exception as e:
                self.get_logger().warn(f'读取手柄数据异常: {e}')
                time.sleep(0.5)
    
    # def publish_cmd_vel(self):
    #     """发布速度命令到/cmd_vel"""
    #     if self.joystick is None:
    #         return
        
    #     # 创建Twist消息
    #     twist_msg = Twist()
        
    #     try:
    #         # 映射手柄数据到速度命令
    #         # 方案1：标准控制模式（左摇杆移动，右摇杆旋转）
    #         left_x = self.axis_data.get(0, 0.0)  # 左摇杆左右
    #         left_y = self.axis_data.get(1, 0.0)  # 左摇杆上下
    #         right_x = self.axis_data.get(2, 0.0) # 右摇杆左右
            
    #         # 方案2：PS2常见布局（可能需要根据实际手柄调整）
    #         # 有些PS2手柄布局不同，这里提供备选方案
    #         # left_x = self.axis_data.get(0, 0.0)
    #         # left_y = self.axis_data.get(1, 0.0)
    #         # right_x = self.axis_data.get(3, 0.0)  # 如果右摇杆左右是轴3
            
    #         # 设置线速度 (m/s)
    #         # 注意：pygame的Y轴向下为正，机器人前进需要取反
    #         twist_msg.linear.x = left_y * self.linear_scale   # 前后移动
    #         twist_msg.linear.y = left_x * self.linear_scale    # 左右移动
    #         twist_msg.linear.z = 0.0
            
    #         # 设置角速度 (rad/s)
    #         twist_msg.angular.x = 0.0
    #         twist_msg.angular.y = 0.0
    #         twist_msg.angular.z = -(right_x * self.angular_scale)  # 旋转
            
    #         # 急停功能（按下X按钮）
    #         if self.button_data.get(0, 0):  # 按钮0通常是X
    #             self.get_logger().warn('急停按钮被按下！')
    #             twist_msg = Twist()  # 发送零速度
            
    #         # 发布消息
    #         self.cmd_vel_pub.publish(twist_msg)
            
    #         # 调试输出（可选，频率调低）
    #         # self.get_logger().debug(f'发布速度: vx={twist_msg.linear.x:.2f}, vy={twist_msg.linear.y:.2f}, ω={twist_msg.angular.z:.2f}')
            
    #     except Exception as e:
    #         self.get_logger().error(f'发布速度命令异常: {e}')

    def publish_cmd_vel(self):
        """发布速度命令到/cmd_vel"""
        if self.joystick is None:
            return
        
        # 创建Twist消息
        twist_msg = Twist()
        
        try:
            # 映射手柄数据到速度命令
            left_x = self.axis_data.get(0, 0.0)  # 左摇杆左右
            left_y = self.axis_data.get(1, 0.0)  # 左摇杆上下
            right_x = self.axis_data.get(2, 0.0) # 右摇杆左右
            
            # 左摇杆控制：输出x和y轴的正负方向速度
            max_linear_speed = 0.3  # 最大线速度
            
            # 直接映射摇杆值到速度，保留正负方向
            twist_msg.linear.x = left_y * max_linear_speed   # 前后移动（注意Y轴向下为正，所以直接使用）
            twist_msg.linear.y = left_x * max_linear_speed   # 左右移动
            twist_msg.linear.z = 0.0
            
            # 右摇杆控制：只设置顺时针或逆时针旋转
            angular_speed = 1.0  # 固定角速度
            
            if right_x > 0.1:  # 向右旋转（顺时针）
                twist_msg.angular.z = -angular_speed  # 负值表示顺时针
            elif right_x < -0.1:  # 向左旋转（逆时针）
                twist_msg.angular.z = angular_speed   # 正值表示逆时针
            else:
                twist_msg.angular.z = 0.0  # 无旋转
            
            # 保持其他角度速度为0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            
            # 急停功能（按下X按钮）
            if self.button_data.get(0, 0):  # 按钮0通常是X
                self.get_logger().warn('急停按钮被按下！')
                twist_msg = Twist()  # 发送零速度
            
            # 发布消息
            self.cmd_vel_pub.publish(twist_msg)
            
            # 调试输出（可选，频率调低）
            # self.get_logger().debug(f'发布速度: vx={twist_msg.linear.x:.2f}, vy={twist_msg.linear.y:.2f}, ω={twist_msg.angular.z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'发布速度命令异常: {e}')
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        # 发送零速度
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.1)
        
        # 关闭pygame
        pygame.quit()
        
        self.get_logger().info('PS2手柄节点已关闭')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PS2ControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()