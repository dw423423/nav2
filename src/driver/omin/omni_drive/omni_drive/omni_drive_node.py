#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
import logging
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from omni_kinematics import OmniKinematics
from can_driver import CANDriver

class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')
        
        # 设置日志级别
        logging.basicConfig(level=logging.INFO)
        
        # 参数配置
        self.declare_parameter('wheel_radius', 0.05)           # 轮子半径 (米)
        self.declare_parameter('wheel_distance', 0.15)         # 轮子到底盘中心的距离 (米)
        self.declare_parameter('can_channel', 'can2')          # CAN接口名称
        self.declare_parameter('can_bitrate', 500000)          # CAN波特率
        self.declare_parameter('setup_can_interface', True)    # 是否自动配置CAN接口
        self.declare_parameter('max_linear_speed', 1.0)        # 最大线速度 (m/s)
        self.declare_parameter('max_angular_speed', 2.0)       # 最大角速度 (rad/s)
        self.declare_parameter('driver1_address', 0x01)        # 驱动器1地址
        self.declare_parameter('driver2_address', 0x02)        # 驱动器2地址
        self.declare_parameter('publish_rate', 100.0)           # 发布频率 (Hz)
        self.declare_parameter('timeout', 0.1)                 # 速度命令超时时间 (秒)
        
        # 获取参数
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_distance = self.get_parameter('wheel_distance').value
        can_channel = self.get_parameter('can_channel').value
        can_bitrate = self.get_parameter('can_bitrate').value
        setup_can = self.get_parameter('setup_can_interface').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.driver1_address = self.get_parameter('driver1_address').value
        self.driver2_address = self.get_parameter('driver2_address').value
        publish_rate = self.get_parameter('publish_rate').value
        self.timeout = self.get_parameter('timeout').value
        
        # 初始化运动学计算
        self.kinematics = OmniKinematics(wheel_radius, wheel_distance)
        
        # 初始化CAN驱动
        self.can_driver = CANDriver(
            channel=can_channel,
            bitrate=can_bitrate,
            setup_can_interface=setup_can
        )
        
        # # 验证CAN连接
        # if not self.can_driver.bus:
        #     self.get_logger().error("CAN驱动初始化失败！请检查CAN接口和权限。")
        #     self.get_logger().info(f'CAN接口: {can_channel}')
        #     self.get_logger().info(f'CANbitrate: {can_bitrate}')
        #     self.get_logger().info(f'CANsetup: {setup_can}')            
        #     self.get_logger().info("尝试使用以下命令手动配置CAN接口：")
        #     self.get_logger().info(f"  sudo ip link set {can_channel} type can bitrate {can_bitrate}")
        #     self.get_logger().info(f"  sudo ip link set {can_channel} up")
        #     return
        
        # 测试CAN连接
        if self.can_driver.test_connection():
            self.get_logger().info("CAN连接测试成功")
        else:
            self.get_logger().warning("CAN连接测试失败，可能没有收到响应")
        
        # 订阅cmd_vel话题
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 状态变量
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # 驱动器状态
        self.driver_enabled = {
            self.driver1_address: True,
            self.driver2_address: True
        }
        
        # 添加标志位，避免重复打印停止信息
        self.motors_stopped = False
        
        self.get_logger().info('四轮全向底盘驱动节点已启动')

        self.get_logger().info(f'CAN接口: {can_channel} @ {can_bitrate}bps')
        self.get_logger().info(f'最大速度: {self.max_linear_speed}m/s, 最大角速度: {self.max_angular_speed}rad/s')
    
    def cmd_vel_callback(self, msg):
        """
        处理速度命令
        :param msg: Twist消息，包含线速度和角速度
        """
        # 获取速度命令
        self.current_vx = msg.linear.x      # x方向线速度 (m/s)
        self.current_vy = msg.linear.y      # y方向线速度 (m/s)
        self.current_omega = msg.angular.z  # 角速度 (rad/s)
        
        # 立即执行速度命令
        self.execute_velocity_command()
        
        # 重置停止标志，因为收到了新的命令
        self.motors_stopped = False
    
    def execute_velocity_command(self):
        """
        执行速度命令，计算轮子速度并发送到驱动器
        """
        # 限制速度范围
        linear_speed = math.sqrt(self.current_vx**2 + self.current_vy**2)
        if linear_speed > self.max_linear_speed:
            scale = self.max_linear_speed / linear_speed
            self.current_vx *= scale
            self.current_vy *= scale
        
        if abs(self.current_omega) > self.max_angular_speed:
            self.current_omega = self.max_angular_speed if self.current_omega > 0 else -self.max_angular_speed
        
        # 计算轮子速度 (m/s)
        wheel_speeds = self.kinematics.inverse_kinematics(
            self.current_vx, 
            self.current_vy, 
            self.current_omega
        )
        
        # 转换为 mm/s (驱动器单位)
        wheel_speeds_mmps = [speed * 1000 for speed in wheel_speeds]
        
        # 分配轮子到驱动器
        # 轮子顺序: [左前, 右前, 左后, 右后]
        front_left_speed = wheel_speeds_mmps[0]   # 左前轮
        front_right_speed = wheel_speeds_mmps[1]  # 右前轮
        rear_left_speed = wheel_speeds_mmps[2]    # 左后轮
        rear_right_speed = wheel_speeds_mmps[3]   # 右后轮
        
        # 添加调试信息
        self.get_logger().debug(f'Wheel speeds: FL={front_left_speed:.2f}, FR={front_right_speed:.2f}, RL={rear_left_speed:.2f}, RR={rear_right_speed:.2f}')
        
        # 发送到驱动器1 (地址0x01): 前轮
        if self.driver_enabled.get(self.driver1_address, False):
            success1 = self.can_driver.send_control_command(
                address=self.driver1_address,
                motor1_speed=front_left_speed,   # 电机1: 左前轮
                motor2_speed=front_right_speed,  # 电机2: 右前轮
                control_mode=0x03
            )
            
            if not success1:
                self.get_logger().warning(f"驱动器1 (地址{hex(self.driver1_address)}) 通信失败")
                self.driver_enabled[self.driver1_address] = False
        
        # 发送到驱动器2 (地址0x02): 后轮
        if self.driver_enabled.get(self.driver2_address, False):
            success2 = self.can_driver.send_control_command(
                address=self.driver2_address,
                motor1_speed=rear_left_speed,    # 电机1: 左后轮
                motor2_speed=rear_right_speed,   # 电机2: 右后轮
                control_mode=0x03
            )
            
            if not success2:
                self.get_logger().warning(f"驱动器2 (地址{hex(self.driver2_address)}) 通信失败")
                self.driver_enabled[self.driver2_address] = False
        
        # 如果所有驱动器都失败，记录错误
        if not any(self.driver_enabled.values()):
            self.get_logger().error("所有驱动器通信失败！")
    
    def stop_all_motors(self):
        """停止所有电机"""
        # 只在确实需要停止时才打印信息
        if not self.motors_stopped:
            self.get_logger().info("停止所有电机")
        
        for address in [self.driver1_address, self.driver2_address]:
            self.can_driver.send_control_command(
                address=address,
                motor1_speed=0,
                motor2_speed=0,
                control_mode=0x03
            )
    
    def emergency_stop(self):
        """紧急停止"""
        self.get_logger().warning("紧急停止！")
        self.stop_all_motors()
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        self.motors_stopped = True
    
    def destroy_node(self):
        """节点销毁时停止电机"""
        self.get_logger().info("关闭驱动节点...")
        self.stop_all_motors()
        if self.can_driver:
            self.can_driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None  # 初始化node变量为None
    
    try:
        node = OmniDriveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("接收到Ctrl+C，正在关闭...")
    except Exception as e:
        if node:
            node.get_logger().error(f"节点运行错误: {e}")
        else:
            print(f"节点初始化错误: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import numpy as np
# import math
# import logging
# import sys
# import os

# # 添加当前目录到Python路径
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# from omni_kinematics import OmniKinematics
# from can_driver import CANDriver

# class OmniDriveNode(Node):
#     def __init__(self):
#         super().__init__('omni_drive_node')
        
#         # 设置日志级别
#         logging.basicConfig(level=logging.INFO)
        
#         # 参数配置
#         self.declare_parameter('wheel_radius', 0.05)           # 轮子半径 (米)
#         self.declare_parameter('wheel_distance', 0.15)         # 轮子到底盘中心的距离 (米)
#         self.declare_parameter('can_channel', 'can2')          # CAN接口名称
#         self.declare_parameter('can_bitrate', 500000)          # CAN波特率
#         self.declare_parameter('setup_can_interface', True)    # 是否自动配置CAN接口
#         self.declare_parameter('max_linear_speed', 1.0)        # 最大线速度 (m/s)
#         self.declare_parameter('max_angular_speed', 2.0)       # 最大角速度 (rad/s)
#         self.declare_parameter('driver1_address', 0x01)        # 驱动器1地址
#         self.declare_parameter('driver2_address', 0x02)        # 驱动器2地址
#         self.declare_parameter('publish_rate', 50.0)           # 发布频率 (Hz)
#         self.declare_parameter('timeout', 0.05)                 # 速度命令超时时间 (秒)
        
#         # 获取参数
#         wheel_radius = self.get_parameter('wheel_radius').value
#         wheel_distance = self.get_parameter('wheel_distance').value
#         can_channel = self.get_parameter('can_channel').value
#         can_bitrate = self.get_parameter('can_bitrate').value
#         setup_can = self.get_parameter('setup_can_interface').value
#         self.max_linear_speed = self.get_parameter('max_linear_speed').value
#         self.max_angular_speed = self.get_parameter('max_angular_speed').value
#         self.driver1_address = self.get_parameter('driver1_address').value
#         self.driver2_address = self.get_parameter('driver2_address').value
#         publish_rate = self.get_parameter('publish_rate').value
#         self.timeout = self.get_parameter('timeout').value
        
#         # 初始化运动学计算
#         self.kinematics = OmniKinematics(wheel_radius, wheel_distance)
        
#         # 初始化CAN驱动
#         self.can_driver = CANDriver(
#             channel=can_channel,
#             bitrate=can_bitrate,
#             setup_can_interface=setup_can
#         )
        

#         # 测试CAN连接
#         if self.can_driver.test_connection():
#             self.get_logger().info("CAN连接测试成功")
#         else:
#             self.get_logger().warning("CAN连接测试失败，可能没有收到响应")
        
#         # 订阅cmd_vel话题
#         self.subscription = self.create_subscription(
#             Twist,
#             'cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
        
#         # 定时器，用于安全检查和超时处理
#         # self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
#         # 状态变量
#         self.last_cmd_time = self.get_clock().now()  # 初始化为当前时间，避免立即超时
#         self.current_vx = 0.0
#         self.current_vy = 0.0
#         self.current_omega = 0.0
        
#         # 驱动器状态
#         self.driver_enabled = {
#             self.driver1_address: True,
#             self.driver2_address: True
#         }
        
#         # 添加标志位，避免重复打印停止信息
#         self.motors_stopped = False
        
#         self.get_logger().info('四轮全向底盘驱动节点已启动')
#         self.get_logger().info(f'轮子半径: {wheel_radius}m, 轮距: {wheel_distance}m')
#         self.get_logger().info(f'CAN接口: {can_channel} @ {can_bitrate}bps')
#         self.get_logger().info(f'最大速度: {self.max_linear_speed}m/s, 最大角速度: {self.max_angular_speed}rad/s')
    
#     def cmd_vel_callback(self, msg):
#         """
#         处理速度命令
#         :param msg: Twist消息，包含线速度和角速度
#         """
#         # 更新最后命令时间
#         self.last_cmd_time = self.get_clock().now()
        
#         # 获取速度命令
#         self.current_vx = msg.linear.x      # x方向线速度 (m/s)
#         self.current_vy = msg.linear.y      # y方向线速度 (m/s)
#         self.current_omega = msg.angular.z  # 角速度 (rad/s)
        
#         # 立即执行速度命令
#         self.execute_velocity_command()
        
#         # 重置停止标志，因为收到了新的命令
#         self.motors_stopped = False
    
#     def execute_velocity_command(self):
#         """
#         执行速度命令，计算轮子速度并发送到驱动器
#         """
#         # 限制速度范围
#         linear_speed = math.sqrt(self.current_vx**2 + self.current_vy**2)
#         if linear_speed > self.max_linear_speed:
#             scale = self.max_linear_speed / linear_speed
#             self.current_vx *= scale
#             self.current_vy *= scale
        
#         if abs(self.current_omega) > self.max_angular_speed:
#             self.current_omega = self.max_angular_speed if self.current_omega > 0 else -self.max_angular_speed
        
#         # 计算轮子速度 (m/s)
#         wheel_speeds = self.kinematics.inverse_kinematics(
#             self.current_vx, 
#             self.current_vy, 
#             self.current_omega
#         )
        
#         # 转换为 mm/s (驱动器单位)
#         wheel_speeds_mmps = [speed * 1000 for speed in wheel_speeds]
        
#         # 分配轮子到驱动器
#         # 轮子顺序: [左前, 右前, 左后, 右后]
#         front_left_speed = wheel_speeds_mmps[0]   # 左前轮
#         front_right_speed = wheel_speeds_mmps[1]  # 右前轮
#         rear_left_speed = wheel_speeds_mmps[2]    # 左后轮
#         rear_right_speed = wheel_speeds_mmps[3]   # 右后轮
        
#         # 发送到驱动器1 (地址0x01): 前轮
#         if self.driver_enabled.get(self.driver1_address, False):
#             success1 = self.can_driver.send_control_command(
#                 address=self.driver1_address,
#                 motor1_speed=front_left_speed,   # 电机1: 左前轮
#                 motor2_speed=front_right_speed,  # 电机2: 右前轮
#                 control_mode=0x03
#             )
            
#             if not success1:
#                 self.get_logger().warning(f"驱动器1 (地址{hex(self.driver1_address)}) 通信失败")
#                 self.driver_enabled[self.driver1_address] = False
        
#         # 发送到驱动器2 (地址0x02): 后轮
#         if self.driver_enabled.get(self.driver2_address, False):
#             success2 = self.can_driver.send_control_command(
#                 address=self.driver2_address,
#                 motor1_speed=rear_left_speed,    # 电机1: 左后轮
#                 motor2_speed=rear_right_speed,   # 电机2: 右后轮
#                 control_mode=0x03
#             )
            
#             if not success2:
#                 self.get_logger().warning(f"驱动器2 (地址{hex(self.driver2_address)}) 通信失败")
#                 self.driver_enabled[self.driver2_address] = False
        
#         # 如果所有驱动器都失败，记录错误
#         if not any(self.driver_enabled.values()):
#             self.get_logger().error("所有驱动器通信失败！")
    
#     # def timer_callback(self):
#     #     """
#     #     定时器回调，用于安全检查和超时处理
#     #     """
#     #     current_time = self.get_clock().now()
#     #     elapsed_time = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
#     #     # 检查命令超时
#     #     if elapsed_time > self.timeout:
#     #         # 超时，停止电机
#     #         if not self.motors_stopped:  # 只在电机未停止时才执行
#     #             self.get_logger().debug(f"速度命令超时 ({elapsed_time:.1f}s > {self.timeout}s)，停止电机")
#     #             self.stop_all_motors()
#     #             self.current_vx = 0.0
#     #             self.current_vy = 0.0
#     #             self.current_omega = 0.0
#     #             self.motors_stopped = True  # 标记电机已停止
        
#     #     # 尝试重新连接失败的驱动器
#     #     for address, enabled in self.driver_enabled.items():
#     #         if not enabled:
#     #             # 尝试发送一个测试命令
#     #             self.get_logger().info(f"尝试重新连接驱动器 (地址{hex(address)})")
#     #             test_success = self.can_driver.send_control_command(
#     #                 address=address,
#     #                 motor1_speed=0,
#     #                 motor2_speed=0,
#     #                 control_mode=0x03
#     #             )
                
#     #             if test_success:
#     #                 self.get_logger().info(f"驱动器 (地址{hex(address)}) 重新连接成功")
#     #                 self.driver_enabled[address] = True
    
#     def stop_all_motors(self):
#         """停止所有电机"""
#         # 只在确实需要停止时才打印信息
#         if not self.motors_stopped:
#             self.get_logger().info("停止所有电机")
        
#         for address in [self.driver1_address, self.driver2_address]:
#             self.can_driver.send_control_command(
#                 address=address,
#                 motor1_speed=0,
#                 motor2_speed=0,
#                 control_mode=0x03
#             )
    
#     def emergency_stop(self):
#         """紧急停止"""
#         self.get_logger().warning("紧急停止！")
#         self.stop_all_motors()
#         self.current_vx = 0.0
#         self.current_vy = 0.0
#         self.current_omega = 0.0
#         self.motors_stopped = True
    
#     def destroy_node(self):
#         """节点销毁时停止电机"""
#         self.get_logger().info("关闭驱动节点...")
#         self.stop_all_motors()
#         if self.can_driver:
#             self.can_driver.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
    
#     node = None  # 初始化node变量为None
    
#     try:
#         node = OmniDriveNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         if node:
#             node.get_logger().info("接收到Ctrl+C，正在关闭...")
#     except Exception as e:
#         if node:
#             node.get_logger().error(f"节点运行错误: {e}")
#         else:
#             print(f"节点初始化错误: {e}")
#     finally:
#         if node is not None:
#             node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()