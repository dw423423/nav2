#!/usr/bin/env python3
# coding=utf8
import rclpy
from rclpy.node import Node
import smbus
import time
from std_msgs.msg import Int32, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# I2C地址
I2C_ADDR = 0x34  # I2C地址
ASR_RESULT_ADDR = 100  # ASR结果寄存器地址
ASR_SPEAK_ADDR = 110  # ASR说话寄存器地址
ASR_CMDMAND = 0x00
ASR_ANNOUNCER = 0xFF

class ASRModule:
    def __init__(self, address, bus=7):
        # 初始化 I2C 总线和设备地址
        self.bus = smbus.SMBus(bus)  # 使用 I2C 总线 7
        self.address = address  # 设备的 I2C 地址
        self.send = [0, 0]  # 初始化发送数据的数组

    def wire_write_byte(self, val):
        """
        向设备写入单个字节
        :param val: 要写入的字节值
        :return: 如果成功写入返回 True，失败返回 False
        """
        try:
            self.bus.write_byte(self.address, val) # 发送字节到设备
            return True # 写入成功
        except IOError:
            return False # 写入失败，返回 False

    def wire_write_data_array(self, reg, val, length):
        """
        向指定寄存器写入字节数组
        :param reg: 寄存器地址
        :param val: 要写入的字节数组
        :param length: 要写入的字节数
        :return: 如果成功写入返回 True，失败返回 False
        """
        try:            
            self.bus.write_i2c_block_data(self.address, reg, val[:length]) # 发送字节数组到设备的指定寄存器
            return True # 写入成功
        except IOError:
            return False # 写入失败，返回 False

    def wire_read_data_array(self, reg, length):
        """
        从指定寄存器读取字节数组
        :param reg: 寄存器地址
        :param length: 要读取的字节数
        :return: 读取到的字节数组，失败时返回空数组
        """         
        try:
            result = self.bus.read_i2c_block_data(self.address, reg, 1) # 从设备读取字节数组
            return result # 返回读取结果
        except IOError:
            return [] # 读取失败，返回空数组

    def rec_recognition(self):
        """
        识别结果读取
        :return: 识别结果，如果读取失败返回 0
        """
        result = self.wire_read_data_array(ASR_RESULT_ADDR, 1) # 从结果寄存器读取一个字节
        if result:
            return result # 返回读取到的结果
        return 0  # 如果没有结果，返回 0

    def speak(self, cmd, id):
        """
        向设备发送说话命令
        :param cmd: 命令字节
        :param id: 说话的 ID
        """
        if cmd == ASR_ANNOUNCER or cmd == ASR_CMDMAND: # 检查命令是否有效
            self.send[0] = cmd # 设置发送数组的第一个元素为命令
            self.send[1] = id # 设置发送数组的第二个元素为 ID
            self.wire_write_data_array(ASR_SPEAK_ADDR, self.send, 2) # 发送命令和 ID 到指定寄存器


class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # 初始化参数
        self.declare_parameter('i2c_address', I2C_ADDR)
        self.declare_parameter('i2c_bus', 7)
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        
        # 初始化ASR模块
        self.asr_module = ASRModule(self.i2c_address, self.i2c_bus)
        
        # 创建发布者
        qos = QoSProfile(depth=10)
        self.result_publisher = self.create_publisher(Int32, 'asr_result', qos)
        self.command_publisher = self.create_publisher(String, 'voice_command', qos)
        
        # 创建定时器，定期读取识别结果
        self.timer = self.create_timer(0.1, self.read_asr_result)  # 每0.1秒读取一次
        
        self.get_logger().info(f'语音识别节点已启动，I2C地址: 0x{self.i2c_address:02X}, 总线: {self.i2c_bus}')

    def read_asr_result(self):
        """
        定期读取ASR识别结果并发布
        """
        recognition_result = self.asr_module.rec_recognition()
        
        # 检查是否是列表类型且有数据
        if isinstance(recognition_result, list) and recognition_result:
            result_int = recognition_result[0]
            # 发布整数结果
            result_msg = Int32()
            result_msg.data = result_int
            self.result_publisher.publish(result_msg)
            
            # 根据识别结果发布相应的命令字符串
            command_str = self.get_command_string(result_int)
            if command_str:
                command_msg = String()
                command_msg.data = command_str
                self.command_publisher.publish(command_msg)
                
                self.get_logger().info(f'识别结果: {command_str} (ID: {result_int})')
        else:
            # 发布0表示无识别结果
            result_msg = Int32()
            result_msg.data = 0
            self.result_publisher.publish(result_msg)

    def get_command_string(self, result_id):
        """
        将识别结果ID转换为命令字符串
        """
        commands = {
            1: 'go',
            2: 'back',
            3: 'left',
            4: 'right',
            9: 'stop'
        }
        return commands.get(result_id, '')


def main():
    rclpy.init()
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()