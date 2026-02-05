#!/usr/bin/env python3
import can
import struct
import time
import os
import subprocess
import logging

class CANDriver:
    def __init__(self, channel='can2', bitrate=500000, setup_can_interface=True):
        """
        初始化CAN驱动，可自动配置CAN接口
        :param channel: CAN接口名称
        :param bitrate: CAN波特率
        :param setup_can_interface: 是否自动配置CAN接口
        """
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        
        # 配置日志
        self.logger = logging.getLogger(__name__)
        # 设置日志级别为INFO以显示详细信息
        self.logger.setLevel(logging.DEBUG)
        
        # 如果启用自动配置，则设置CAN接口
        if setup_can_interface:
            self.setup_can_interface()
        
        # 初始化CAN总线
        self.init_can_bus()
    
    def setup_can_interface(self):
        """
        配置CAN接口（需要root权限）
        使用socketcan和ip命令配置CAN接口
        """
        try:
            self.logger.info(f"配置CAN接口 {self.channel}，波特率 {self.bitrate}")
            
            # 检查是否已加载can模块
            modules = subprocess.run(['lsmod'], capture_output=True, text=True).stdout
            if 'can' not in modules or 'can_raw' not in modules:
                self.logger.info("加载CAN内核模块...")
                subprocess.run(['sudo', 'modprobe', 'can'], check=True)
                subprocess.run(['sudo', 'modprobe', 'can_raw'], check=True)
                subprocess.run(['sudo', 'modprobe', 'vcan'], check=True)
            
            # 检查接口是否存在
            result = subprocess.run(['ip', 'link', 'show', self.channel], 
                                  capture_output=True, text=True)
            
            if result.returncode != 0:
                # 接口不存在，创建虚拟CAN接口（测试用）
                self.logger.warning(f"CAN接口 {self.channel} 不存在，创建虚拟接口用于测试")
                subprocess.run(['sudo', 'ip', 'link', 'add', 'dev', self.channel, 
                              'type', 'vcan'], check=True)
            
            # 设置CAN接口参数
            commands = [
                ['sudo', 'ip', 'link', 'set', self.channel, 'down'],
                ['sudo', 'ip', 'link', 'set', self.channel, 'type', 'can', 
                 'bitrate', str(self.bitrate)],
                ['sudo', 'ip', 'link', 'set', self.channel, 'up']
            ]
            
            for cmd in commands:
                try:
                    subprocess.run(cmd, check=True, timeout=5)
                    self.logger.debug(f"成功执行命令: {' '.join(cmd)}")
                except subprocess.CalledProcessError as e:
                    self.logger.warning(f"命令执行失败: {' '.join(cmd)}: {e}")
                except subprocess.TimeoutExpired:
                    self.logger.warning(f"命令超时: {' '.join(cmd)}")
            
            # 等待接口启动
            time.sleep(1)
            
            # 验证接口状态
            result = subprocess.run(['ip', '-details', 'link', 'show', self.channel],
                                  capture_output=True, text=True)
            
            if result.returncode == 0 and 'UP' in result.stdout and 'LOWER_UP' in result.stdout:
                self.logger.info(f"CAN接口 {self.channel} 配置成功")
                return True
            else:
                self.logger.error(f"CAN接口 {self.channel} 配置失败")
                self.logger.error(f"接口状态: {result.stdout}")
                return False
                
        except Exception as e:
            self.logger.error(f"配置CAN接口时发生错误: {e}")
            return False
    
    def init_can_bus(self):
        """
        初始化CAN总线连接
        """
        try:
            # 检查接口是否存在
            if not os.path.exists(f'/sys/class/net/{self.channel}'):
                self.logger.error(f"CAN接口 {self.channel} 不存在")
                return False
            
            # 对于使用cansend的场景，我们不需要保持bus连接
            # 但仍然记录初始化成功
            self.logger.info(f"CAN接口准备就绪: {self.channel} @ {self.bitrate}bps")
            return True
            
        except Exception as e:
            self.logger.error(f"初始化CAN接口时发生错误: {e}")
            return False
    
    def send_control_command(self, address, motor1_speed, motor2_speed, control_mode=0x03):
        """
        发送双电机控制指令 (CAN协议)
        :param address: 驱动器地址 (0x01, 0x02)
        :param motor1_speed: 电机1速度 (mm/s), 范围: -1800 到 1800
        :param motor2_speed: 电机2速度 (mm/s), 范围: -1800 到 1800
        :param control_mode: 控制模式 (0x03: 分别控制两个电机)
        """
        try:
            # 限制速度范围
            motor1_speed = max(min(motor1_speed, 1800), -1800)
            motor2_speed = max(min(motor2_speed, 1800), -1800)
            
            # 转换为int16
            motor1_speed_int = int(motor1_speed)
            motor2_speed_int = int(motor2_speed)
            
            # 第一包数据 (0xA1)
            packet1 = bytearray()
            packet1.append(0xA1)          # 第一包标识
            packet1.append(0xAA)          # 包头1
            packet1.append(0x55)          # 包头2
            packet1.append(address)       # 驱动器地址
            packet1.append(control_mode)  # 控制模式
            packet1.append(0x04)          # 数据长度
            
            # 电机2速度 (高8位，低8位) - 注意是第一包
            packet1.append((motor2_speed_int >> 8) & 0xFF)
            packet1.append(motor2_speed_int & 0xFF)
            
            # 填充到8字节
            while len(packet1) < 8:
                packet1.append(0x00)
            
            # 第二包数据 (0xA2)
            packet2 = bytearray()
            packet2.append(0xA2)          # 第二包标识
            
            # 电机1速度 (高8位，低8位)
            packet2.append((motor1_speed_int >> 8) & 0xFF)
            packet2.append(motor1_speed_int & 0xFF)
            
            # 功能配置位: 进入运行状态 (01)
            packet2.append(0x02)          # 功能配置位: Bit0=0(不清零), Bit2&Bit1=01(运行状态)
            packet2.append(0x00)          # 保留
            
            # 计算CRC (简单求和校验)
            crc = self.calculate_crc(packet1, packet2)
            packet2.append(crc)
            packet2.append(0xCC)          # 包尾
            
            # 填充到8字节
            while len(packet2) < 8:
                packet2.append(0x00)
            
            # 将字节数组转换为十六进制字符串，用于cansend命令
            packet1_hex = ''.join(format(byte, '02X') for byte in packet1)
            packet2_hex = ''.join(format(byte, '02X') for byte in packet2)
            
            # 构建cansend命令
            can_id = format(address, '03X')  # 将地址转换为3位十六进制ID
            
            # 发送消息前记录详细信息
            self.logger.info(f"准备发送两帧CAN消息到地址 {hex(address)}:")
            self.logger.info(f"  消息1 ID: {can_id}, Data: {packet1_hex}")
            self.logger.info(f"  消息2 ID: {can_id}, Data: {packet2_hex}")
            
            # 使用cansend发送第一帧
            cmd1 = f"cansend {self.channel} {can_id}#{packet1_hex}"
            try:
                result1 = subprocess.run(cmd1.split(), check=True, capture_output=True, text=True)
                self.logger.debug(f"已发送第一帧到地址 {hex(address)} 通过 cansend")
            except subprocess.CalledProcessError as e:
                self.logger.error(f"发送第一帧失败: {e}")
                return False
            
            # 使用cansend发送第二帧
            cmd2 = f"cansend {self.channel} {can_id}#{packet2_hex}"
            try:
                result2 = subprocess.run(cmd2.split(), check=True, capture_output=True, text=True)
                self.logger.debug(f"已发送第二帧到地址 {hex(address)} 通过 cansend")
            except subprocess.CalledProcessError as e:
                self.logger.error(f"发送第二帧失败: {e}")
                return False
            
            # 成功发送日志
            self.logger.info(
                f"成功发送控制指令: 地址={hex(address)}, "
                f"电机1={motor1_speed_int}mm/s, 电机2={motor2_speed_int}mm/s"
            )
            return True
            
        except Exception as e:
            self.logger.error(f"发送控制指令时发生错误: {e}")
            return False
    
    def calculate_crc(self, packet1, packet2):
        """
        计算CRC校验和
        :param packet1: 第一包数据
        :param packet2: 第二包数据 (前6个字节用于CRC计算)
        :return: CRC校验字节
        """
        crc = 0
        
        # 排除包标识字节(0xA1, 0xA2)后的数据
        for byte in packet1[1:]:
            crc = (crc + byte) & 0xFF
        
        # 第二包的前6个字节 (排除包标识)
        for byte in packet2[1:6]:
            crc = (crc + byte) & 0xFF
        
        return crc
    
    def send_speed_command(self, address, motor1_speed_mmps, motor2_speed_mmps):
        """
        简化的速度发送接口
        :param address: 驱动器地址
        :param motor1_speed_mmps: 电机1速度 (mm/s)
        :param motor2_speed_mmps: 电机2速度 (mm/s)
        """
        return self.send_control_command(
            address=address,
            motor1_speed=motor1_speed_mmps,
            motor2_speed=motor2_speed_mmps,
            control_mode=0x03
        )
    
    def test_connection(self):
        """
        测试CAN连接
        """
        try:
            # 发送一个测试消息
            test_data = "0000000000000000"
            can_id = "000"
            cmd = f"cansend {self.channel} {can_id}#{test_data}"
            result = subprocess.run(cmd.split(), check=True, capture_output=True, text=True)
            return True
        except Exception as e:
            self.logger.error(f"测试连接失败: {e}")
            return False
    
    def close(self):
        """关闭CAN总线"""
        try:
            # 发送停止命令到所有已知地址
            for address in [0x01, 0x02]:
                self.send_control_command(
                    address=address,
                    motor1_speed=0,
                    motor2_speed=0,
                    control_mode=0x03
                )
        except Exception as e:
            self.logger.error(f"发送停止命令失败: {e}")
        
        self.logger.info(f"CAN接口 {self.channel} 已关闭")