#!/usr/bin/env python3
# coding=utf-8

import rclpy
import smbus
import time
import tf_transformations

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String


I2C_ADDR = 0x34
ASR_RESULT_ADDR = 100   # 0x64


class ASRModule:
    def __init__(self, address, bus=7):
        self.bus = smbus.SMBus(bus)
        self.address = address

    def wire_read_data_array(self, reg, length):
        try:
            return self.bus.read_i2c_block_data(self.address, reg, length)
        except IOError:
            return []

    def rec_recognition(self):
        result = self.wire_read_data_array(ASR_RESULT_ADDR, 1)
        if result:
            return result
        return [0]


class VoiceNavNode(BasicNavigator):
    def __init__(self, node_name="voice_nav_node"):
        super().__init__(node_name)

        self.declare_parameter('i2c_address', I2C_ADDR)
        self.declare_parameter('i2c_bus', 7)

        self.declare_parameter('point_go',   [-18.0, 3.15, 0.0])
        self.declare_parameter('point_back', [-8.63,   1.98,  0.0])
        self.declare_parameter('point_left', [-11.4, 20.0,  1.57])
        self.declare_parameter('point_right',[0.0,   -1.0, -1.57])

        self.i2c_address = self.get_parameter('i2c_address').value
        self.i2c_bus = self.get_parameter('i2c_bus').value

        self.point_go = self.get_parameter('point_go').value
        self.point_back = self.get_parameter('point_back').value
        self.point_left = self.get_parameter('point_left').value
        self.point_right = self.get_parameter('point_right').value

        self.asr_module = ASRModule(self.i2c_address, self.i2c_bus)

        self.result_publisher = self.create_publisher(Int32, 'asr_result', 10)
        self.command_publisher = self.create_publisher(String, 'voice_command', 10)

        self.last_cmd_id = 0
        self.last_cmd_time = 0.0
        self.same_cmd_ignore_sec = 2.0

        self.get_logger().info(
            f'语音导航节点已启动, I2C地址: 0x{self.i2c_address:02X}, 总线: {self.i2c_bus}'
        )

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def get_command_string(self, result_id):
        commands = {
            128: 'go',
            129: 'back',
            130: 'left',
            131: 'right',
            132: 'stop'
        }
        return commands.get(result_id, '')

    def command_to_target(self, cmd_id):
        mapping = {
            128: self.point_go,
            129: self.point_back,
            130: self.point_left,
            131: self.point_right,
        }
        return mapping.get(cmd_id, None)

    def publish_asr_info(self, result_int, command_str=''):
        result_msg = Int32()
        result_msg.data = result_int
        self.result_publisher.publish(result_msg)

        if command_str:
            command_msg = String()
            command_msg.data = command_str
            self.command_publisher.publish(command_msg)

    def nav_to_pose_blocking(self, target_pose):
        self.goToPose(target_pose)

        while rclpy.ok() and not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is not None and hasattr(feedback, 'distance_remaining'):
                if feedback.distance_remaining is not None:
                    self.get_logger().info(f'剩余距离: {feedback.distance_remaining:.3f}')
            time.sleep(0.5)

        result = self.getResult()
        self.get_logger().info(f'导航结果: {result}')
        return result

    def try_read_command_once(self):
        recognition_result = self.asr_module.rec_recognition()
        if not (isinstance(recognition_result, list) and recognition_result):
            return 0

        result_int = recognition_result[0]
        self.get_logger().info(f'I2C原始返回: {recognition_result}, 当前识别ID: {result_int}')
        self.publish_asr_info(result_int)

        if result_int == 0:
            return 0

        command_str = self.get_command_string(result_int)
        if not command_str:
            self.get_logger().info(f'收到未定义命令ID: {result_int}')
            return 0

        self.publish_asr_info(result_int, command_str)

        now = time.time()
        if result_int == self.last_cmd_id and (now - self.last_cmd_time) < self.same_cmd_ignore_sec:
            self.get_logger().info(f'忽略重复命令: {command_str} (ID: {result_int})')
            return 0

        self.last_cmd_id = result_int
        self.last_cmd_time = now

        self.get_logger().info(f'识别到命令: {command_str} (ID: {result_int})')
        return result_int


def main():
    rclpy.init()
    node = VoiceNavNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            cmd_id = node.try_read_command_once()
            if cmd_id == 0:
                time.sleep(0.1)
                continue

            if cmd_id == 132:
                try:
                    node.cancelTask()
                    node.get_logger().warn('收到 stop 命令，已取消当前导航任务')
                except Exception as e:
                    node.get_logger().warn(f'取消任务失败: {e}')
                time.sleep(0.2)
                continue

            target = node.command_to_target(cmd_id)
            command_str = node.get_command_string(cmd_id)

            if target is None:
                node.get_logger().warn(f'命令 {command_str} 没有配置目标点')
                time.sleep(0.2)
                continue

            target_pose = node.get_pose_by_xyyaw(target[0], target[1], target[2])

            node.get_logger().info(
                f'发送导航目标: {command_str} -> ({target[0]}, {target[1]}, {target[2]})'
            )
            node.get_logger().info(
                f'target_pose.position=({target_pose.pose.position.x}, {target_pose.pose.position.y}), '
                f'orientation.z={target_pose.pose.orientation.z}, orientation.w={target_pose.pose.orientation.w}'
            )

            try:
                result = node.nav_to_pose_blocking(target_pose)
                node.get_logger().info(f'本次语音导航结束: {result}')
            except Exception as e:
                node.get_logger().error(f'语音导航执行失败: {e}')

            time.sleep(0.5)

    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()