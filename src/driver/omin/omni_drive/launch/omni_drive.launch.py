#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件的路径
    config_file = os.path.join(
        get_package_share_directory('omni_drive'),
        'config',
        'omni_drive.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='omni_drive',
            executable='omni_drive_node',
            name='omni_drive_node',
            output='screen',
            parameters=[config_file],
            # 以root权限运行（需要配置sudo免密码）
            # prefix='sudo -E',
            emulate_tty=True,
        )
    ])