import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    following_server_node = Node(
        package='opennav_following',
        executable='opennav_following',
        name='following_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_following',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['following_server']}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/hero/ros2_humble_2D_backup_omin/src/pose/config/following_params.yaml',
            description='Full path to the following server params file'
        ),
        following_server_node,
        lifecycle_manager_node,
    ])