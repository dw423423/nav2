import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pose_share_dir = get_package_share_directory('pose')
    default_params_file = os.path.join(
        pose_share_dir,
        'config',
        'following_params.yaml',
    )
    params_file = LaunchConfiguration('params_file')

    node = ExecuteProcess(
        cmd=[
            'python3',
            '/home/hero/ros2_humble_2D_backup_omin/src/pose/pose/person_follower_node.py',
            '--ros-args',
            '-r', '__node:=person_follower_node',
            '--params-file', params_file,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='YAML file containing person follower ROS parameters',
        ),
        node,
    ])
