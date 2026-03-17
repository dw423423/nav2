from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='pose',
        executable='2d',
        name='auto_relocalizer',
        output='screen',
    )

    return LaunchDescription([node])
