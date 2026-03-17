from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='pose',
        executable='speech',
        name='voice_nav_node',
        output='screen',
    )

    return LaunchDescription([node])
