from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'base_link', '--child-frame-id' , 'livox_frame'
        #     ]
        # ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                # Prefer the body-frame cloud so scan generation does not depend on
                # a dynamic odom -> base transform before projecting into livox_frame.
                ('cloud_in', '/cloud_registered_body'),
                ('scan', '/scan')  # 输出 /scanner/scan
            ],
            parameters=[{
                # FAST_LIO publishes /cloud_registered_body in base_footprint.
                # Keeping the scan in the same frame avoids reintroducing the
                # static base_footprint -> livox_frame offset into /scan.
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.5,
                'queue_size': 20,
                # Keep the scan slice aligned with the documented map filtering band.
                'min_height': -0.09,
                'max_height': 0.35,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.3333,
                'range_min': 0.0,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
