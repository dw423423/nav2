import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    depth_profile = LaunchConfiguration('depth_profile')
    color_profile = LaunchConfiguration('color_profile')
    scan_height = LaunchConfiguration('scan_height')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    output_frame = LaunchConfiguration('output_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'depth_profile',
            default_value='424x240x30',
            description='D435 depth stream profile widthxheightxfps'
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value='424x240x30',
            description='D435 color stream profile widthxheightxfps'
        ),
        DeclareLaunchArgument(
            'scan_height',
            default_value='20',
            description='Number of depth image rows projected into /d435_scan'
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='0.20',
            description='Minimum valid range for /d435_scan in meters'
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='3.0',
            description='Maximum valid range for /d435_scan in meters'
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='camera_depth_frame',
            description='LaserScan frame_id published by depthimage_to_laserscan'
        ),
        # Mirror the manually verified command line exactly. This avoids the
        # startup difference seen when wrapping rs_launch.py via IncludeLaunchDescription.
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'initial_reset:=true',
                'pointcloud.enable:=false',
                'enable_sync:=true',
                'align_depth.enable:=true',
                'enable_color:=true',
                'enable_depth:=true',
                'enable_infra1:=false',
                'enable_infra2:=false',
                'enable_gyro:=false',
                'enable_accel:=false',
                ['depth_module.depth_profile:=', depth_profile],
                ['rgb_camera.color_profile:=', color_profile],
            ],
            output='screen',
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[{
                'use_sim_time': use_sim_time,
                'scan_height': scan_height,
                'range_min': range_min,
                'range_max': range_max,
                'output_frame': output_frame,
            }],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/d435_scan'),
            ],
            output='screen',
        ),
    ])
