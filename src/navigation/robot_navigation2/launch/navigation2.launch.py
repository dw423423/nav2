import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    my_rviz_dir = get_package_share_directory('robot_navigation2')
    default_rviz_config_path = os.path.join(
        my_rviz_dir, 'rviz', 'nav2.rviz')

    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    



    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),



        launch_ros.actions.Node(
            package='tf2_ros',
            executable='tf2_monitor',
            name='tf_monitor',
            arguments=['base_footprint', 'laser_link'],
            output='screen'
        ),


        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [fishbot_navigation2_dir, '/launch', '/custom_bringup', '/bringup_launch.py']),  # 修改为本地的bringup_launch.py
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'map': '/home/hero/ros2_humble_2D_backup_omin/map.yaml',                  
                'use_map_topic': 'true',
                'slam': 'true'
                
            }.items(),
        ),
        # launch_ros.actions.Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', default_rviz_config_path],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),


           launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),         
    ])




