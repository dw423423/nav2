# import os
# import launch
# import launch_ros
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# #因为
# def generate_launch_description():
#     fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

#     use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
#     nav2_param_path = launch.substitutions.LaunchConfiguration(
#         'params_file',
#         default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

#     return launch.LaunchDescription([
#         launch.actions.DeclareLaunchArgument(
#             'use_sim_time',
#             default_value=use_sim_time,
#             description='Use simulation (Gazebo) clock if true'
#         ),
#         launch.actions.DeclareLaunchArgument(
#             'params_file',
#             default_value=nav2_param_path,
#             description='Full path to param file to load'
#         ),


#         launch.actions.IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
#             launch_arguments={
#                 'use_sim_time': use_sim_time,
#                 'params_file': nav2_param_path,
#                 'map': '',                  
#                 'use_map_topic': 'true'     
#             }.items(),
#         ),

#         launch_ros.actions.Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             parameters=[{'use_sim_time': use_sim_time}],
#             output='screen'),
#     ])


import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # URDF文件路径
    urdf_file = os.path.join(fishbot_navigation2_dir, 'urdf', 'diff_drive_robot.urdf')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    # 读取URDF文件内容
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()


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
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'map': '',                  
                'use_map_topic': 'true'     
            }.items(),
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
