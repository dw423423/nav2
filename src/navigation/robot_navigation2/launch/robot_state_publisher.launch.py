import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包目录
    robot_navigation2_dir = get_package_share_directory('robot_navigation2')
    
    # URDF文件路径
    urdf_file = os.path.join(robot_navigation2_dir, 'urdf', 'diff_drive_robot.urdf')

    # 检查URDF文件是否存在
    if not os.path.exists(urdf_file):
        print(f"❌ 错误: URDF文件不存在: {urdf_file}")
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # 读取URDF文件内容
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Launch参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),

        # 启动robot_state_publisher节点
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }],
            output='screen'
        ),
    ])