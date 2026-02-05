WORKSPACE_DIR="/home/hero/ros2_humble_2D_backup_omin"
cd $WORKSPACE_DIR

# 确保环境变量正确设置
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash

cmds=(
	
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"    #发布/livox/imu  /livox/lidar
	"ros2 launch robot_navigation2 robot_state_publisher.launch.py"
	"ros2 launch fast_lio mapping.launch.py"				#发布/cloud_effected	/cloud_registered	/cloud_registered_body

	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"   #局部代价地图 发布/scan


	"ros2 launch slam_toolbox online_async_launch.py"



	 )




for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	# 使用更可靠的终端启动方式
	gnome-terminal --tab -- bash -c "cd $WORKSPACE_DIR; source /opt/ros/humble/setup.bash; source install/setup.bash; $cmd; exec bash;"
	sleep 1.0
done