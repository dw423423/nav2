WORKSPACE_DIR="/home/hero/ros2_humble_2D_backup_omin"
cd $WORKSPACE_DIR

# 确保环境变量正确设置
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash

cmds=(
	
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"    #发布/livox/imu  /livox/lidar
	"ros2 launch robot_navigation2 robot_state_publisher.launch.py"
	"ros2 launch fast_lio mapping.launch.py"				#发布/cloud_effected	/cloud_registered	/cloud_registered_body
	# "ros2 launch serial_node serial_comm.launch.py "
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"   #局部代价地图 发布/scan
	# "ros2 launch octomap_server2 octomap_server_launch.py"
	# "ros2 launch pcd2pgm pcd2pgm.launch.py"
	

	"ros2 launch robot_navigation2 navigation2.launch.py"
	# "ros2 launch slam_toolbox localization_launch.py"
	# "ros2 launch slam_toolbox location_launch.py"
	# "ros2 launch slam_toolbox online_async_launch.py"

	# "ros2 launch icp_registration icp.launch.py"	
	# "ros2 launch amcl_registration amcl.launch.py"

	# "ros2 launch pose pose.launch.py"

	 )


#"ros2 launch amcl_registration amcl.launch.py"
# "ros2 launch rc_nav_bringup bringup_real.launch.py     world:=YOUR_WORLD_NAME     mode:=mapping      lio:=fastlio     lio_rviz:=False     nav_rviz:=True use_sim_time:=False"
#"ros2 launch octomap_server2 octomap_server_launch.py"
# "ros2 run tf2_ros static_transform_publisher 0 0 0 -45 0 0 map odom"


# for cmd in "${cmds[@]}"
# do
# 	echo Current CMD : "$cmd"
# 	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
# 	sleep 2
# done


for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	# 使用更可靠的终端启动方式
	gnome-terminal --tab -- bash -c "cd $WORKSPACE_DIR; source /opt/ros/humble/setup.bash; source install/setup.bash; $cmd; exec bash;"
	sleep 1.0
done