cmds=(
	
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"    #发布/livox/imu  /livox/lidar
	"ros2 launch robot_navigation2 robot_state_publisher.launch.py"
	"ros2 launch fast_lio mapping.launch.py"				#发布/cloud_effected	/cloud_registered	/cloud_registered_body
	"ros2 launch serial_node serial_comm.launch.py "
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"   #局部代价地图 发布/scan
	# "ros2 launch octomap_server2 octomap_server_launch.py"
	# "ros2 launch pcd2pgm pcd2pgm.launch.py"
	

	"ros2 launch robot_navigation2 navigation2.launch.py"
	"ros2 launch slam_toolbox localization_launch.py"
	# "ros2 launch slam_toolbox location_launch.py"
	# "ros2 launch slam_toolbox online_async_launch.py"

	# "ros2 launch icp_registration icp.launch.py"	
	# "ros2 launch amcl_registration amcl.launch.py"
	"ros2 run diablo_ctrl diablo_ctrl_node"
	# "ros2 launch pose pose.launch.py"

	 )

# cmds=(
# # ... existing code ...
# 	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"    #发布/livox/imu  /livox/lidar
# 	"ros2 launch robot_navigation2 robot_state_publisher.launch.py"
# 	# 删除建图，改为地图服务器和SLAM_Toolbox定位
# 	"ros2 launch serial_node serial_comm.launch.py "
# 	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"   #局部代价地图 发布/scan
# 	"ros2 launch octomap_server2 octomap_server_launch.py"
# 	# 使用现有2D地图，启动地图服务器
# 	"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/hero/ros2_humble_2D_backup/map_2d.yaml"
# 	# 使用SLAM_Toolbox进行定位，提供map->odom变换
# 	"ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/hero/ros2_humble_2D_backup/src/slam_toolbox/config/mapper_params_localization.yaml"
# 	# 将原来的pcd转2D地图的节点替换为使用已有地图
# 	"ros2 launch icp_registration icp.launch.py"	
# 	"ros2 launch robot_navigation2 navigation2.launch.py"
# 	"ros2 run diablo_ctrl diablo_ctrl_node"
# 	# "ros2 launch pose pose.launch.py"
# # ... existing code ...
# 	)


#"ros2 launch amcl_registration amcl.launch.py"
# "ros2 launch rc_nav_bringup bringup_real.launch.py     world:=YOUR_WORLD_NAME     mode:=mapping      lio:=fastlio     lio_rviz:=False     nav_rviz:=True use_sim_time:=False"
#"ros2 launch octomap_server2 octomap_server_launch.py"
# "ros2 run tf2_ros static_transform_publisher 0 0 0 -45 0 0 map odom"
for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 2
done