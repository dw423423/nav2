
cmds=(  "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
		"ros2 launch robot_navigation2 robot_state_publisher.launch.py"
		"ros2 launch fast_lio mapping.launch.py"
		# "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py" 
		"ros2 launch octomap_server2 octomap_server_launch.py"
		# "ros2 launch pcd2pgm pcd2pgm.launch.py"
	 )

#"ros2 launch pcd2pgm pcd2pgm.launch.py"

# "ros2 launch octomap_server2 octomap_server_launch.py"
for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	# 使用更可靠的终端启动方式
	gnome-terminal --tab -- bash -c "cd $WORKSPACE_DIR; source /opt/ros/humble/setup.bash; source install/setup.bash; $cmd; exec bash;"
	sleep 1.0
done