cmds=(
	
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 launch robot_navigation2 robot_state_publisher.launch.py"
	"ros2 launch fast_lio mapping.launch.py"
	"ros2 launch serial_node serial_comm.launch.py "
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	"ros2 launch pcd2pgm pcd2pgm.launch.py"
	# "ros2 launch amcl_registration amcl.launch.py"
	"ros2 launch icp_registration icp.launch.py"	
	"ros2 launch robot_navigation2 navigation2.launch.py"
	"ros2 run diablo_ctrl diablo_ctrl_node"

	 )


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