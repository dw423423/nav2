
#cmds=(  "ros2 run nav2_map_server map_saver_cli -t /projected_map -f /home/hsh/workspase/hsh_ws/hsh_ros2_humble_main/map")
cmds=(  "ros2 run nav2_map_server map_saver_cli -t /projected_map -f $(pwd)/map_2d")

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2 
done



