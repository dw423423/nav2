#!/bin/bash

WORKSPACE_DIR="/home/hero/ros2_humble_2D_backup_omin"
cd "$WORKSPACE_DIR" || exit 1

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

cmds=(
    "ros2 launch robot_navigation2 robot_state_publisher.launch.py"
    "ros2 launch robot_navigation2 d435_scan.launch.py"
    "ros2 launch pose person_follower.launch.py"
    "ros2 run rqt_image_view rqt_image_view"
)

for cmd in "${cmds[@]}"
do
    echo "Current CMD: $cmd"
    gnome-terminal --tab -- bash -c "cd $WORKSPACE_DIR; source /opt/ros/humble/setup.bash; source install/setup.bash; $cmd; exec bash;"
    sleep 1.0
done
