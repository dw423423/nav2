
#!/bin/bash

# 切换到工作空间根目录
cd "$(dirname "$(dirname "$0")")"

# 编译 文件
colcon build --symlink-install --cmake-args   -DROS_EDITION=ROS2   -DHUMBLE_ROS=humble  

