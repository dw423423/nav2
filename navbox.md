# navbox.sh 启动链路复盘

## 1. 概括
 该导航方案使用了slam_toolbox，使用slam_toolbox导航时需要使用slam_toolbox建图，也已经写好脚本 `mapbox_sh`
 启动该脚本还需启动 ` ros2 run omni_drive omni_drive_node`（打开底盘）


## 2. 启动链路总览

`navbox.sh` 当前实际启动的模块如下：

1. `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
2. `ros2 launch robot_navigation2 robot_state_publisher.launch.py`
3. `ros2 launch robot_navigation2 d435_scan.launch.py`
4. `ros2 launch fast_lio mapping.launch.py`
5. `ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py`
6. `ros2 launch robot_navigation2 navigation2.launch.py`
7. `ros2 launch pose 2d_pose.launch.py`
8. `ros2 launch pose speech_recognition.launch.py`

如果把它抽象成一条数据流，可以理解为：

传感器输入 -> 机器人 TF 建立 -> LIO 状态估计 -> 点云/深度转 LaserScan -> Nav2 导航 -> 自动重定位与语音任务入口

整体不是单一导航 demo，而是一套具备多传感器融合、定位建图、导航执行和人机交互能力的机器人系统。

---

## 3. 各 launch 文件功能与相互关系

### 3.1 `msg_MID360_launch.py`：Livox MID360 传感器入口

这个 launch 的作用是启动 Livox MID360 雷达驱动节点。它负责把硬件采集到的激光雷达点云和 IMU 数据发布到 ROS 2 系统中。

从整条链路来看，它是最前端的数据入口。后续的 `FAST_LIO` 需要依赖它提供的雷达和惯导数据才能进行融合定位和建图，因此它是整个感知链的起点。

复盘时可以把它理解为：

- 它解决的是“机器人如何获得外界环境观测”的问题
- 它为后续定位建图模块提供原始输入
- 一旦这一层异常，后面的 LIO、导航和任务执行都会失去基础

---

### 3.2 `robot_state_publisher.launch.py`：机器人本体 TF 框架

这个 launch 会读取机器人 URDF，并启动 `robot_state_publisher` 节点，持续发布机器人本体的 TF 关系。

它的核心作用不是感知环境，而是回答“机器人自身各部件之间是什么坐标关系”。例如底盘、雷达、相机等传感器相对于机器人底座的姿态和位置，都依赖这部分 TF 定义。

它和前后模块的关系可以这样理解：

- 对前面传感器模块而言，它提供统一的坐标系参考
- 对后面的导航与建图模块而言，它保证不同来源的数据能被投到一致的坐标框架中
- 对整个系统而言，它是连接传感器数据和机器人模型的基础

因此，这个模块虽然不直接“生成地图”或“控制运动”，但它决定了系统坐标关系是否自洽，是整个导航系统能够稳定工作的前提。

---

### 3.3 `d435_scan.launch.py`：深度相机局部障碍感知


这个 launch 做了两件事：

1. 启动 `realsense2_camera`，打开 D435 的深度和彩色流
2. 启动 `depthimage_to_laserscan`，把深度图转换为 `/d435_scan`

这说明 D435 在项目中的定位并不是主建图传感器，而是近距离局部障碍感知的补充来源。

它与其他模块之间的关系非常明确：

- 它不承担全局定位
- 它也不负责主地图构建
- 它主要为局部代价地图提供额外的障碍信息，尤其适合补充机器人前方近距离区域的感知

从工程设计上看，这是一种典型的“补盲思路”：  
激光雷达负责较大范围环境感知，深度相机补充近距离和低矮障碍信息。

---

### 3.4 `mapping.launch.py`：FAST_LIO2 状态估计与点云配准核心


这个 launch 启动 `fastlio_mapping` 节点，是整套系统中的状态估计核心。

它的职责可以概括为：

- 消费 Livox 雷达和 IMU 数据
- 进行激光惯导融合
- 输出机器人位姿估计结果
- 发布配准后的点云，如 `/cloud_registered`、`/cloud_registered_body`

从项目整体架构看，`FAST_LIO` 并不只是“把点云发出来”，而是在原始传感器数据和导航层之间，增加了一层能够稳定提供位姿和空间结构的中间层。

也正因为有这一层，导航模块拿到的环境信息不是原始点云，而是已经经过配准和时空对齐的结果。这显著提升了后续环境建模和导航使用的稳定性。

---

### 3.5 `pointcloud_to_laserscan_launch.py`：将 LIO 点云转成 2D 导航可用激光


这个 launch 的作用，是把 `FAST_LIO` 输出的 `/cloud_registered_body` 转换为二维激光扫描 `/scan`。

这里体现出项目中的一个重要设计选择：  
系统不是直接拿原始雷达输出做二维导航，而是先经过 `FAST_LIO` 融合与配准，再从配准后的三维点云中提取二维切片，供 Nav2 使用。

这样做的意义在于：

- 先利用 LIO 获得更稳定的空间表达
- 再把三维环境信息降维成导航层更易消费的 `LaserScan`
- 让 Nav2 在保留成熟二维导航机制的同时，也能受益于三维传感器和 LIO 的结果

因此，它实际上扮演的是“感知表达转换层”的角色，是从 3D 感知系统到 2D 导航系统的桥梁。

---

### 3.6 `navigation2.launch.py`：导航系统总入口


这个 launch 是导航子系统的入口。它不是单纯启动某一个节点，而是把 Nav2 所需的规划、控制、行为树、代价地图等模块整体带起来。


同时使用了：

- 地图文件 `map.yaml`
- Nav2 参数文件 `nav2_params.yaml`
- `slam:=true`

这意味着这个项目里的导航系统并不是一个最小配置版本，而是经过定制集成的导航框架。


可以发现局部代价地图中的 `obstacle_layer` 同时订阅了：

- `/scan`
- `/d435_scan`

这说明 Nav2 的局部避障信息来自两个来源：

- `/scan`：由 Livox + FAST_LIO + pointcloud_to_laserscan 链路产生
- `/d435_scan`：由 D435 深度图转换产生
---

### 3.7 `2d_pose.launch.py`：自动重定位辅助节点


这个 launch 启动的是 `auto_relocalizer` 节点。它会周期性读取 `map -> base_footprint` 的 TF，然后持续向 `/initialpose` 发布当前位置。

`/initialpose` 往往需要人工在 RViz 中指定；而这里通过程序自动发布当前位姿，等于是给定位系统持续提供参考。

它与导航模块之间的联系很直接：

- Nav2 依赖合理的初始位姿来建立定位状态
- `auto_relocalizer` 通过自动发布 `/initialpose`，降低人工干预成本
- 它提升的不是功能上限，而是系统运行过程中的稳定性和便利性



---

### 3.8 `speech_recognition.launch.py`：语音触发导航任务


这个 launch 启动的是 `voice_nav_node`。它直接继承 `BasicNavigator`，语音命令直接转化为导航目标。

它的工作流程可以概括为：

1. 通过 I2C 从语音识别模块读取命令 ID
2. 将命令映射为预设目标点，如前进、返回、左侧点、右侧点
3. 把这些目标点转换为 `map` 坐标系下的 `PoseStamped`
4. 调用 Nav2 的 `goToPose()` 发起导航任务
5. 如果收到 `stop` 命令，则取消当前导航


需要注意的是：

- Nav2 解决的是“如何去目标点”
- `voice_nav_node` 解决的是“目标点由谁来下达”


---

## 4. 模块之间的联系：从底层感知到上层任务


1. Livox MID360 发布激光雷达和 IMU 数据
2. `robot_state_publisher` 建立机器人本体 TF 关系
3. `FAST_LIO` 融合雷达与 IMU，生成位姿与配准点云
4. `pointcloud_to_laserscan` 将配准点云转成 `/scan`
5. D435 生成 `/d435_scan` 作为近距离障碍补充
6. Nav2 同时使用 `/scan` 和 `/d435_scan` 构建代价地图并执行路径规划与控制
7. `auto_relocalizer` 周期性向 `/initialpose` 发布位姿，帮助定位稳定
8. `voice_nav_node` 将语音命令转成目标点，驱动 Nav2 执行任务

从这条链路可以看出，系统整体采用了明确的分层思路：

- 感知层：Livox、D435
- 状态估计层：FAST_LIO
- 表达转换层：pointcloud_to_laserscan、depthimage_to_laserscan
- 导航决策层：Nav2
- 任务交互层：自动重定位、语音导航
