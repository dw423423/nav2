# nav.sh 启动链路复盘

## 1. 脚本定位

[`nav.sh`](/home/hero/ros2_humble_2D_backup_omin/nav.sh) 是本项目另一条非常重要的运行入口。和偏向“完整业务运行链”的 `navbox.sh` 不同，`nav.sh` 更像是一条围绕建图成果利用、地图配准定位、Nav2 导航执行以及底盘驱动控制展开的主链路。

如果从复盘角度理解，这个脚本回答的不是“机器人如何完成感知 + 交互的所有能力”，而是“在已有雷达建图基础上，机器人如何完成地图生成、定位对齐、导航规划和真实运动执行”。

因此，`nav.sh` 的系统定位可以概括为：

- 以 Livox + FAST_LIO 为底层感知与状态估计基础
- 以 `pcd2pgm` 和 `icp_registration` 为地图生成与定位配准桥梁
- 以 Nav2 为导航决策核心
- 以全向底盘驱动节点为最终执行出口

它体现的是一条更偏“从地图到导航落地”的工程主线。

---

## 2. 启动链路总览

`nav.sh` 当前实际启动的模块如下：

1. `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
2. `ros2 launch robot_navigation2 robot_state_publisher.launch.py`
3. `ros2 launch fast_lio mapping.launch.py`
4. `ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py`
5. `ros2 launch pcd2pgm pcd2pgm.launch.py`
6. `ros2 launch icp_registration icp.launch.py`
7. `ros2 launch robot_navigation2 navigation2.launch.py`
8. `ros2 run omni_drive omni_drive_node`

把这条链路抽象成系统数据流，可以理解为：

雷达/IMU输入 -> 机器人TF建立 -> LIO位姿与点云 -> 点云转二维激光 -> PCD转栅格地图 -> ICP提供 map-odom 配准 -> Nav2规划与控制 -> 底盘执行 `cmd_vel`

从这一点可以看出，`nav.sh` 的重点不在多模态交互，而在地图落地、定位闭环和运动控制闭环。

---

## 3. 各模块功能与相互关系

### 3.1 `msg_MID360_launch.py`：Livox MID360 传感器输入源

文件路径：
[`src/driver/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/driver/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py)

这个 launch 启动 Livox MID360 雷达驱动节点，负责向 ROS 2 发布激光雷达点云和 IMU 数据。

在 `nav.sh` 这条链路中，它仍然是最前端的输入源。后续的 `FAST_LIO` 需要依赖它提供的原始观测数据来完成激光惯导融合，因此它是整条定位与导航链路的起点。

复盘时可以把它理解为：

- 它提供原始环境观测数据
- 它决定后续 LIO 是否有稳定输入
- 它一旦异常，建图、定位、导航都会受到连锁影响

---

### 3.2 `robot_state_publisher.launch.py`：机器人本体坐标框架建立

文件路径：
[`src/navigation/robot_navigation2/launch/robot_state_publisher.launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/navigation/robot_navigation2/launch/robot_state_publisher.launch.py)

这个 launch 读取机器人 URDF，并通过 `robot_state_publisher` 持续发布机器人各部件之间的 TF 关系。

在这条链路里，它承担的职责非常基础但不可替代。因为后续的雷达坐标、底盘坐标、激光坐标、里程计坐标，最终都需要落在一个统一的 TF 体系下，导航系统和 ICP 配准才能成立。

换句话说，它解决的是“系统内部谁相对谁在哪里”的问题，而不是“机器人现在在地图哪里”的问题。

它与后续模块的关系主要体现在：

- 给 `FAST_LIO` 和 ICP 提供一致的传感器/机体坐标参考
- 给 Nav2 提供统一的机器人基坐标系基础
- 保证控制和感知可以在同一个 TF 语义下工作

---

### 3.3 `mapping.launch.py`：FAST_LIO 位姿估计与配准点云输出

文件路径：
[`src/lio/FAST_LIO/launch/mapping.launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/lio/FAST_LIO/launch/mapping.launch.py)

这个 launch 启动 `fastlio_mapping` 节点，是整条链路中的状态估计核心。

它的主要职责包括：

- 融合 Livox 雷达与 IMU 数据
- 输出机器人位姿估计结果
- 发布配准后的点云，如 `/cloud_registered`、`/cloud_registered_body`

这一层的重要性在于，它把原始传感器数据转化成了带时空一致性的环境表达。也就是说，后续系统使用的并不是“裸点云”，而是已经经过运动补偿、配准和融合处理后的结果。

这使得 `nav.sh` 后面的两个模块都能直接受益：

- `pointcloud_to_laserscan` 可以从更稳定的点云中提取二维扫描
- `pcd2pgm` 和 `icp_registration` 可以建立在更可用的环境表达之上

所以从复盘角度看，`FAST_LIO` 是连接传感器层和地图/导航层的核心中间层。

---

### 3.4 `pointcloud_to_laserscan_launch.py`：从三维点云到二维导航观测

文件路径：
[`src/mapper/pointcloud_to_laserscan/launch/pointcloud_to_laserscan_launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/mapper/pointcloud_to_laserscan/launch/pointcloud_to_laserscan_launch.py)

这个 launch 将 `FAST_LIO` 输出的 `/cloud_registered_body` 转换为 `/scan`。

这一步在 `nav.sh` 中非常关键，因为它体现了一个明确的设计思想：

系统底层使用三维雷达和 LIO 获取更完整、更稳定的环境信息；但导航和定位中的某些模块，仍然采用成熟的二维表达形式进行处理。

因此，这个模块扮演的是“表达转换器”的角色。它把三维感知结果降维为二维 `LaserScan`，为后续模块提供通用输入。

在这条链路里，`/scan` 至少承担了两类作用：

- 作为 Nav2 局部代价地图的障碍输入
- 作为 `icp_registration` 的扫描匹配输入

这说明 `pointcloud_to_laserscan` 不只是简单“兼容 Nav2”，而是在整套地图定位链路中充当了重要桥梁。

---

### 3.5 `pcd2pgm.launch.py`：将点云地图转为二维栅格地图

文件路径：
[`src/mapper/pcd2pgm/launch/pcd2pgm.launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/mapper/pcd2pgm/launch/pcd2pgm.launch.py)

配置文件：
[`src/mapper/pcd2pgm/config/pcd.yaml`](/home/hero/ros2_humble_2D_backup_omin/src/mapper/pcd2pgm/config/pcd.yaml)

这个 launch 启动 `pcd2pgm_node`，其职责是读取指定 `.pcd` 文件，并将其转换为 Nav2 可用的二维栅格地图，再以 `map` 话题形式持续发布。

从配置文件可以看出，它会读取指定目录下的点云地图文件，并根据高度阈值、点数阈值和分辨率参数生成栅格地图。这意味着它承担的是“把三维建图成果转成导航地图”的工作。

这一层非常值得在复盘中强调，因为它体现了项目的一个重要落地思路：

- 底层建图结果以三维点云形式保留更多环境细节
- 上层导航仍然依赖二维栅格地图这一成熟、稳定、生态完善的表达方式

因此，`pcd2pgm` 实际上是地图侧的桥梁模块。没有它，点云地图很难直接被 Nav2 当作标准二维地图使用。

---

### 3.6 `icp.launch.py`：基于扫描匹配的地图配准定位

文件路径：
[`src/registration/icp_registration/launch/icp.launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/registration/icp_registration/launch/icp.launch.py)

配置文件：
[`src/registration/icp_registration/config/icp.yaml`](/home/hero/ros2_humble_2D_backup_omin/src/registration/icp_registration/config/icp.yaml)

这个 launch 启动 `icp_registration_node`。从参数配置和源码结构可以看出，它会：

- 读取指定的 `.pcd` 地图文件
- 订阅 `/scan`
- 使用 `laser_link`、`odom`、`map` 等坐标系配置进行匹配
- 计算并持续发布 `map -> odom` 变换

这意味着它在系统中的作用并不是建图，而是定位对齐。更具体地说，它解决的是：机器人如何把“当前扫描到的局部环境”对齐到“已经保存好的全局地图”上。

这一步的重要性非常高。因为 Nav2 要正常工作，不仅要知道机器人相对于自身底盘的运动，还要知道机器人在全局地图中的位置。`FAST_LIO` 更偏向提供连续的局部位姿估计，而 ICP 配准模块则在这里承担了“把局部运动轨迹挂接到全局地图”的职责。

从系统关系上可以这样理解：

- `FAST_LIO` 提供连续、实时的局部状态估计
- `pointcloud_to_laserscan` 生成用于配准的 `/scan`
- `icp_registration` 将当前扫描与已有地图进行匹配
- 最终输出 `map -> odom`，为 Nav2 提供全局定位参考

因此，这个模块是 `nav.sh` 和 `navbox.sh` 最大的区别之一。`nav.sh` 明显更强调“已有地图条件下的配准定位”。

---

### 3.7 `navigation2.launch.py`：Nav2 导航决策主入口

文件路径：
[`src/navigation/robot_navigation2/launch/navigation2.launch.py`](/home/hero/ros2_humble_2D_backup_omin/src/navigation/robot_navigation2/launch/navigation2.launch.py)

参数文件：
[`src/navigation/robot_navigation2/config/nav2_params.yaml`](/home/hero/ros2_humble_2D_backup_omin/src/navigation/robot_navigation2/config/nav2_params.yaml)

这个 launch 是导航栈的总入口，负责拉起规划、控制、行为树、代价地图和 RViz 等核心模块。

在 `nav.sh` 这条链路中，Nav2 处于“感知结果消费者”和“运动控制组织者”的位置。前面的 Livox、FAST_LIO、`pcd2pgm`、ICP 已经分别完成了环境输入、位姿估计、地图生成和全局配准，而 Nav2 则基于这些输入完成：

- 目标点到路径的规划
- 对局部障碍的代价地图建模
- 速度指令的生成

从配置可以看出，局部代价地图会使用 `/scan` 作为障碍输入。相比 `navbox.sh` 中同时引入 `/d435_scan` 的双源障碍感知方案，`nav.sh` 的导航链路更纯粹地依赖 Livox + LIO + 点云转扫描这一主干。

这也反映出 `nav.sh` 更偏“基础导航主链”而非“多传感器补盲增强链”。

---

### 3.8 `omni_drive_node`：底盘执行层

文件路径：
[`src/driver/omin/omni_drive/omni_drive/omni_drive_node.py`](/home/hero/ros2_humble_2D_backup_omin/src/driver/omin/omni_drive/omni_drive/omni_drive_node.py)

这个节点是整条链路的最终执行端。它订阅 `cmd_vel`，根据四轮全向底盘运动学将速度指令分解为各个车轮速度，然后通过 CAN 总线发送给驱动器。

从代码可以看出，它完成了几件非常关键的事情：

- 订阅 `Twist` 类型的 `cmd_vel`
- 根据全向轮运动学进行逆解
- 做速度限幅和安全处理
- 通过 CAN 向两个驱动器下发前后轮控制指令
- 在退出或异常时主动停止电机

这说明 `omni_drive_node` 并不是一个简单的“底层驱动”，而是导航结果真正落地到机器人运动的最后一环。

从系统角度看，前面的所有模块都在回答“机器人应该怎么走”，而这个节点负责回答“机器人如何真的动起来”。

因此，`nav.sh` 之所以是一条闭环链路，不只是因为它有感知和导航，更因为它把 `cmd_vel` 最终送到了真实底盘执行层。

---

## 4. 模块之间的联系：从地图构建成果到真实运动闭环

如果将整个 `nav.sh` 串起来理解，可以得到下面这条主流程：

1. Livox MID360 发布雷达和 IMU 数据
2. `robot_state_publisher` 建立机器人本体 TF 框架
3. `FAST_LIO` 进行激光惯导融合，输出位姿和配准点云
4. `pointcloud_to_laserscan` 将三维点云转成 `/scan`
5. `pcd2pgm` 读取 `.pcd` 地图并生成二维 `map`
6. `icp_registration` 使用 `/scan` 与 `.pcd` 地图匹配，持续估计 `map -> odom`
7. Nav2 在地图、位姿和障碍信息基础上完成规划与控制，输出 `cmd_vel`
8. `omni_drive_node` 接收 `cmd_vel` 并驱动全向底盘运动

如果进一步抽象，这条链路其实对应三个连续的问题：

- 机器人看到什么：Livox + FAST_LIO + `/scan`
- 机器人在地图哪里：`pcd2pgm` + `icp_registration`
- 机器人如何移动：Nav2 + `omni_drive_node`

这说明 `nav.sh` 的设计重点非常明确，它追求的是一个从“地图可用”到“真实可走”的完整工程闭环。

---

## 5. 与 `navbox.sh` 的差异

如果把 `nav.sh` 和 `navbox.sh` 放在一起看，两者的定位并不相同。

`navbox.sh` 更强调：

- 深度相机补充近距离障碍感知
- 自动重定位辅助
- 语音交互和任务入口
- 更完整的业务演示链路

而 `nav.sh` 更强调：

- 利用点云地图生成二维地图
- 通过 ICP 将当前扫描配准到已有地图
- 建立 `map -> odom` 的全局定位关系
- 将 Nav2 输出直接落实到底盘控制执行

所以可以把两者做一个简洁区分：

- `navbox.sh` 更像“集成化运行与交互链”
- `nav.sh` 更像“地图定位导航与执行链”

这一点对项目复盘很重要，因为它说明项目并不是只有一条固定运行方式，而是根据任务重点形成了不同的启动编排方案。

---

## 6. 复盘结论

从复盘视角看，`nav.sh` 的核心价值不在于它启动了多少模块，而在于它把“点云地图成果、全局地图定位、Nav2 路径规划和真实底盘控制”组织成了一条闭环。

它体现出的工程思路非常清晰：

- 用 Livox 和 `FAST_LIO` 保证稳定的底层感知与位姿估计
- 用 `pcd2pgm` 解决三维地图向二维导航地图的转换
- 用 `icp_registration` 解决已有地图条件下的全局配准定位
- 用 Nav2 负责决策与控制生成
- 用底盘驱动节点负责最终执行

因此，`nav.sh` 代表的是项目中偏向“可落地导航执行”的那条主工程链路。

如果说 `navbox.sh` 展现的是系统集成和人机交互扩展，那么 `nav.sh` 更集中体现了项目在地图利用、定位闭环和真实运动执行方面的核心能力。

---

## 7. 当前架构特点与可继续优化的方向

从现有实现来看，`nav.sh` 这条链路已经具有较强的工程完整性，但也能看出一些典型特征。

优点：

- 从底层感知到底盘执行形成了完整闭环
- 将三维点云地图成功转接到二维导航框架中，工程思路清晰
- ICP 模块补足了全局地图定位能力
- 全向底盘执行链路明确，导航结果能直接落地

可以继续优化的方向：

- 启动顺序目前依赖 `sleep`，建议逐步改为依赖就绪检测或生命周期管理
- `pcd` 路径、工作空间路径等配置存在硬编码，移植性一般
- `pcd2pgm` 与 `icp_registration` 对地图文件命名和路径耦合较强，后续可统一配置入口
- `omni_drive_node` 直接订阅 `cmd_vel`，后续若引入更多控制源，建议增加仲裁或安全层
- `map`、`odom`、`laser_link`、`base_footprint` 等 TF 链的一致性仍是整条链路稳定性的关键风险点

总体来说，`nav.sh` 既是一份启动脚本，也是一张很清晰的系统架构图。理解了这条链路，就基本理解了项目如何把已有地图、实时扫描、导航决策和真实底盘执行整合成一个可运行系统。
