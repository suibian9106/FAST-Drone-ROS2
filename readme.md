## realsense 安装
```bash
#依赖
git clone -b v2.51.1 https://github.com/IntelRealSense/librealsense.git
sudo apt-get install guvcview git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev -y
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev -y

#安装权限脚本
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 

#编译
cd librealsense
mkdir build
cd build
cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=false -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make
sudo make install

```


## ROS2 安装

* **设置编码**
```bash
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
$ export LANG=en_US.UTF-8
```
* **添加源**
```bash
$ sudo apt update && sudo apt install curl gnupg lsb-release 
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

* **安装ROS2**
```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-foxy-desktop
```
* **设置环境变量**
```bash
sudo vim ~/.bashrc 
##在文件最后添加以下内容，使ROS1和ROS2共存
unset ROS_DISTRO
unset ROS_PACKAGE_PATH
unset ROS_VERSION
echo "ros noetic(1) or ros2 foxy(2)?"
read edition
if [ "$edition" -eq "1" ];then
  source /opt/ros/noetic/setup.bash
else
  source /opt/ros/foxy/setup.bash
fi
```
## 3. 实验前的参数设置

### 3.1 VINS 参数设置

```shell
cd ./src/realflight_modules/VINS_Fusion/config/
ros2 topic echo /camera/infra1/camera_info # 把其中的K矩阵中的fx,fy,cx,cy填入left.yaml和right.yaml
mkdir /home/coolpi/vins_output # vins输出的结果会存储在这里
```

修改 fast-drone-250.yaml 的 body_T_cam0 和 body_T_cam1 的 data 矩阵的第四列为你的无人机上的相机相对于飞控的实际外参，单位为米，顺序为 x/y/z，第四项是 1，不用改

```yaml
body_T_cam0: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [0, 0, 1, 0.02, -1, 0, 0, 0.02, 0, -1, 0, 0.02, 0, 0, 0, 1]
body_T_cam1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [0, 0, 1, 0.02, -1, 0, 0, -0.03, 0, -1, 0, 0.02, 0, 0, 0, 1]
```

### 3.2 VINS 外参标定

```shell
sh shfiles/rspx4.sh
rostopic echo /vins_fusion/imu_propagate
```

- 拿起飞机沿着场地尽量缓慢地行走，场地内光照变化不要太大，灯光不要太暗，不要使用会频闪的光源，尽量多放些杂物来增加 VINS 用于匹配的特征点
- 把`vins_output/extrinsic_parameter.txt`里的内容替换到`fast-drone-250.yaml`的`body_T_cam0`和`body_T_cam1`
  重复上述操作直到走几圈后 VINS 的里程计数据偏差收敛到满意值（一般在 0.3 米内）

### 3.3 ego-planner 验证

```shell
sh shfiles/rspx4.sh
ros2 launch ego_planner single_run_in_exp.launch.py
ros2 launch ego_planner rviz.launch.py
```

## 3. 启动 Fast Drone

**:skull: 注意默认源码编译的 realsense ros wrapper 是不会拉取灰度图像流的，所以需要在启动 realsense 节点语句最后增加`enable_infra1:=true enable_infra2:=true`**

```shell
sudo chmod 777 /dev/ttyACM0 & sleep 2;
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true & sleep 10;
ros2 run mavros mavros_node --ros-args --param fcu_url:=/dev/ttyACM0:57600 & sleep 10;
# ros2 launch vins fast_drone_250.launch.py
ros2 run vins vins_node install/vins/share/vins/config/fast_drone_250.yaml
wait;
```

**确认 IMU 频率接近 200hz**

```shell
ros2 topic hz /mavros/imu/data_raw
```

**如果没有达到，执行下面的命令**

```shell
# 设置EXTENDED_STATUS数据流,5000us对应200HZ
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{
  command: 511,
  param1: 105,
  param2: 5000,
  param3: 0,
  param4: 0,
  param5: 0,
  param6: 0,
  param7: 0
}"

# 设置SCALED_IMU数据流
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{
  command: 511,
  param1: 31,
  param2: 5000,
  param3: 0,
  param4: 0,
  param5: 0,
  param6: 0,
  param7: 0
}"
```

## 4. 实验

### 4.0 实验前的准备

```shell
sh shfiles/rspx4.sh
ros2 topic echo /vins_fusion/imu_propagate
```

- 拿起飞机进行缓慢的小范围晃动，放回原地后确认没有太大误差
- 遥控器 5 通道拨到内侧，六通道拨到下侧，油门打到中位

### 4.1 auto takeoff + hover + auto land

- 保证`shfiles/rspx4.sh`脚本已经在运行

```shell
ros2 launch px4ctrl run_ctrl.launch.py
sh shfiles/takeoff.sh
# 等待无人机悬停一会后
sh shfiles/land.sh
```

**注意：如果飞机螺旋桨开始旋转，但无法起飞，说明 hover_percent 参数过小；如果飞机有明显飞过 1 米高，再下降的样子，说明 hover_percent 参数过大**

### 4.2 auto takeoff + hover + auto land + auto fly

- 保证`shfiles/rspx4.sh`脚本已经在运行

```shell
ros2 launch px4ctrl run_ctrl.launch.py
sh shfiles/takeoff.sh
ros2 launch ego_planner single_run_in_exp.launch.py
sh shfiles/record.sh
# 执行我们自己的指点飞行
ros2 run ndsl_control ndsl_control_node
# 等待指点飞行结束
sh shfiles/land.sh
```


## px4ctrl话题服务

根据代码分析，这个px4ctrl ROS2节点的通信方式如下：

### 订阅的话题（Subscriptions）

1. **`/mavros/state`** - `mavros_msgs::msg::State`
   - 飞行器状态信息（连接状态、模式、是否上锁等）

2. **`/mavros/extended_state`** - `mavros_msgs::msg::ExtendedState`
   - 扩展状态信息（着陆状态等）

3. **`odom`** - `nav_msgs::msg::Odometry`
   - 里程计信息（位置、速度、姿态），QoS为100

4. **`cmd`** - `quadrotor_msgs::msg::PositionCommand`
   - 位置控制指令，QoS为100

5. **`/mavros/imu/data`** - `sensor_msgs::msg::Imu`
   - IMU数据（姿态、角速度、线加速度），QoS为100

6. **`/mavros/rc/in`** - `mavros_msgs::msg::RCIn`（可选）
   - 遥控器输入数据，根据参数`takeoff_land.no_RC`决定是否启用

7. **`/mavros/battery`** - `sensor_msgs::msg::BatteryState`
   - 电池状态信息，QoS为100

8. **`takeoff_land`** - `quadrotor_msgs::msg::TakeoffLand`
   - 起飞/着陆指令，QoS为100

### 发布的话题（Publishers）

1. **`/mavros/setpoint_raw/attitude`** - `mavros_msgs::msg::AttitudeTarget`
   - 姿态控制指令（姿态四元数或角速度+推力）

2. **`/traj_start_trigger`** - `geometry_msgs::msg::PoseStamped`
   - 轨迹开始触发信号

3. **`/debugPx4ctrl`** - `quadrotor_msgs::msg::Px4ctrlDebug`
   - 调试信息

### 使用的服务（Services - 客户端）

1. **`/mavros/set_mode`** - `mavros_msgs::srv::SetMode`
   - 设置飞行模式（如OFFBOARD模式）

2. **`/mavros/cmd/arming`** - `mavros_msgs::srv::CommandBool`
   - 上锁/解锁电机

3. **`/mavros/cmd/command`** - `mavros_msgs::srv::CommandLong`
   - 发送通用命令（用于重启FCU）

### 主要功能总结

这个节点实现了一个完整的PX4飞行控制器，具有：
- **状态机管理**：手动控制、自动悬停、指令控制、自动起飞、自动着陆
- **线性控制器**：位置、速度、加速度控制
- **推力模型估计**：在线估计推力-加速度映射关系
- **安全机制**：超时检测、着陆检测、遥控器切换
- **调试输出**：发布详细的控制器状态信息

节点运行频率由`param.ctrl_freq_max`参数控制，支持姿态控制和角速度控制两种输出模式。


根据提供的代码文件，我可以分析出ego-planner系统的向上提供的接口和需要接收的数据来源：

## 向上提供的接口（发布的服务和话题）

### 1. B样条轨迹发布
```cpp
// 发布给轨迹服务器的B样条轨迹
bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
```

### 2. 集群轨迹发布（多机协同）
```cpp
// 发布给其他无人机的集群轨迹
string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);
```

### 3. 广播B样条轨迹
```cpp
// 广播B样条轨迹给其他规划器
broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
```

### 4. 数据显示发布
```cpp
// 发布规划数据用于显示
data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
```

## 需要接收的数据来源

### 1. 里程计数据
```cpp
// 世界坐标系下的里程计数据
odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
```

### 2. 目标点输入
根据目标类型不同，有两种输入方式：

**手动目标模式**：
```cpp
// 来自RVIZ的目标点
waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
```

**预设目标模式**：
```cpp
// 来自控制器的触发信号
trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
```

### 3. 集群轨迹数据
```cpp
// 接收其他无人机的轨迹信息
if (planner_manager_->pp_.drone_id >= 1)
{
    string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
    swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
}
```

### 4. 广播B样条轨迹
```cpp
// 接收其他规划器广播的B样条轨迹
broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());
```

## 数据流总结

**输入数据来源**：
- **定位信息**：`odom_world`话题（世界坐标系下的里程计）
- **任务触发**：
  - 手动模式：`/move_base_simple/goal`（RVIZ点击目标）
  - 预设模式：`/traj_start_trigger`（来自px4ctrl的触发信号）
- **集群信息**：
  - `/drone_{id-1}_planning/swarm_trajs`（前一个无人机的轨迹）
  - `planning/broadcast_bspline_to_planner`（广播轨迹）

**输出规划结果**：
- **轨迹执行**：`planning/bspline` → traj_server → px4ctrl
- **集群通信**：`/drone_{id}_planning/swarm_trajs`（给下一个无人机）
- **系统广播**：`planning/broadcast_bspline_from_planner`（广播本机轨迹）
- **可视化**：`planning/data_display`（规划数据显示）

## 与px4ctrl系统的关系

ego-planner和px4ctrl构成了完整的无人机自主导航系统：

```
ego-planner (规划层) → traj_server (轨迹服务器) → px4ctrl (控制层) → PX4飞控
```

**数据流向**：
1. **ego-planner**接收目标点和当前位置，生成B样条轨迹
2. 轨迹通过`planning/bspline`发布给**traj_server**
3. **traj_server**将B样条轨迹转换为位置指令，通过`/position_cmd`发布给**px4ctrl**
4. **px4ctrl**根据位置指令生成控制指令发送给PX4飞控

**触发机制**：
- 在预设目标模式下，**px4ctrl**通过`/traj_start_trigger`触发**ego-planner**开始规划
- 这实现了控制层和规划层的解耦和协同工作

这个系统支持单机和多机协同规划，具有完整的自主导航能力。


基于提供的代码，使用ego-planner和px4ctrl实现无人机操控的全流程如下：

## 🚀 系统启动与初始化流程

### 1. 系统组件启动顺序
```
PX4飞控 → MAVROS → px4ctrl → ego-planner → traj_server
```

### 2. 初始化阶段
**px4ctrl初始化：**
- 读取控制参数（PID增益、质量、最大速度等）
- 订阅MAVROS话题（状态、IMU、里程计、电池等）
- 等待RC遥控器信号（除非配置为无RC模式）
- 等待与PX4飞控连接

**ego-planner初始化：**
- 读取规划参数（最大速度、加速度、规划视野等）
- 初始化地图和优化器
- 根据配置选择目标输入方式（手动点击或预设航点）
- 等待触发信号和里程计数据

## 🎯 飞行模式切换流程

### 状态机转换流程：
```
MANUAL_CTRL → AUTO_HOVER → CMD_CTRL
```

### 详细步骤：

#### 步骤1：手动控制 → 自动悬停
```bash
# 通过RC遥控器切换
RC模式开关 → HOVER模式 → px4ctrl进入AUTO_HOVER状态
```

**条件检查：**
- ✓ 收到里程计数据
- ✓ 无人机速度 < 3.0m/s
- ✗ 没有正在接收控制指令

#### 步骤2：自动悬停 → 指令控制
```bash
# 通过RC遥控器切换
RC档位开关 → COMMAND模式 → px4ctrl进入CMD_CTRL状态
```

## 📍 任务规划与执行流程

### 方式1：手动目标模式
```
RVIZ点击目标点 → ego-planner接收目标 → 轨迹规划 → 执行
```

### 方式2：预设航点模式  
```
px4ctrl发送触发信号 → ego-planner读取预设航点 → 分段规划 → 顺序执行
```

### 详细规划执行流程：

#### 1. 目标接收阶段
```cpp
// ego-planner接收目标
waypointCallback() 或 triggerCallback()
↓
planNextWaypoint()  // 规划下一个航点
↓
changeFSMExecState(GEN_NEW_TRAJ)  // 状态切换
```

#### 2. 轨迹规划阶段
```cpp
// ego-planner进行轨迹规划
planFromGlobalTraj()  // 从全局轨迹规划
↓
callReboundReplan()   // 反弹重规划算法
↓
BsplineOptimizeTrajRebound()  // B样条优化
```

#### 3. 轨迹发布阶段
```cpp
// 发布规划结果
bspline_pub_.publish(bspline)        // → traj_server
swarm_trajs_pub_.publish(trajs)      // → 其他无人机（集群）
broadcast_bspline_pub_.publish(bspline) // → 广播
```

#### 4. 轨迹执行阶段
```cpp
// traj_server处理轨迹
bsplineCallback()  // 接收B样条轨迹
↓
cmdCallback()      // 定时发布位置指令
↓
pos_cmd_pub.publish(cmd)  // → px4ctrl
```

#### 5. 控制执行阶段
```cpp
// px4ctrl执行控制
process()  // 主处理循环
↓
calculateControl()  // 计算控制输出
↓
publish_attitude_ctrl()  // 发布姿态控制指令 → PX4
```

## 🔄 实时重规划流程

### 触发重规划的条件：
1. **位置偏差**：`(end_pt_ - pos).norm() > no_replan_thresh_`
2. **时间触发**：`t_cur > replan_thresh_`
3. **碰撞检测**：检测到障碍物或其他无人机
4. **航点切换**：到达当前航点，规划下一个航点

### 重规划过程：
```
检测到需要重规划 → changeFSMExecState(REPLAN_TRAJ)
↓
planFromCurrentTraj()  // 从当前轨迹重新规划
↓
callReboundReplan()    // 执行重规划
↓
发布新轨迹 → 继续执行
```

## 🛡️ 安全保护机制

### 1. 紧急停止
```cpp
// 检测到紧急情况
changeFSMExecState(EMERGENCY_STOP)
↓
callEmergencyStop()  // 紧急停止
↓
发布悬停轨迹
```

### 2. 碰撞检测
- **静态障碍物**：通过栅格地图检测
- **动态障碍物**：检测其他无人机轨迹
- **安全距离**：使用`getSwarmClearance()`确保安全间距

### 3. 超时保护
```cpp
// 各类消息超时检查
rc_is_received()    // RC信号
odom_is_received()  // 里程计
cmd_is_received()   // 控制指令
imu_is_received()   // IMU数据
```

## 🌐 多机协同流程

### 集群通信机制：
```
无人机N-1 → /drone_{N-1}_planning/swarm_trajs → 无人机N
↓
无人机N规划时考虑无人机N-1的轨迹
↓
无人机N发布自己的轨迹给无人机N+1
```

### 协同避障：
- 每架无人机广播自己的B样条轨迹
- 接收并解析其他无人机的轨迹
- 在规划时考虑集群轨迹避障
- 使用时间同步确保轨迹一致性

## 📊 系统特点总结

### 🎯 核心优势：
1. **分层架构**：规划、轨迹处理、控制分离，职责清晰
2. **状态机管理**：明确的状态转换，保证系统稳定性
3. **实时重规划**：应对环境变化和突发状况
4. **集群支持**：天然支持多机协同作业
5. **安全完备**：多重保护机制确保飞行安全

### 🔧 关键技术：
- **B样条轨迹**：平滑可导，适合无人机动力学
- **反弹重规划**：高效的局部轨迹优化
- **模型预测控制**：px4ctrl基于模型的控制算法
- **分布式通信**：基于ROS的集群通信机制

这个系统提供了一个完整、安全、高效的无人机自主导航解决方案，从单机到集群都有良好的扩展性。