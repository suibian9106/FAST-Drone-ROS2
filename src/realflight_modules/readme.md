### 启动vins_fusion
```bash
ros2 run vins vins_node install/vins/share/vins/config/fast_drone_250.yaml
```
VINS-Fusion订阅的话题
imu_topic: "mavros/imu/data_raw"
image0_topic: "camera/infra1/image_rect_raw"
image1_topic: "camera/infra2/image_rect_raw"

### 启动realsense
```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true;
```
realsense发布vins-fusion需要的话题

### 启动mavros
```bash
ros2 run mavros mavros_node --ros-args --param fcu_url:="udp://:14540@127.0.0.1:14557"
```