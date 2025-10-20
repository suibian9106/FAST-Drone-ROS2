sudo chmod 777 /dev/ttyACM0 & sleep 2;
ros2 launch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true & sleep 10;
ros2 run mavros mavros_node --ros-args --param fcu_url:=/dev/ttyACM0:57600 & sleep 10;
ros2 launch vins fast_drone_250.launch.py
wait;
