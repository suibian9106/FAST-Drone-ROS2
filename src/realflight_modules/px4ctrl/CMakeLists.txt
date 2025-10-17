cmake_minimum_required(VERSION 3.5)
project(px4ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# 查找自定义消息包 - 需要确认这些包已支持ROS2
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)

# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 添加包含目录
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

# 设置目标依赖和链接库
target_include_directories(px4ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(px4ctrl_node PUBLIC c_std_99 cxx_std_14)
target_compile_definitions(px4ctrl_node PUBLIC
  $<$<BOOL:${Eigen3_FOUND}>:EIGEN3_FOUND>
)

# 链接依赖库
ament_target_dependencies(px4ctrl_node
  rclcpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  uav_utils
)

# 安装可执行文件
install(TARGETS px4ctrl_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装包含目录（如果有头文件需要被其他包使用）
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 安装config目录到install/share/px4ctrl下
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出依赖
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  uav_utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
