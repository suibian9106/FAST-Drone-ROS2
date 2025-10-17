#-cmake_minimum_required(VERSION 2.4.6)
#-include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#-rosbuild_init()

#set the default path for built executables to the "bin" directory
#-set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#-set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#-\rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#-rosbuild_add_library(encode_msgs src/encode_msgs.cpp)
#-rosbuild_add_link_flags(encode_msgs -Wl,--as-needed)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)

#include_directories(${EIGEN3_INCLUDE_DIR})
#-rosbuild_add_library(decode_msgs src/decode_msgs.cpp)
#-rosbuild_add_link_flags(decode_msgs -Wl,--as-needed)

#------------------------------------------------------------------
cmake_minimum_required(VERSION 3.5)
project(quadrotor_msgs)
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
# find_package(catkin REQUIRED COMPONENTS
#   #armadillo
#   roscpp
#   nav_msgs
#   geometry_msgs
#   message_generation
# )
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/GoalSet.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/OptimalTimeAllocator.msg"
  "msg/TrajectoryMatrix.msg"
  "msg/ReplanCheck.msg"
  "msg/SpatialTemporalTrajectory.msg"
  "msg/Bspline.msg"
  "msg/SwarmCommand.msg"
  "msg/Replan.msg"
  "msg/SwarmOdometry.msg"
  "msg/SwarmInfo.msg"
  "msg/Px4ctrlDebug.msg"
  "msg/TakeoffLand.msg"
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
)

# # 添加库
# add_library(encode_msgs src/encode_msgs.cpp)
# add_library(decode_msgs src/decode_msgs.cpp)

# # 添加对生成消息的依赖
# add_dependencies(encode_msgs ${PROJECT_NAME})
# add_dependencies(decode_msgs ${PROJECT_NAME})

# # 为库添加依赖
# ament_target_dependencies(encode_msgs
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
#   Eigen3
# )

# ament_target_dependencies(decode_msgs
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
#   Eigen3
# )

# 包含目录
# target_include_directories(encode_msgs PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${Eigen3_INCLUDE_DIRS}
# )

# target_include_directories(decode_msgs PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${Eigen3_INCLUDE_DIRS}
# )

# # 安装目标
# install(TARGETS
#   encode_msgs
#   decode_msgs
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION include
# )

# 安装消息接口
#rosidl_export_interfaces(export_${PROJECT_NAME} ${PROJECT_NAME})

# 导出依赖
ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  nav_msgs
  std_msgs
  rclcpp
)

# 导出包含目录
# ament_export_include_directories(include)

# 导出库
# ament_export_libraries(encode_msgs decode_msgs)

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
##1.自身工程的.h库
ament_package()

