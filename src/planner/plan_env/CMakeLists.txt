cmake_minimum_required(VERSION 3.5)
project(plan_env)

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
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# 包含目录
include_directories(
  SYSTEM 
  include 
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 创建库
add_library(plan_env 
  src/grid_map.cpp 
  src/raycast.cpp
  src/obj_predictor.cpp
)

# 链接库依赖
target_link_libraries(plan_env
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${message_filters_LIBRARIES}
)

# 添加ROS2特定的依赖管理
ament_target_dependencies(plan_env
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

# 创建可执行文件
add_executable(obj_generator
  src/obj_generator.cpp 
)

# 链接可执行文件依赖
target_link_libraries(obj_generator
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(obj_generator
  rclcpp
  std_msgs
)

# 安装目标
install(TARGETS
  plan_env
  obj_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装包含目录
install(DIRECTORY include/
  DESTINATION include/
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_dependencies(
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
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
