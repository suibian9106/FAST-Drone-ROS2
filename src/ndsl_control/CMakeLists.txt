cmake_minimum_required(VERSION 3.5)
project(ndsl_control)

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

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Create human_control_node executable
add_executable(human_control_node 
  src/human_control_interface.cpp
  src/human_control_node.cpp
)

ament_target_dependencies(human_control_node
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
)

target_link_libraries(human_control_node
  ${PCL_LIBRARIES}
)

# Create example_node executable
add_executable(example_node 
  src/human_control_interface.cpp
  src/example_node/example_node.cpp
)

ament_target_dependencies(example_node
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
)

target_link_libraries(example_node
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  human_control_node
  example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  Eigen3
)

# Export include directories
ament_export_include_directories(include)

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
