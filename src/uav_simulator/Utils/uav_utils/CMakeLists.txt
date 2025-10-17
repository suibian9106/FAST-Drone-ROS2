cmake_minimum_required(VERSION 3.5)
project(uav_utils)

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

# Find Eigen3
find_package(Eigen3 REQUIRED)

# 创建接口库（因为 uav_utils 只有头文件）
add_library(${PROJECT_NAME} INTERFACE)

# 添加包含目录到接口库
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${Eigen3_INCLUDE_DIRS}  # 添加 Eigen3 包含目录
)

# 添加编译定义（如果有）
# target_compile_definitions(${PROJECT_NAME} INTERFACE ...)

# 添加依赖到接口库
target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen  # 链接 Eigen3 目标
)

# 导出目标
ament_export_targets(${PROJECT_NAME})

# 导出包含目录
ament_export_include_directories(include)

# 导出依赖（对于非 ament 包，可能需要手动处理）
# ament_export_dependencies(Eigen3)  # 注释掉，因为 Eigen3 不是 ament 包

# 对于 Eigen3，我们手动导出包含目录
ament_export_include_directories(${Eigen3_INCLUDE_DIRS})

#############
## Install ##
#############

# Mark header files for installation
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# 安装导出目标
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下行可选的，根据您的需要启用/禁用不同的lint检查
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cppcheck_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_flake8_FOUND TRUE)
  # set(ament_cmake_lint_cmake_FOUND TRUE)
  # set(ament_cmake_pep257_FOUND TRUE)
  # set(ament_cmake_uncrustify_FOUND TRUE)
  # set(ament_cmake_xmllint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()

  # 添加gtest测试（如果有测试文件的话）
  if(AMENT_CMAKE_GTEST_FOUND)
    ament_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
    if(TARGET ${PROJECT_NAME}-test)
      target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
      ament_target_dependencies(${PROJECT_NAME}-test rclcpp rclpy)
    endif()
  endif()
endif()

ament_package()
