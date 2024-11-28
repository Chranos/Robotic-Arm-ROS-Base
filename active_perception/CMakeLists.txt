cmake_minimum_required(VERSION 3.8)
project(active_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(plan_env REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)



# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Build the library
add_library(active_perception
  include/active_perception/perception_utils.h
  src/perception_utils.cpp
)

# 添加头文件地址
target_include_directories(active_perception PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)




ament_target_dependencies(active_perception
  rclcpp
  std_msgs
  rclpy
  Eigen3
  PCL
  plan_env
  cv_bridge
  pcl_conversions
)
target_link_libraries(active_perception
  ${PCL_LIBRARIES}
  ${TORCH_LIBRARIES}
)

# 为了使下游文件可以访问
ament_export_targets(active_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  rclpy
  Eigen3
  PCL
  plan_env
  cv_bridge
  pcl_conversions
)

# 安装目标
# install(TARGETS active_perception
#   DESTINATION lib/${PROJECT_NAME}
# )
install(TARGETS active_perception # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT active_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
