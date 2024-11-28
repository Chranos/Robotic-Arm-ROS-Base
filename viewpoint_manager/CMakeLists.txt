cmake_minimum_required(VERSION 3.8)
project(viewpoint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosa REQUIRED)
find_package(traj_utils REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories( 
    include
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# viewpoint_manager 库
add_library(viewpoint_manager
    src/viewpoint_manager.cpp
)

ament_target_dependencies(viewpoint_manager
  rclcpp
  std_msgs
  rosa
  traj_utils
  plan_env
  active_perception
  quadrotor_msgs
  PCL
  Eigen3
  OpenCV
  pcl_conversions
)

# Link PCL to viewpoint_manager library
target_link_libraries(viewpoint_manager
    ${PCL_LIBRARIES}
)

# skeleton_viewpoint 库
# add_library(skeleton_viewpoint
#     src/skeleton_viewpoint.cpp
# )

# ament_target_dependencies(skeleton_viewpoint
#   rclcpp
#   std_msgs
#   rosa
#   traj_utils
#   plan_env
#   active_perception
#   quadrotor_msgs
#   PCL
#   Eigen3
#   OpenCV
#   pcl_conversions
# )

# viewpoint_manager_node 可执行文件
add_executable(viewpoint_manager_node
    src/viewpoint_manager_node.cpp
)

ament_target_dependencies(viewpoint_manager_node
  rclcpp
  std_msgs
  rosa
  traj_utils
  plan_env
  active_perception
  quadrotor_msgs
  PCL
  Eigen3
  OpenCV
  pcl_conversions
)

# 设置 viewpoint_manager_node 可执行文件链接的库
target_link_libraries(viewpoint_manager_node
    viewpoint_manager
)

# skeleton_viewpoint_node 可执行文件
add_executable(skeleton_viewpoint_node
    src/skeleton_viewpoint_node.cpp
)

ament_target_dependencies(skeleton_viewpoint_node
  rclcpp
  std_msgs
  rosa
  traj_utils
  plan_env
  active_perception
  quadrotor_msgs
  PCL
  Eigen3
  OpenCV
  pcl_conversions
)

# 设置 skeleton_viewpoint_node 可执行文件链接的库
target_link_libraries(skeleton_viewpoint_node
    ${PCL_LIBRARIES}
    viewpoint_manager  # 如果 skeleton_viewpoint 依赖 viewpoint_manager 库
)


# 安装
install(TARGETS viewpoint_manager
    DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS skeleton_viewpoint
#     DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS viewpoint_manager_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS skeleton_viewpoint_node
    DESTINATION lib/${PROJECT_NAME}
)


# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 launch 文件夹中的 .launch.py 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 resource 文件夹中的 pcd 文件
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# 安装 config 文件夹中的 rviz 文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
