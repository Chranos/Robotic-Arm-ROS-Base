cmake_minimum_required(VERSION 3.8)
project(hierarchical_coverage_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosa REQUIRED)
find_package(traj_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)
find_package(path_searching REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(viewpoint_manager REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)



# 安装 config 文件夹中的 rviz 文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 data 文件
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
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

# 安装 solution 文件夹中的 文件
install(
  DIRECTORY solution/
  DESTINATION share/${PROJECT_NAME}/solution
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
