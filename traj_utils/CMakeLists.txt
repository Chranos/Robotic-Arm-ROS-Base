cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找需要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# 包含头文件
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)




# # 设置 INCLUDE_DIRS 变量
# set(traj_utils_INCLUDE_DIRS
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )


# # 导出 INCLUDE_DIRS
# ament_export_include_directories(${traj_utils_INCLUDE_DIRS})

# 链接库
link_directories(${PCL_LIBRARY_DIRS})

# 添加源文件，生成库
add_library(traj_utils SHARED
  include/traj_utils/planning_visualization.h
  src/planning_visualization.cpp
)

# 添加头文件地址
target_include_directories(traj_utils PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 链接库和依赖
ament_target_dependencies(traj_utils 
  PUBLIC
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge
  quadrotor_msgs
  Eigen3
  PCL
  visualization_msgs
  pcl_conversions 
)

# 为了使下游文件可以访问
ament_export_targets(traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge
  quadrotor_msgs
  Eigen3
  PCL
  visualization_msgs
  pcl_conversions 
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)


# 安装库文件
install(TARGETS traj_utils # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


#忽略警告
# add_compile_options(-Wno-pedantic)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pedantic")



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

# ament_package()
# 包配置
ament_package()