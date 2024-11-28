cmake_minimum_required(VERSION 3.8)
project(rosa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(traj_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(VTK REQUIRED)
find_package(pcl_conversions REQUIRED)



# 包含头文件
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



# 构建库
add_library(rosa SHARED
    include/rosa/datawrapper.h
    include/rosa/Extra_Del.h
    include/rosa/rosa_main.h
    src/rosa_main.cpp
    src/Extra_Del.cpp
)

# 添加头文件地址
target_include_directories(rosa PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)




ament_target_dependencies(rosa
  rclcpp
  std_msgs
  Eigen3
  PCL
  traj_utils
  quadrotor_msgs
  VTK
  pcl_conversions
)

# 构建可执行文件
add_executable(rosa_exec
    src/rosa_exec.cpp
    src/rosa_main.cpp
    src/Extra_Del.cpp
)

ament_target_dependencies(rosa_exec
  rclcpp
  std_msgs
  Eigen3
  PCL
  traj_utils
  quadrotor_msgs
  VTK
  pcl_conversions
)

# target_include_directories(rosa_exec
#     PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#     ${traj_utils_INCLUDE_DIRS}
# )


target_link_libraries(rosa_exec
    ${PCL_LIBRARIES}
)


# target_include_directories(rosa
#     PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#     ${traj_utils_INCLUDE_DIRS}
# )

target_link_libraries(rosa
    ${PCL_LIBRARIES}
)


set(library
  rosa
  rosa_exec
)


ament_export_libraries(rosa HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  Eigen3
  PCL
  traj_utils
  quadrotor_msgs
  VTK
  pcl_conversions
)

ament_export_include_directories(include)
ament_export_libraries(rosa)


# 安装目标
# install(TARGETS rosa rosa_exec
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS rosa # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT rosa
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS rosa_exec # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT rosa_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

# 安装 data 文件夹中的 pcd 文件
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# 安装 config 文件夹中的 rviz 文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


message(STATUS "traj_utils include dirs: ${traj_utils_INCLUDE_DIRS}")
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")


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
