cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)



# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Link directories for PCL
link_directories(${PCL_LIBRARY_DIRS})

# Add library
add_library(plan_env
  include/plan_env/edt_environment.h
  include/plan_env/linear_obj_model.hpp
  include/plan_env/map_ros.h
  include/plan_env/obj_predictor.h
  include/plan_env/polynomial_traj.hpp
  include/plan_env/raycast.h
  include/plan_env/sdf_map.h
  src/sdf_map.cpp 
  src/obj_predictor.cpp 
  src/edt_environment.cpp 
  src/raycast.cpp
  src/map_ros.cpp
)


# 添加头文件地址
target_include_directories(plan_env PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# Link libraries
ament_target_dependencies(plan_env
  rclcpp
  rclpy
  sensor_msgs
  visualization_msgs
  cv_bridge
  message_filters
  Eigen3
  PCL
  OpenCV
  nav_msgs 
  pcl_conversions
)

target_link_libraries( plan_env
    ${PCL_LIBRARIES}
)  

# 为了使下游文件可以访问
ament_export_targets(plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  visualization_msgs
  cv_bridge
  message_filters
  Eigen3
  PCL
  OpenCV
  nav_msgs 
  pcl_conversions
)


# Export package
# install(TARGETS plan_env
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS plan_env # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


install(DIRECTORY include/
  DESTINATION include/
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
