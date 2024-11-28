cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(
    ${ament_INCLUDE_DIRS}
    include
    )

set(dependencies
  geometry_msgs  
  nav_msgs
  std_msgs
)

set(msg_files 
    "msg/AuxCommand.msg"
    "msg/Corrections.msg"
    "msg/Gains.msg"
    "msg/OutputData.msg"
    "msg/PositionCommand.msg"
    "msg/PPROutputData.msg"
    "msg/Serial.msg"
    "msg/SO3Command.msg"
    "msg/StatusData.msg"
    "msg/TRPYCommand.msg"
    "msg/Odometry.msg"
    "msg/PolynomialTrajectory.msg"
    "msg/PolynomialTraj.msg"
    "msg/PolynomialTrajGroup.msg"
    "msg/PolynomialMatrix.msg"
    "msg/TakeoffLand.msg"
    "msg/Px4ctrlDebug.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES nav_msgs geometry_msgs std_msgs
)

ament_register_extension(
    "rosidl_generate_interfaces"
    "rosidl_generator_cpp"
    "rosidl_generator_cpp_generate_interfaces.cmake"
)





add_library(decode_msgs SHARED
  src/decode_msgs.cpp
)
add_library(encode_msgs SHARED
  src/encode_msgs.cpp
)

set(librarys
  encode_msgs
  decode_msgs
)

target_include_directories(decode_msgs PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(encode_msgs PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(decode_msgs
  ${dependencies}
)
ament_target_dependencies(encode_msgs
  ${dependencies}
)

rosidl_target_interfaces(decode_msgs
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(encode_msgs
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


ament_export_libraries(${librarys})
ament_export_dependencies(rosidl_default_runtime)

# 注册 导出头文件
install(
        DIRECTORY include/
        DESTINATION include
)

# 注册 导出库文件
install(
        TARGETS  decode_msgs# 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT decode_msgs
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# 注册 导出库文件
install(
        TARGETS  encode_msgs# 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT encode_msgs
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
ament_export_include_directories(include)

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
