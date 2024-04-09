cmake_minimum_required(VERSION 3.5)
project(control_msgs)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(common_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(chart_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/Debug.msg"
  "msg/MpcDebug.msg"
  "msg/PostDebug.msg"
  "msg/PreDebug.msg"
  "msg/LongitudinalDebug.msg"
  "msg/LateralDebug.msg"
  "msg/MassIdentificationDebug.msg"
  DEPENDENCIES common_msgs
  DEPENDENCIES chart_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()