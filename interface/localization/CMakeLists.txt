cmake_minimum_required(VERSION 3.5)
project(localization_msgs)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GlobalLocalizeResult.msg"
  "msg/GlobalLocInfo.msg"
  "msg/LaneInLocInfo.msg"
  "msg/LaneLevelLocInfo.msg"
  "msg/LocalizeOutput.msg"
  "msg/LocalLocalizeResult.msg"
  "msg/RoadLocInfo.msg"
  "msg/Transformd.msg"
  "msg/LidarGlocOutput.msg"
  DEPENDENCIES common_msgs
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()