cmake_minimum_required(VERSION 3.8)
project(viewlane_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneGeom.msg"
  "msg/ViewLaneInfo.msg"
  DEPENDENCIES common_msgs
  DEPENDENCIES localization_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
