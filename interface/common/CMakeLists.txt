cmake_minimum_required(VERSION 3.5)
project(common_msgs)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EngageAdvice.msg"
  "msg/ErrorCode.msg"
  "msg/Header.msg"
  "msg/FrenetFramePoint.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/Point2D.msg"
  "msg/Point3D.msg"
  "msg/Quaternion.msg"
  "msg/SLPoint.msg"
  "msg/SpeedPoint.msg"
  "msg/StatusPb.msg"
  "msg/STPoint.msg"
  "msg/SVPoint.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Vector3D.msg"
  "msg/VehicleSignal.msg"
  "msg/LocalizeState.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()