cmake_minimum_required(VERSION 3.5)
project(chassis_msgs)
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
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrakeInfo.msg"
  "msg/CarStatus.msg"
  "msg/GearBoxInfo.msg"
  "msg/ICAKey.msg"
  "msg/LeadLineInfo.msg"
  "msg/LightInfo.msg"
  "msg/Line.msg"
  "msg/NetManagerPdu.msg"
  "msg/SteerInfo.msg"
  "msg/TargetInfo.msg"
  "msg/TargetInfoArray.msg"
  "msg/ThrottleInfo.msg"
  "msg/CarMassInfo.msg"
  "msg/CarSpeedInfo.msg"
  "msg/WheelSpeedInfo.msg"
  "msg/ArbiratorInfo.msg"
  "msg/GnssImuInfo.msg"
  "msg/HuResqInfo.msg"
  "msg/SFDInfo.msg"
  "msg/LVPwrInfo.msg"
  "msg/LgtdInfo.msg"
  "msg/LatInfo.msg"
  "msg/WhiInfo.msg"
  DEPENDENCIES common_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()