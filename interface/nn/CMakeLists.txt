cmake_minimum_required(VERSION 3.8)
project(perception_nn_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Demomsg.msg
  msg/PointXY.msg
  msg/PointXYZ.msg
  msg/PointXYZI.msg
  msg/Type.msg
  msg/PublicAttrs.msg
  msg/SubType.msg
  msg/SpecialVehicleType.msg
  msg/LineAttrs.msg
  msg/LineCurve.msg
  msg/LaneLine.msg
  msg/LaneLines.msg
  msg/LaneLineColor.msg
  msg/LaneLinePosition.msg
  msg/LaneLineSceneType.msg
  msg/LaneLineType.msg
  msg/SplitLine.msg
  msg/SplitLine2F.msg
  msg/SplitLine3F.msg
  msg/Box2D.msg
  msg/Box3D.msg
  msg/Boxes3D.msg
  msg/Object.msg
  msg/Objects.msg
  msg/Tensor.msg
  DEPENDENCIES
  std_msgs
  common_msgs
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
