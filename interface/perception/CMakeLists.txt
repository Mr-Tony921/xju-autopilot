cmake_minimum_required(VERSION 3.5)
project(perception_em_msgs)
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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
set(msg_dependencies
  geometry_msgs
  sensor_msgs
  builtin_interfaces)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubicCurve.msg"
  "msg/EmLanes.msg"
  "msg/EmObstacle.msg"
  "msg/EmObstacles.msg"
  "msg/HdmapLaneUnit.msg"
  "msg/HdmapLineMark.msg"
  "msg/LaneInfo.msg"
  "msg/LaneMark.msg"
  "msg/LaneMarkingQuality.msg"
  "msg/LaneMarkingType.msg"
  "msg/LaneMarkingTypeSection.msg"
  "msg/LaneType.msg"
  "msg/ObsTrajectory.msg"
  "msg/PredictionTrajectory.msg"
  "msg/PredictionObstacle.msg"
  "msg/PredictionObstacles.msg"
  "msg/RestrictedLaneType.msg"
  "msg/SensorMeasurement.msg"
  msg/AbnormalStatus.msg
  msg/BarrierType.msg
  msg/BoolTag.msg
  msg/Box2D.msg
  msg/Box3D.msg
  msg/CameraObjectSupplement.msg
  msg/CameraObjectTrackState.msg
  msg/CameraRawObject.msg
  msg/CarLight.msg
  msg/CLInfo.msg
  msg/CLStatus.msg
  msg/CLType.msg
  msg/ConfidenceType.msg
  msg/HeadingStatus.msg
  msg/LidarObjectSupplement.msg
  msg/LidarCustomObjectMsg.msg
  msg/LidarVectorMsg.msg
  msg/LocatedInLaneType.msg
  msg/MotionState.msg
  msg/ObjectMessage.msg
  msg/ObjectSubType.msg
  msg/ObjectTags.msg
  msg/ObjectType.msg
  msg/ObservedStatus.msg
  msg/ObstacleLaneType.msg
  msg/ObstacleStatus.msg
  msg/OccludedStatus.msg
  msg/ParsingType.msg
  msg/Point2D.msg
  msg/Point3D.msg
  msg/PointCloud.msg
  msg/PointD.msg
  msg/PointXYZIT.msg
  msg/PolygonDType.msg
  msg/ProcessStage.msg
  msg/RadarObjectSupplement.msg
  msg/RadarRoiType.msg
  msg/RadarSegmentationType.msg
  msg/RawMotionState.msg
  msg/RoadState.msg
  msg/SensorFrame.msg
  msg/SensorFrameMessage.msg
  msg/SensorInfo.msg
  msg/SensorOrientation.msg
  msg/SensorType.msg
  msg/SpecialVehicleStatus.msg
  msg/SpecialVehicleType.msg
  msg/SubtypeSizeRelation.msg
  msg/Vector2D.msg
  msg/Vector3D.msg
  msg/VisualObjectType.msg
  DEPENDENCIES common_msgs
  ${msg_dependencies}
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()