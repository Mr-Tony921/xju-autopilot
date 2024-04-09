#pragma once

#include <string>

namespace {

//perception_nn
const std::string kTopicPerceptionNnLod = "/perception/nn/lod";

//perception_em
const std::string kTopicPerceptionEmRadarObs = "/perception/em/inner/obstacles";
const std::string kTopicPerceptionEmLidarCluster = "/perception/em/inner/lidar_cluster";
const std::string kTopicPerceptionEmLidarTrack = "/perception/em/inner/obstacles";
const std::string kTopicPerceptionEmFusionObs = "/perception/em/obstacles";
const std::string kTopicPerceptionEmLanes = "/perception/em/lanes";
const std::string kTopicPerceptionEmPrediction = "/perception/em/prediction";

//localization
const std::string kTopicLocalizationLidarGlobalLoc = "/localization/inner/lidar_gloc";
const std::string kTopicLocalizationOutput = "/localization/output";
const std::string kTopicMapRouting = "/map/routing";

//pnc
const std::string kTopicPncPlanning = "/pnc/planning";
const std::string kTopicPncControl = "/pnc/control";

//sw
const std::string kTopicDataInterfaceChassisThrottle = "/data_interface/chassis/throttle";
const std::string kTopicDataInterfaceChassisBrake = "/data_interface/chassis/brake";
const std::string kTopicDataInterfaceChassisSteer = "/data_interface/chassis/steer";
const std::string kTopicDataInterfaceChassisGear = "/data_interface/chassis/gear";
const std::string kTopicDataInterfaceChassisVehicleSpeed = "/data_interface/chassis/vehicle_speed";
const std::string kTopicDataInterfaceChassisWheelSpeed = "/data_interface/chassis/wheel_speed";
const std::string kTopicDataInterfaceChassisVehicleMass = "/data_interface/chassis/vehicle_mass";

const std::string kTopicDataInterfaceSensorFrontRadar = "/data_interface/sensor/front_radar";
const std::string kTopicDataInterfaceSensorLeftFrontRadar = "/data_interface/sensor/leftfront_radar";
const std::string kTopicDataInterfaceSensorRightFrontRadar = "/data_interface/sensor/rightfront_radar";
const std::string kTopicDataInterfaceSensorLeftRearRadar = "/data_interface/sensor/leftrear_radar";
const std::string kTopicDataInterfaceSensorRightRearRadar = "/data_interface/sensor/rightrear_radar";
const std::string kTopicDataInterfaceSensorFrontLidar = "/data_interface/sensor/front_lidar/pointcloud";
const std::string kTopicDataInterfaceSensorGnss = "/data_interface/sensor/gps_data";
const std::string kTopicDataInterfaceSensorImu = "/data_interface/sensor/imu_data";

const std::string kTopicDiagnosis = "/diagnosis_data";

//mcu_data
const std::string kTopicMcuDataSourceImu = "/mcu_data_source/imu";
const std::string kTopicMcuDataSourceImu = "/mcu_data_source/gnss";

} // namespace
