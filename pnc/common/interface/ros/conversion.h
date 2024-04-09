/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

// common message
#include "common_msgs/msg/header.hpp"
#include "common_msgs/msg/path_point.hpp"
#include "common_msgs/msg/point2_d.hpp"
#include "common_msgs/msg/point3_d.hpp"
#include "common_msgs/msg/trajectory_point.hpp"
#include "common_msgs/msg/vector3_d.hpp"
#include "header.pb.h"
#include "pnc_point.pb.h"

// chassis message
#include "chassis.pb.h"
#include "chassis_msgs/msg/brake_info.hpp"
#include "chassis_msgs/msg/gear_box_info.hpp"
#include "chassis_msgs/msg/steer_info.hpp"
#include "chassis_msgs/msg/throttle_info.hpp"
#include "chassis_msgs/msg/car_mass_info.hpp"
#include "chassis_msgs/msg/car_speed_info.hpp"

// control message
#include "control.pb.h"
#include "control_msgs/msg/control.hpp"

// localization message
#include "localization.pb.h"
#include "localization_msgs/msg/localize_output.hpp"

// prediction message
#include "perception_em_msgs/msg/prediction_obstacles.hpp"
#include "prediction_obstacle.pb.h"

// planning message
#include "planning.pb.h"
#include "planning_msgs/msg/planning.hpp"

// emlanes message
#include "em_lane.pb.h"
#include "perception_em_msgs/msg/em_lanes.hpp"

// chart message
#include "chart_msgs/msg/axis.hpp"
#include "chart_msgs/msg/chart.hpp"
#include "chart_msgs/msg/color_rgba.hpp"
#include "chart_msgs/msg/debug.hpp"
#include "chart_msgs/msg/double_variable_pari.hpp"
#include "chart_msgs/msg/line.hpp"
#include "chart_msgs/msg/options.hpp"
#include "chart_msgs/msg/polygon.hpp"
#include "chart_msgs/msg/string_variable_pari.hpp"
#include "debug.pb.h"

namespace xju {
namespace pnc {

// common meesage converison
void HeaderConversion(const common_msgs::msg::Header& msg, Header* const proto);

void HeaderConversion(const Header& proto, common_msgs::msg::Header* const msg);

void Point2DConversion(const common_msgs::msg::Point2D& msg,
                       Point2D* const proto);

void Point2DConversion(const Point2D& proto, common_msgs::msg::Point2D* msg);

void Vector3DConversion(const common_msgs::msg::Vector3D& msg,
                        Point3D* const proto);

void Point3DConversion(const common_msgs::msg::Point3D& msg,
                       Point3D* const proto);

void PathPointConversion(const common_msgs::msg::PathPoint& msg,
                         PathPoint* const proto);

void PathPointConversion(const PathPoint& proto,
                         common_msgs::msg::PathPoint* const msg);

void TrajectoryPointConversion(const common_msgs::msg::TrajectoryPoint& msg,
                               TrajectoryPoint* const proto);

void TrajectoryPointConversion(const TrajectoryPoint& proto,
                               common_msgs::msg::TrajectoryPoint* const msg);

// chassis info message conversion
void BrakeInfoConversion(const chassis_msgs::msg::BrakeInfo::SharedPtr msg_ptr,
                         BrakeInfo* const proto_ptr);

void ThrottleInfoConversion(
    const chassis_msgs::msg::ThrottleInfo::SharedPtr msg_ptr,
    ThrottleInfo* const proto_ptr);

void SteerInfoConversion(const chassis_msgs::msg::SteerInfo::SharedPtr msg_ptr,
                         SteerInfo* const proto_ptr);

void CarSpeedInfoConversion(
    const chassis_msgs::msg::CarSpeedInfo::SharedPtr msg_ptr,
    CarSpeedInfo* const proto_ptr);

void CarMassInfoConversion(
    const chassis_msgs::msg::CarMassInfo::SharedPtr msg_ptr,
    CarMassInfo* const proto_ptr);

void GearBoxInfoConversion(
    const chassis_msgs::msg::GearBoxInfo::SharedPtr msg_ptr,
    GearBoxInfo* const proto_ptr);

// control message conversion
void ControlConversion(const control_msgs::msg::Control::SharedPtr msg,
                       control::ControlCommand* const proto);

void ControlConversion(const control::ControlCommand& proto,
                       control_msgs::msg::Control* const msg);

// localization message conversion
void QuaternionConversion(const common_msgs::msg::Quaternion& msg,
                          Quaternion* const proto);

void LocalizationConversion(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg_ptr,
    Localization* const proto_ptr);

void GlobalLocalizationConversion(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg_ptr,
    Localization* const proto_ptr);

// prediction message conversion
void PerceptionObstacleConversion(const perception_em_msgs::msg::EmObstacle& msg,
                                  PerceptionObstacle* const proto_ptr);

void PredicitonTrajectoryConversion(
    const perception_em_msgs::msg::PredictionTrajectory& msg,
    PredictionTrajectory* const proto_ptr);

void PredictionObstacleConversion(
    const perception_em_msgs::msg::PredictionObstacle& msg,
    PredictionObstacle* const proto_ptr);

void PredictionConversion(
    const perception_em_msgs::msg::PredictionObstacles::SharedPtr msg_ptr,
    PredictionObstacles* const proto_ptr);

// planning message conversion
void PlanningConversion(const planning::ADCTrajectory& proto,
                        planning_msgs::msg::Planning* const msg_ptr);

void PlanningConversion(const planning_msgs::msg::Planning::SharedPtr msg_ptr,
                        planning::ADCTrajectory* const proto);

// emlanes message conversion
void EmLanesConversion(const perception_em_msgs::msg::EmLanes::SharedPtr msg_ptr,
                       EmLanes* proto_ptr);

// debug and chart message conversion
void DebugConversion(const Debug& proto, chart_msgs::msg::Debug* const msg);

void ChartConversion(const Chart& proto, chart_msgs::msg::Chart* const msg);

void LineConversion(const Line& proto, chart_msgs::msg::Line* const msg);

void PolygonConversion(const Polygon& proto,
                       chart_msgs::msg::Polygon* const msg);

void OptionsConversion(const Options& proto,
                       chart_msgs::msg::Options* const msg);

void AxisConversion(const Axis& proto, chart_msgs::msg::Axis* const msg);

void RGBAConverison(const std::string& color,
                    chart_msgs::msg::ColorRGBA* const msg);

void LineStyleConversion(const std::string& style, uint8_t* msg);

}  // namespace pnc
}  // namespace xju
