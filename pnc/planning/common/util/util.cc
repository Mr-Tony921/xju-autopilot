/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/util/util.h"

namespace xju {
namespace planning {
namespace util {

int BuildStopDecision(
    const std::string& stop_wall_id, const double stop_line_s,
    const double stop_distance, Frame* const frame,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  // ADEBUG << "stop_line_s = " << stop_line_s;
  // ADEBUG << "stop_distance = " << stop_distance;
  const auto& reference_line = reference_line_info->reference_line();
  if (stop_line_s < 0 || stop_line_s > reference_line.length()) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id, stop_line_s);
  // ADEBUG << "obstacle->id() = " << obstacle->id();
  // ADEBUG << "obstacle->isVirtual() = " << obstacle->is_virtual();
  // ADEBUG << "obstacle->is_static() = " << obstacle->is_static();
  // ADEBUG << "perception_obstacle().position().x"
  //        << obstacle->perception_obstacle().position().x();
  // ADEBUG << "perception_obstacle().position().y"
  //        << obstacle->perception_obstacle().position().y();
  // ADEBUG << "perception_obstacle().heading = "
  //        << obstacle->perception_obstacle().heading();
  // ADEBUG << "perception_obstacle().velocity = "
  //        << obstacle->perception_obstacle().velocity().x();
  // ADEBUG << "perception_obstacle().length = "
  //        << obstacle->perception_obstacle().length();
  // ADEBUG << "perception_obstacle().width = "
  //        << obstacle->perception_obstacle().width();

  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }
  // ADEBUG << "start_s = " << stop_wall->sl_boundary().start_s();
  // ADEBUG << "end_s = " << stop_wall->sl_boundary().end_s();
  // ADEBUG << "start_l = " << stop_wall->sl_boundary().start_l();
  // ADEBUG << "end_l = " << stop_wall->sl_boundary().end_l();
  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  // AINFO << "stop_s = " << stop_s;
  const auto& stop_point = reference_line.GetReferencePoint(stop_s);
  const double stop_heading = reference_line.GetReferencePoint(stop_s).theta();

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_distance_m(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);
  auto* path_decision = reference_line_info->path_decision();
  path_decision->SetLongitudinalDecision(stop_wall->id(), stop);
  return 0;
}

}  // namespace util
}  // namespace planning
}  // namespace xju