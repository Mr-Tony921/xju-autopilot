/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/path_decider/path_decider.h"

#include "object_decision.pb.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

PathDecider::PathDecider(const pnc::TaskConfig& config,
                         const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {
  vehicle_config_ = xju::pnc::VehicleConfigProvider::GetConfig();
}

void PathDecider::Init(const pnc::TaskConfig& config) { config_ = config; }

void PathDecider::Reset() {}

bool PathDecider::Process(
    std::shared_ptr<ReferenceLineInfo> const reference_line_info,
    Frame* const frame) {
  Task::Process(reference_line_info, frame);
  std::string blocking_obstacle_id;
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->id();
  }
  ADEBUG << "blocking_obstacle_id = " << blocking_obstacle_id;
  if (!MakeObjectDecision(reference_line_info->path_data(),
                          blocking_obstacle_id,
                          reference_line_info->path_decision())) {
    AERROR << "Failed to make decision";
    return false;
  }
  if (AddReferencelineEndStop(frame, reference_line_info) == -1) {
    AERROR << "Creation of refernce_line_end_stop wall is failed ";
  }
  if (AddPathEndStop(frame, reference_line_info) == -1) {
    AERROR << "Creation of path_end_stop wall is failed ";
    return false;
  }
  if (AddDestinationStop(frame, reference_line_info) == -1) {
    AERROR << "Creation of destination_stop wall is failed ";
    return false;
  }
  return true;
}

bool PathDecider::MakeObjectDecision(const PathData& path_data,
                                     const std::string& blocking_obstacle_id,
                                     PathDecision* const path_decision) {
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(
    const PathData& path_data, const std::string& blocking_obstacle_id,
    PathDecision* const path_decision) {
  // Sanity checks and get important values.
  ACHECK(path_decision);
  const auto frenet_path = path_data.frenet_points();
  if (frenet_path.empty()) {
    AERROR << "planned Path is empty.";
    return false;
  }
  const double half_width = vehicle_config_.width() / 2.0;
  const double lateral_radius =
      half_width + config_.path_decider_config().lateral_ignore_buffer();  // 3m
  ADEBUG << "half width of vehicle: " << half_width;
  ADEBUG << "lateral radius: " << lateral_radius;
  // Go through every obstacle and make decisions.
  ADEBUG << "obstacles size: " << path_decision->obstacles().Items().size();
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (!obstacle) {
      continue;
    }
    ADEBUG << "obstacle id: " << obstacle->id() << ", "
           << (obstacle->is_static() ? "is static." : "is not static.");
    const std::string& obstacle_id = obstacle->id();
    if (!obstacle->is_static() || obstacle->is_virtual()) {
      continue;
    }
    // - skip decision making for obstacles with IGNORE/STOP decisions already.
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->longitudinal_decision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->lateral_decision().has_ignore()) {
      continue;
    }
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->longitudinal_decision().has_stop()) {
      // STOP decision
      continue;
    }
    if (obstacle->id() == blocking_obstacle_id) {
      // Add stop decision
      ADEBUG << "Blocking obstacle id: " << blocking_obstacle_id;
      ObjectDecisionType object_decision;
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      ADEBUG << "stop distance is: "
             << object_decision.mutable_stop()->distance_m();
      ADEBUG << "stop point coordinate (x, y) = ("
             << object_decision.mutable_stop()->mutable_stop_point()->x()
             << " ,"
             << object_decision.mutable_stop()->mutable_stop_point()->y()
             << ")";
      path_decision->SetLongitudinalDecision(obstacle->id(), object_decision);
      continue;
    }

    // IGNORE by default and if obstacle is not in path s at all.
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    const auto& sl_boundary = obstacle->sl_boundary();
    ADEBUG << "start_s of obstacle is: " << sl_boundary.start_s();
    ADEBUG << "end_s of obstacle is: " << sl_boundary.end_s();
    ADEBUG << "start_s of planned path is: " << frenet_path.front().s();
    ADEBUG << "end_s of planned path is:  = " << frenet_path.back().s();
    if (sl_boundary.end_s() < frenet_path.front().s() ||
        sl_boundary.start_s() > frenet_path.back().s()) {
      path_decision->SetLongitudinalDecision(obstacle->id(), object_decision);
      path_decision->SetLateralDecision(obstacle->id(), object_decision);
      continue;
    }

    const auto frenet_point = GetNearestPoint(frenet_path, sl_boundary);
    const double curr_l = frenet_point.l();
    double min_nudge_l =
        half_width + config_.path_decider_config().static_obstacle_buffer();
    ADEBUG << "min_nudge_l = " << min_nudge_l;
    ADEBUG << "curr_l = " << curr_l;
    ADEBUG << "start_l of obstacle is: " << sl_boundary.start_l();
    ADEBUG << "end_l of obstacle is: " << sl_boundary.end_l();
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // IGNORE if laterally too far away.
      object_decision.mutable_ignore();
      const auto& sl_boundary = obstacle->sl_boundary();
      path_decision->SetLateralDecision(obstacle->id(), object_decision);
    } else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
               sl_boundary.start_l() <= curr_l + min_nudge_l) {
      AINFO << "obstacle is too close to vehicle: STOP";
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      ADEBUG << "stop distance is: "
             << object_decision.mutable_stop()->distance_m();
      ADEBUG << "stop point coordinate (x, y) = ("
             << object_decision.mutable_stop()->mutable_stop_point()->x()
             << " ,"
             << object_decision.mutable_stop()->mutable_stop_point()->y()
             << ")";
      path_decision->SetLongitudinalDecision(obstacle->id(), object_decision);
    } else {
      // NUDGE if laterally very close.
      if (sl_boundary.end_l() < curr_l - min_nudge_l) {
        ADEBUG << "LEFT NUDGE";
        ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        path_decision->SetLateralDecision(obstacle->id(), object_decision);
      } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {
        ADEBUG << "RIGHT NUDGE";
        ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        path_decision->SetLateralDecision(obstacle->id(), object_decision);
      }
    }
  }
  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle& obstacle) const {
  ObjectStop object_stop;
  object_stop.set_distance_m(-config_.path_decider_config().stop_distance());
  ADEBUG << "start_s of obstacle is: " << obstacle.sl_boundary().start_s();
  const double stop_ref_s = obstacle.sl_boundary().start_s() -
                            config_.path_decider_config().stop_distance();
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.theta());
  return object_stop;
}

int PathDecider::AddPathEndStop(
    Frame* const frame,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info) {
  if (reference_line_info->path_data().frenet_points().size() == 0) {
    return 0;
  }
  if (reference_line_info->path_data().frenet_points().back().s() -
          reference_line_info->path_data().frenet_points().front().s() <
      config_.path_decider_config().short_path_length_threshold()) {
    const std::string stop_wall_id =
        PATH_END_VO_ID_PREFIX + reference_line_info->reference_line().id();
    ADEBUG << "stop_wall_id = " << stop_wall_id;
    const double stop_s =
        vehicle_config_.wheel_base() +
        vehicle_config_.front_overhang_length() + 1.0;
    const double path_end_obstable_s = std::fmax(
        0.0,
        reference_line_info->path_data().frenet_points().back().s() - stop_s);
    ADEBUG << "stop_line_s = "
           << reference_line_info->path_data().frenet_points().back().s() -
                  stop_s;
    int status = BuildStopDecision(stop_wall_id, path_end_obstable_s, 0.0,
                                   frame, reference_line_info);
    if (status < 0) {
      AERROR << "Create path_end stop wall failure";
      return -1;
    }
  }
  return 0;
}

int PathDecider::AddDestinationStop(
    Frame* const frame,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info) {
  if (!frame->NearDestination()) {
    return 0;
  }
  const pnc::Vec2d destination = frame->destination();
  pnc::SLPoint vehicle_destination_sl;
  reference_line_info->reference_line().XYToSL(destination,
                                               &vehicle_destination_sl);
  const double dest_lane_s =
      std::fmax(0.0, vehicle_destination_sl.s() - FLAGS_virtual_stop_wall_length);
  ADEBUG << "destination.x = " << destination.x();
  ADEBUG << "destination.y = " << destination.y();
  ADEBUG << "dest_lane_s = " << dest_lane_s;
  const std::string stop_wall_id =
      DESTINATION_VO_ID_PREFIX + reference_line_info->reference_line().id();
  ADEBUG << "stop_wall_id = " << stop_wall_id;
  int status = BuildStopDecision(
      stop_wall_id, dest_lane_s,
      config_.path_decider_config().virtual_obstacle_stop_distance(), frame,
      reference_line_info);
  if (status < 0) {
    AERROR << "Create destination stop wall failure";
    return -1;
  }
  return 0;
}

int PathDecider::AddReferencelineEndStop(
    Frame* const frame,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info) {
  double remain_s = reference_line_info->reference_line().length() -
                    reference_line_info->car_sl_boundary().end_s();
  ADEBUG << "remain_s = " << remain_s;
  if (remain_s >
      config_.path_decider_config().min_reference_line_remain_length()) {
    return 0;
  }
  const double obstacle_start_s =
      std::fmax(0.0, reference_line_info->reference_line().length() -
                         FLAGS_virtual_stop_wall_length);
  const std::string stop_wall_id = REFERENCELINE_END_VO_ID_PREFIX +
                                   reference_line_info->reference_line().id();
  ADEBUG << "stop_wall_id = " << stop_wall_id;
  int status = BuildStopDecision(
      stop_wall_id, obstacle_start_s,
      config_.path_decider_config().virtual_obstacle_stop_distance(), frame,
      reference_line_info);
  if (status < 0) {
    AERROR << "Create referenceline_end stop wall failure";
    return -1;
  }
  return 0;
}

int PathDecider::BuildStopDecision(
    const std::string& stop_wall_id, const double stop_line_s,
    const double stop_distance, Frame* const frame,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info) {
  // check
  ADEBUG << "stop_line_s = " << stop_line_s;
  ADEBUG << "stop_distance = " << stop_distance;
  const auto& reference_line = reference_line_info->reference_line();
  if (stop_line_s < 0 || stop_line_s > reference_line.length()) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id, stop_line_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  ADEBUG << "obstacle id: " << obstacle->id() << ", is "
         << (obstacle->is_virtual() ? "Virtual." : "not Virtual.") << " is  "
         << (obstacle->is_static() ? "static." : "not static.")
         << ",x: " << obstacle->perception_obstacle().position().x()
         << ",y: " << obstacle->perception_obstacle().position().y()
         << ",heading: " << obstacle->perception_obstacle().heading()
         << ",speed: " << obstacle->perception_obstacle().velocity().x()
         << ",length: " << obstacle->perception_obstacle().length()
         << ",width: " << obstacle->perception_obstacle().width();

  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }
  ADEBUG << "stop_wall_start_s = " << stop_wall->sl_boundary().start_s();
  ADEBUG << "stop_wall_end_s = " << stop_wall->sl_boundary().end_s();
  ADEBUG << "stop_wall_start_l = " << stop_wall->sl_boundary().start_l();
  ADEBUG << "stop_wall_end_l = " << stop_wall->sl_boundary().end_l();
  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  ADEBUG << "stop_s = " << stop_s;
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

pnc::FrenetFramePoint PathDecider::GetNearestPoint(
    const std::vector<pnc::FrenetFramePoint>& frenet_path,
    const SLBoundary& sl) const {
  auto LowerBoundComparator = [](const pnc::FrenetFramePoint& p,
                                 const double s) { return p.s() < s; };

  auto it_lower = std::lower_bound(frenet_path.begin(), frenet_path.end(),
                                   sl.start_s(), LowerBoundComparator);
  if (it_lower == frenet_path.end()) {
    return frenet_path.back();
  }

  auto UpperBoundComparator =
      [](const double s, const pnc::FrenetFramePoint& p) { return p.s() > s; };

  auto it_upper = std::upper_bound(it_lower, frenet_path.end(), sl.end_s(),
                                   UpperBoundComparator);
  double min_dist = std::numeric_limits<double>::max();
  auto min_it = it_upper;
  for (auto it = it_lower; it != it_upper; ++it) {
    if (it->l() >= sl.start_l() && it->l() <= sl.end_l()) {
      return *it;
    } else if (it->l() > sl.end_l()) {
      double diff = it->l() - sl.end_l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    } else {
      double diff = sl.start_l() - it->l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    }
  }
  return *min_it;
}

}  // namespace planning
}  // namespace xju
