/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/frame/frame.h"

#include <cmath>
#include <limits>

#include "common/logger/logger.h"
#include "common/thread_pool/thread_pool.h"
#include "common/time/time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/util/coordinate.h"
#include "common/util/vehicle_helper.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

namespace {
using ReferenceLine = pnc_map::ReferenceLine;
}  // namespace
std::string Frame::last_ego_lane_id_;

bool Frame::Init(const double current_timestamp,
                 const pnc::TrajectoryPoint& planning_start_point,
                 const pnc::VehicleState& vehicle_state,
                 const std::list<ReferenceLine>& reference_lines,
                 const LocalView& local_view) {
  const double time_1 = pnc::Time::NowInSeconds();
  planning_start_time_ = current_timestamp;
  planning_start_point_ = planning_start_point;
  vehicle_state_ = vehicle_state;
  local_view_ = local_view;

  Reset();
  if (!InitFrameData()) {
    return false;
  }
  if (!BuildReferenceLineInfo(reference_lines)) {
    return false;
  }
  CalculationReferenceLineInfo();
  BuildRoadChangeInfo();
  const double time_2 = pnc::Time::NowInSeconds();
  ADEBUG << "cost time for frame init is: " << (time_2 - time_1) * 1e3 << " ms";
  return true;
}

bool Frame::InitFrameData() {
  if (!pnc::VehicleHelper::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Vehicle state is not valided.";
    return false;
  }
  AlignPredictionTime();
  for (auto& ptr : Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);
  }
  DestinationConvert();
  return true;
}

void Frame::AlignPredictionTime() {
  auto prediction_obstacles = local_view_.prediction_obstacles;
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }

  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();
  for (auto& obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    for (auto& prediction_trajectory :
         *obstacle.mutable_prediction_trajectory()) {
      auto& trajectory = *prediction_trajectory.mutable_trajectory();
      for (auto& point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time_);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

bool Frame::BuildReferenceLineInfo(
    const std::list<ReferenceLine>& reference_lines) {
  const double time_1 = pnc::Time::NowInSeconds();
  reference_line_infos_.clear();

  std::vector<std::future<std::shared_ptr<ReferenceLineInfo>>> futures;
  auto pool = xju::pnc::ThreadPool::Instance();
  auto init_ref_info =
      [](std::shared_ptr<ReferenceLineInfo> reference_line_info_ptr,
         const decltype(obstacles())& obs)
      -> std::shared_ptr<ReferenceLineInfo> {
    if (reference_line_info_ptr->Init(obs)) {
      return reference_line_info_ptr;
    } else {
      return nullptr;
    }
  };

  for (const ReferenceLine& reference_line : reference_lines) {
    auto reference_line_info = std::make_shared<ReferenceLineInfo>(
        vehicle_state_, planning_start_point_, reference_line);

    if (reference_line.is_recommended_lane()) {
      recommended_lane_id_ = reference_line.id();
    }

    futures.push_back(
        pool->submit(init_ref_info, reference_line_info, obstacles()));
    // if (reference_line_info->Init(obstacles())) {
    //   reference_line_infos_.push_back(reference_line_info);
    // }
  }

  for (auto& future : futures) {
    auto res = future.get();
    if (res) {
      reference_line_infos_.push_back(res);
    }
  }

  double min_dis = std::numeric_limits<double>::infinity();
  pnc::Vec2d vehicle_xy(planning_start_point_.path_point().x(),
                      planning_start_point_.path_point().y());

  pnc::SLPoint vehicle_sl;
  for (const auto& reference_line_info : reference_line_infos_) {
    if (!reference_line_info->reference_line().XYToSL(vehicle_xy, &vehicle_sl)) {
      continue;
    }

    if (!last_ego_lane_id_.empty() &&
        std::fabs(vehicle_sl.l()) < FLAGS_keep_lane_id_range &&
        last_ego_lane_id_ == reference_line_info->reference_line().id()) {
      min_dis = vehicle_sl.l();
      ego_lane_id_ = reference_line_info->reference_line().id();
      break;
    }

    if (std::fabs(vehicle_sl.l()) < min_dis) {
      min_dis = std::fabs(vehicle_sl.l());
      ego_lane_id_ = reference_line_info->reference_line().id();
    }
  }
  ADEBUG << "cost time for BuildReferenceLineInfo is: "
         << (pnc::Time::NowInSeconds() - time_1) * 1e3 << " ms";
  return !ego_lane_id_.empty();
}

const std::shared_ptr<ReferenceLineInfo> Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  std::shared_ptr<ReferenceLineInfo> drive_reference_line_info = nullptr;
  for (const auto& reference_line_info : reference_line_infos_) {
    if (reference_line_info->drivable() &&
        reference_line_info->cost() < min_cost) {
      drive_reference_line_info = reference_line_info;
      min_cost = reference_line_info->cost();
    }
  }
  return drive_reference_line_info;
}

void Frame::CalculationReferenceLineInfo() {
  const double time_1 = pnc::Time::NowInSeconds();

  std::string left_id, right_id;
  auto cur_iter =
      std::find_if(reference_line_infos_.begin(), reference_line_infos_.end(),
                   [&](std::shared_ptr<xju::planning::ReferenceLineInfo>& ref) {
                     return ref->reference_line().id() == ego_lane_id_;
                   });
  if (cur_iter != reference_line_infos_.end()) {
    cur_reference_line_info_ = *cur_iter;
    left_id = cur_reference_line_info_->reference_line().left_lane_id();
    right_id = cur_reference_line_info_->reference_line().right_lane_id();
    last_ego_lane_id_ = ego_lane_id_;
  } else {
    last_ego_lane_id_.clear();
  }

  left_reference_line_info_ = nullptr;
  right_reference_line_info_ = nullptr;
  for (auto const& reference_line_info : reference_line_infos_) {
    if (reference_line_info->reference_line().id() == left_id) {
      left_reference_line_info_ = reference_line_info;
    }
    if (reference_line_info->reference_line().id() == right_id) {
      right_reference_line_info_ = reference_line_info;
    }
  }

  const double time_2 = pnc::Time::NowInSeconds();
  ADEBUG << "cost time for CalculationReferenceLineInfo is: "
         << (time_2 - time_1) * 1e3 << " ms";
}

const Obstacle* Frame::CreateStopObstacle(
    std::shared_ptr<ReferenceLineInfo> const reference_line_info,
    const std::string& obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto& reference_line = reference_line_info->reference_line();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).theta();
  static constexpr double kStopWallWidth = 4.0;
  pnc::Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                           kStopWallWidth};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

const Obstacle* Frame::CreateStaticVirtualObstacle(const std::string& id,
                                                   const pnc::Box2d& box) {
  const auto* object = obstacles_.Find(id);
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto* ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

void Frame::AddObstacle(const Obstacle& obstacle) {
  obstacles_.Add(obstacle.id(), obstacle);
}

const std::vector<const Obstacle*> Frame::obstacles() const {
  return obstacles_.Items();
}

const double Frame::planning_start_time() const { return planning_start_time_; }

const std::string& Frame::ego_lane_id() const { return ego_lane_id_; }

const std::string& Frame::recommended_lane_id() const {
  return recommended_lane_id_;
}

const pnc::VehicleState& Frame::vehicle_state() const { return vehicle_state_; }

const pnc::TrajectoryPoint& Frame::planning_start_point() const {
  return planning_start_point_;
}

const bool Frame::NearDestination() const {
  if (!local_view_.routing->valid()) {
    return false;
  }
  if (!cur_reference_line_info_) {
    return false;
  }

  const pnc::Vec2d destination_xy = destination();
  pnc::SLPoint destination_sl;
  bool status = cur_reference_line_info_->reference_line().XYToSL(
      destination_xy, &destination_sl);
  if (!status) {
    return false;
  }
  ADEBUG << "local_view_.routing->destination().x() = "
         << local_view_.routing->destination().x();
  ADEBUG << "local_view_.routing->destination().y() = "
         << local_view_.routing->destination().y();
  ADEBUG << "local_destination_x = " << destination_xy.x();
  ADEBUG << "local_destination_y = " << destination_xy.y();
  double distance =
      std::hypot(destination_xy.x() - vehicle_state_.x(),
                 destination_xy.y() - vehicle_state_.y());
  ADEBUG << "distance = " << distance;
  ADEBUG << "destination_sl.s() = " << destination_sl.s();
  ADEBUG << "destination_sl.l() = " << destination_sl.l();
  ADEBUG << "reference_line().length()"
         << cur_reference_line_info_->reference_line().length();
  if (distance < 200 && destination_sl.l() < 10 &&
      destination_sl.s() <
          cur_reference_line_info_->reference_line().length()) {
    ADEBUG << "near destination";
    return true;
  } else {
    return false;
  }
}

void Frame::DestinationConvert() {
  auto localize_pose = pnc::VehicleStateProvider::localize_pose();
  double destination_x = local_view_.routing->destination().x();
  double destination_y = local_view_.routing->destination().y();
  xju::pnc::TransformLocalToVehicleCoord(
      local_view_.localization->gloal_localize().x(),
      local_view_.localization->gloal_localize().y(),
      local_view_.localization->gloal_localize().theta(), &destination_x,
      &destination_y);
  destination_xy_.set_x(destination_x);
  destination_xy_.set_y(destination_y);
}

const pnc::Vec2d& Frame::destination() const { return destination_xy_; }

void Frame::BuildRoadChangeInfo() {
  if (!local_view_.routing->mutable_header()->has_timestamp_sec() ||
      pnc::Time::NowInSeconds() -
              local_view_.routing->mutable_header()->timestamp_sec() >
          1.0) {
    return;
  }

  change_point_xy_.set_x(local_view_.routing->change_point().x());
  change_point_xy_.set_y(local_view_.routing->change_point().y());
  change_direction_ = local_view_.routing->direction();

  lane_ids_.clear();
  if (cur_reference_line_info_) {
    lane_ids_.emplace_back(cur_reference_line_info_->reference_line().id());
  }
  switch (change_direction_) {
    case 1:
      if (left_reference_line_info_) {
        lane_ids_.emplace_back(
            left_reference_line_info_->reference_line().id());
      }
      break;
    case 2:
      if (right_reference_line_info_) {
        lane_ids_.emplace_back(
            right_reference_line_info_->reference_line().id());
      }
      break;
    case 0:
    case 3:
    default:
      break;
  }
}

const double Frame::DistToChangePoint() const {
  auto equal = [](double x, double y) { return std::abs(x - y) < 1e-6; };
  if (equal(0, change_point_xy_.x()) && equal(0, change_point_xy_.y())) {
    return std::numeric_limits<double>::max();
  }

  auto localize_pose = pnc::VehicleStateProvider::localize_pose();
  double local_x = change_point_xy_.x();
  double local_y = change_point_xy_.y();
  xju::pnc::TransformLocalToVehicleCoord(
      local_view_.localization->gloal_localize().x(),
      local_view_.localization->gloal_localize().y(),
      local_view_.localization->gloal_localize().theta(), &local_x, &local_y);

  auto distance =
      std::hypot(local_x - vehicle_state_.x(),
                 local_y - vehicle_state_.y());
  if (!cur_reference_line_info_) {
    return distance;
  }

  pnc::Vec2d point_xy(local_x, local_y);
  pnc::Vec2d vehicle_xy(vehicle_state_.x(),
                      vehicle_state_.y());
  pnc::SLPoint point_sl, vehicle_sl;
  if (!cur_reference_line_info_->reference_line().XYToSL(point_xy, &point_sl) ||
      !cur_reference_line_info_->reference_line().XYToSL(vehicle_xy, &vehicle_sl)) {
    return distance;
  }

  if (distance < 200 && point_sl.l() < 10 &&
      point_sl.s() < cur_reference_line_info_->reference_line().length()) {
    auto s_diff = point_sl.s() - vehicle_sl.s();
    ADEBUG << "extremely close to change point " << s_diff;
    return s_diff;
  }

  return distance;
}

const std::vector<std::string>& Frame::LanesToChangePoint() const {
  return lane_ids_;
}

void Frame::Reset() {
  ego_lane_id_.clear();
  recommended_lane_id_.clear();
}

}  // namespace planning
}  // namespace xju
