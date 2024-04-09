/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/planning.h"

#include <iostream>

#include "common/file/file.h"
#include "common/logger/logger.h"
#include "common/time/time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/common//trajectory_stitcher/trajectory_stitcher.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/tasks/task_factory.h"

namespace xju {
namespace planning {

namespace {
using TrajectoryPoint = pnc::TrajectoryPoint;
using ReferenceLine = pnc_map::ReferenceLine;
using ReferenceLineProvider = pnc_map::ReferenceLineProvider;
}  // namespace

bool Planning::Init(const std::string& proto_config_file_path) {
  Init();
  pnc::File::GetProtoConfig<PlanningConfig>(proto_config_file_path, &config_);
  TaskFactory::Init(config_, internal_);
  if (config_.has_vehicle_state_config()) {
    pnc::VehicleStateProvider::Init(config_.vehicle_state_config());
  } else {
    AERROR << "Loss vehicle State Config.";
    return false;
  }
  scenario_manager_.Init(internal_);
  reference_line_provider_ptr_->Start();
  return true;
}

void Planning::Init() {
  internal_ = std::make_shared<PlanningInternal>();
  frame_ = std::make_unique<Frame>();
  reference_line_provider_ptr_ = std::make_unique<ReferenceLineProvider>();
  internal_->Init();
}

Planning::~Planning() {
  if (reference_line_provider_ptr_) {
    reference_line_provider_ptr_->Stop();
  }
}

bool Planning::Process(const LocalView& local_view,
                       ADCTrajectory* const ptr_trajectory_pb) {
  internal_->clear_debug_info();
  frame_.reset(new Frame());
  if (frame_ == nullptr) {
    AERROR << "Frame ptr is nullptr.";
    GenerateFallbackTrajectory(ptr_trajectory_pb);
    return false;
  }

  local_view_ = local_view;

  const double start_timestamp = pnc::Time::NowInSeconds();
  if (!CheckInput()) {
    AERROR << "LocalView Input Check Failed.";
    GenerateFallbackTrajectory(ptr_trajectory_pb);
    return false;
  }

  bool status = pnc::VehicleStateProvider::Update(
      *(local_view_.localization), *(local_view_.chassis_info));

  if (!status) {
    AERROR << "Vehicle State Provider Update Failed.";
    GenerateFallbackTrajectory(ptr_trajectory_pb);
    return false;
  }

  pnc::VehicleState vehicle_state = pnc::VehicleStateProvider::GetVehicleState();
  pnc::LocalizePose localize_pose = pnc::VehicleStateProvider::localize_pose();
  ADEBUG << "Vehicle localization: " << localize_pose.x << " " << localize_pose.y << " " << localize_pose.heading;
  TransformLastPublishedTrajectory(last_localize_pose_, localize_pose,
                                   internal_->mutable_publishable_trajectory());

  last_localize_pose_ = localize_pose;

  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);

  std::vector<pnc::TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          vehicle_state, start_timestamp, planning_cycle_time,
          internal_->mutable_publishable_trajectory());

  pnc::TrajectoryPoint planning_start_point = stitching_trajectory.back();

  std::list<ReferenceLine> reference_lines;
  status = reference_line_provider_ptr_->CreateReferenceLines(
      local_view_.em_lanes, localize_pose, &reference_lines);
  if (!status) {
    AERROR << "Create Reference lines failed.";
    GenerateFallbackTrajectory(ptr_trajectory_pb);
    return false;
  }

  frame_->Init(start_timestamp, planning_start_point, vehicle_state,
               reference_lines, local_view_);

  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

  if (status) {
    FillPlanningPb(stitching_trajectory.size() == 1, ptr_trajectory_pb);
    internal_->mutable_last_planning_status()->CopyFrom(internal_->planning_status());
    internal_->set_fallback_counter(0);
  } else {
    GenerateFallbackTrajectory(ptr_trajectory_pb);
  }
  return true;
}

bool Planning::CheckInput() {
  if (!local_view_.localization->has_header()) {
    AERROR << "Input localization is not ready!";
    return false;
  } 

  if (!local_view_.chassis_info->brake_info().has_header() ||
      !local_view_.chassis_info->gear_info().has_header() ||
      !local_view_.chassis_info->steer_info().has_header() ||
      !local_view_.chassis_info->throttle_info().has_header() ||
      !local_view_.chassis_info->car_speed_info().has_header() ||
      !local_view_.chassis_info->car_mass_info().has_header()) {
    AERROR << "Input chassis_info is not ready!";
    return false;
  }

  if (!local_view_.em_lanes->has_header()) {
    AERROR << "Input em_lanes is not ready!";
    return false;
  }

  if (!local_view_.prediction_obstacles->has_header()) {
    AERROR << "Input prediction_obstacles is not ready!";
    return false;
  }
  
  const double now = pnc::Time::NowInSeconds();

  if (!FLAGS_ignore_localize_message_delay && 
      (now - local_view_.localization->header().timestamp_sec() >
       FLAGS_localize_msg_delay_threshold)) {
    AERROR << "Input localization is delay! "
           << now - local_view_.localization->header().timestamp_sec()
           << " > " << FLAGS_localize_msg_delay_threshold;
    return false;
  }

  if (!FLAGS_ignore_chassis_message_delay) {
    if (now - local_view_.chassis_info->brake_info().header().timestamp_sec() >
        FLAGS_chassis_msg_delay_threshold) {
      AERROR << "Input brake_info is delay!";
      return false;
    } else if (now - local_view_.chassis_info->gear_info().header().timestamp_sec() >
               FLAGS_chassis_msg_delay_threshold) {
      AERROR << "Input gear_info is delay!";
      return false;
    } else if (now - local_view_.chassis_info->steer_info().header().timestamp_sec() >
               FLAGS_chassis_msg_delay_threshold) {
      AERROR << "Input steer_info is delay!";
      return false;
    } else if (now - local_view_.chassis_info->throttle_info().header().timestamp_sec() >
               FLAGS_chassis_msg_delay_threshold) {
      AERROR << "Input throttle_info is delay!";
      return false;
    } else if (now - local_view_.chassis_info->car_speed_info().header().timestamp_sec() >
               FLAGS_chassis_msg_delay_threshold) {
      AERROR << "Input car_speed_info is delay!";
      return false;
    } else if (now - local_view_.chassis_info->car_mass_info().header().timestamp_sec() >
               FLAGS_chassis_msg_delay_threshold) {
      AERROR << "Input car_mass_info is delay!";
      return false;
    }
  }

  if (!FLAGS_ignore_prediction_message_delay &&
      (now - local_view_.prediction_obstacles->header().timestamp_sec() >
       FLAGS_prediction_msg_delay_threshold)) {
    AERROR << "Input prediction_obstacles is delay!";
    return false;
  }

  if (!FLAGS_ignore_emlanes_message_delay &&
      (now - local_view_.em_lanes->header().timestamp_sec() >
       FLAGS_emlanes_msg_delay_threshold)) {
    AERROR << "Input em_lanes is delay!";
    return false;
  }
  
  if (!FLAGS_ignore_routing_message_delay && !local_view_.routing->has_header()) {
    AERROR << "Input routing is not ready!";
    return false;
  }

  return true;
}

bool Planning::Plan(
    const double timestamp,
    const std::vector<pnc::TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  scenario_manager_.Update(*frame_);
  auto scenario = scenario_manager_.current_scenario();
  Scenario::Status status = scenario->Process(frame_.get());
  if (status == Scenario::Status::UNKNOWN) {
    AERROR << "Scenario Status ERROR!";
    return false;
  }
  auto best_reference_line_info = frame_->FindDriveReferenceLineInfo();
  if (!best_reference_line_info) {
    AERROR << "Do not find best reference line info!";
    return false;
  }
  DiscretizedTrajectory trajectory = best_reference_line_info->trajectory();
  trajectory.PrependTrajectoryPoints(std::vector<TrajectoryPoint>(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1));

  auto publishable_trajectory = PublishableTrajectory(timestamp, trajectory);

  internal_->set_publishable_trajectory(publishable_trajectory);

  publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  return true;
}

void Planning::FillPlanningPb(const bool replan,
                              ADCTrajectory* const ptr_trajectory_pb) {
  double start_time = ptr_trajectory_pb->header().timestamp_sec();
  FillHeader(ptr_trajectory_pb);
  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - ptr_trajectory_pb->header().timestamp_sec();
  for (auto& p : *ptr_trajectory_pb->mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }

  ptr_trajectory_pb->set_is_replan(replan);
  ptr_trajectory_pb->mutable_scenario_status()->CopyFrom(
      internal_->planning_status().scenario_status());
  pnc::TurnSignal turn_signal =
      internal_->planning_status().lane_change_status().turn_signal();
  ptr_trajectory_pb->set_turn_signal(turn_signal);
  ptr_trajectory_pb->set_direction(pnc::Direction::FORWARD);
  if (!frame_->ego_lane_id().empty()) {
    ptr_trajectory_pb->set_lane_id(std::stoi(frame_->ego_lane_id()));
  }

  auto best_reference_line_info = frame_->FindDriveReferenceLineInfo();
  if (best_reference_line_info) {
    ptr_trajectory_pb->set_target_lane_id(
        std::stoi(best_reference_line_info->reference_line().id()));
    ptr_trajectory_pb->set_target_speed(
        best_reference_line_info->cruise_speed());
    ptr_trajectory_pb->set_trajectory_type(
        best_reference_line_info->trajectory_type());
    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_reference_line_info->latency_stats());
  }
  
  const double total_time_ms = (pnc::Time::NowInSeconds() - start_time) * 1000.0;
  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(total_time_ms);

  auto* pnc_map_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
  pnc_map_task->set_name("pnc_map");
  pnc_map_task->set_time_ms(reference_line_provider_ptr_->time_delay_ms());

  if (internal_->planning_status().has_scenario_status()) {
    ptr_trajectory_pb->mutable_scenario_status()->CopyFrom(
        internal_->planning_status().scenario_status());
  }

  if (internal_->planning_status().has_lateral_shift_status()) {
    ptr_trajectory_pb->mutable_lateral_shift_status()->CopyFrom(
        internal_->planning_status().lateral_shift_status());
  }

  if (internal_->planning_status().has_lane_change_status()) {
    ptr_trajectory_pb->mutable_lane_change_status()->CopyFrom(
        internal_->planning_status().lane_change_status());
  }

  ptr_trajectory_pb->mutable_debug()->CopyFrom(internal_->debug_info());
  auto* localize_pose = ptr_trajectory_pb->mutable_localize_pose();
  localize_pose->set_x(last_localize_pose_.x);
  localize_pose->set_y(last_localize_pose_.y);
  localize_pose->set_theta(last_localize_pose_.heading);
}

void Planning::FillHeader(ADCTrajectory* const ptr_trajectory_pb) {
  double now_time = pnc::Time::NowInSeconds();
  auto* header = ptr_trajectory_pb->mutable_header();
  header->set_module_name("planning");
  header->set_timestamp_sec(now_time);
  if (local_view_.prediction_obstacles->has_header()) {
    ptr_trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    ptr_trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    ptr_trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }
  header->set_sequence_num(++sequence_num_);
  header->set_frame_id("vehicle");
}

//TODO: Estop in danerous, highway and overload especially
void Planning::GenerateFallbackTrajectory(ADCTrajectory* ptr_trajectory_pb) {
  FillHeader(ptr_trajectory_pb);
  // internal_->mutable_planning_status()->CopyFrom(internal_->last_planning_status());

  PublishableTrajectory* last_trajectory_ptr = internal_->mutable_publishable_trajectory();
  if (!last_trajectory_ptr || last_trajectory_ptr->empty()) {
    ptr_trajectory_pb->clear_trajectory_point();
    pnc::EStop estop;
    estop.set_is_estop(true);
    estop.set_reason("Localize is unvalid");
    ptr_trajectory_pb->mutable_debug()->CopyFrom(internal_->debug_info());
    ptr_trajectory_pb->set_trajectory_type(ADCTrajectory::TRAJECTORY_FALLBACK);
    internal_->Clear();
    AERROR << "Generate fallback trajectory";
    return;
  }
  
  // this condition occurs when no localize anytime
  if (!last_localize_pose_.is_valid) {
    ptr_trajectory_pb->clear_trajectory_point();
    pnc::EStop estop;
    estop.set_is_estop(true);
    estop.set_reason("Localize is unvalid");
    ptr_trajectory_pb->mutable_debug()->CopyFrom(internal_->debug_info());
    ptr_trajectory_pb->set_trajectory_type(ADCTrajectory::TRAJECTORY_FALLBACK);
    internal_->Clear();
    AERROR << "Generate fallback trajectory";
    return;
  }

  ptr_trajectory_pb->mutable_trajectory_point()->CopyFrom(
      {last_trajectory_ptr->begin(), last_trajectory_ptr->end()});
  double dt = last_trajectory_ptr->header_time() - ptr_trajectory_pb->header().timestamp_sec();
  for (auto& p : *ptr_trajectory_pb->mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  FillPlanningPb(false, ptr_trajectory_pb);
  ptr_trajectory_pb->set_trajectory_type(ADCTrajectory::TRAJECTORY_FALLBACK);

  int fallback_counter = internal_->fallback_counter() + 1;
  internal_->set_fallback_counter(fallback_counter);
  if (fallback_counter >= FLAGS_fallback_counter_threshold) {
    internal_->set_is_report_mrm(true);
  }

  last_trajectory_ptr->set_header_time(ptr_trajectory_pb->header().timestamp_sec());
  last_trajectory_ptr->UpdateRelativeTime(dt);
  AERROR << "Generate fallback trajectory";
}

void Planning::TransformLastPublishedTrajectory(
    const pnc::LocalizePose& last_localize_pose,
    const pnc::LocalizePose& current_localize_pose,
    PublishableTrajectory* const prev_trajectory) {
  if (!last_localize_pose.is_valid || !current_localize_pose.is_valid) {
    return;
  }

  double x_diff_map = current_localize_pose.x - last_localize_pose.x;
  double y_diff_map = current_localize_pose.y - last_localize_pose.y;

  double cos_map_veh = std::cos(last_localize_pose.heading);
  double sin_map_veh = std::sin(last_localize_pose.heading);

  double x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
  double y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

  double heading_diff =
      current_localize_pose.heading - last_localize_pose.heading;

  TrajectoryStitcher::TransformLastPublishedTrajectory(
      x_diff_veh, y_diff_veh, heading_diff, prev_trajectory);
}

}  // namespace planning
}  // namespace xju