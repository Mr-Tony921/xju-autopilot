/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <memory>
#include <vector>

#include "planning/common/local_view.h"
#include "planning/scenarios/scenario_manager.h"
#include "planning/common/frame/frame.h"
#include "pnc_map/reference_line_provider.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "planning_config.pb.h"
#include "common/data_manager/data_manager.h"

namespace xju {
namespace planning {

class Planning {
 public:
  Planning() = default;
  ~Planning();

 public:
  bool Init(const std::string& proto_config_file_path);

  bool Process(const LocalView& local_view, 
               ADCTrajectory* const ptr_trajectory_pb);

 private:
  void Init();
  bool CheckInput();
  bool Plan(
      const double timestamp,
      const std::vector<pnc::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const ptr_trajectory_pb);

  void FillPlanningPb(const bool replan,
                      ADCTrajectory* const ptr_trajectory_pb);

  void FillHeader(ADCTrajectory* const ptr_trajectory_pb);

  void GenerateFallbackTrajectory(ADCTrajectory* ptr_trajectory_pb);

  void TransformLastPublishedTrajectory(
      const pnc::LocalizePose& last_localize_pose,
      const pnc::LocalizePose& current_localize_pose,
      PublishableTrajectory* const prev_trajectory);

 private:
  uint64_t sequence_num_ = 0;
  PlanningConfig config_;
  LocalView local_view_;
  ScenarioManager scenario_manager_;
  std::unique_ptr<Frame> frame_;
  std::shared_ptr<PlanningInternal> internal_;
  std::unique_ptr<pnc_map::ReferenceLineProvider> reference_line_provider_ptr_;
  pnc::LocalizePose last_localize_pose_;
};

} // namespace planning
} // namespace xju
