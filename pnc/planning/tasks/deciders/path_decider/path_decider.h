/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/vehicle_config/vehicle_config_provider.h"
#include "planning/common/obstacle/obstacle.h"
#include "planning/tasks/task.h"

namespace xju {
namespace planning {
class PathDecider : public Task {
 public:
  PathDecider(const pnc::TaskConfig& config,
              const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
               Frame* const frame) override;

 private:
  bool MakeObjectDecision(const PathData& path_data,
                          const std::string& blocking_obstacle_id,
                          PathDecision* const path_decision);

  bool MakeStaticObstacleDecision(const PathData& path_data,
                                  const std::string& blocking_obstacle_id,
                                  PathDecision* const path_decision);

  ObjectStop GenerateObjectStopDecision(const Obstacle& obstacle) const;

  pnc::FrenetFramePoint GetNearestPoint(
      const std::vector<pnc::FrenetFramePoint>& frenet_path,
      const SLBoundary& sl) const;

  int AddPathEndStop(
      Frame* const frame,
      std::shared_ptr<ReferenceLineInfo> const reference_line_info);

  int AddDestinationStop(
      Frame* const frame,
      std::shared_ptr<ReferenceLineInfo> const reference_line_info);

  int AddReferencelineEndStop(
      Frame* const frame,
      std::shared_ptr<ReferenceLineInfo> const reference_line_info);

  int BuildStopDecision(
      const std::string& stop_wall_id, const double stop_line_s,
      const double stop_distance, Frame* const frame,
      std::shared_ptr<ReferenceLineInfo> const reference_line_info);

 private:
  pnc::VehicleConfig vehicle_config_;
  static constexpr char const* PATH_END_VO_ID_PREFIX = "PATH_END_";
  static constexpr char const* DESTINATION_VO_ID_PREFIX = "DESTINATION_";
  static constexpr char const* REFERENCELINE_END_VO_ID_PREFIX =
      "REFERENCELINE_END_";
};

}  // namespace planning
}  // namespace xju
