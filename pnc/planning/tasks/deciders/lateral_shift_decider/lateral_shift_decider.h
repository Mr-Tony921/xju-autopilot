/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/common/obstacle/obstacle.h"
#include "planning/tasks/task.h"
namespace xju {
namespace planning {
class LateralShiftDecider : public Task {
 public:
  LateralShiftDecider(const pnc::TaskConfig& config,
                      const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
               Frame* const frame) override;

 private:
  bool UpdateADCState(const ReferenceLineInfo& reference_line_info,
                      const Frame& frame);
  bool IsOutOfLane(const ReferenceLineInfo& reference_line_info) const;
  void GetDistanceBetweenObstacleToBoundary(
      const ReferenceLineInfo& reference_line_info,
      double* left_distance_to_boundary, double* right_distance_to_boundary);
  bool NeedPayAttention(const Obstacle& obstacle,
                        const ReferenceLineInfo& reference_line_info) const;
  double CalculateShiftDistances(const double& distance_to_bound) const;
  void SelectShiftDirection(const double& lateral_shift_distance_left,
                            const double& lateral_shift_distance_right,
                            LateralShiftStatus::Type* const candidate_type,
                            double* const candidate_shift_distance);
  void UpdateLateralShiftStatus(const std::string& id,
                                const LateralShiftStatus::Type& type,
                                const double& shift_distance);
  void ShiftArbitration(const LateralShiftStatus::Type& candidate_type,
                        const double& candidate_shift_distance);

 private:
  double adc_frenet_s_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_speed_ = 0.0;
  int safe_count_ = 0;
  int warnning_count_ = 0;
  int hold_on_direction_count_ = 0;
  int hold_on_distance_count_ = 0;
  std::string target_id_ = "";
  std::string left_target_id_ = "";
  std::string right_target_id_ = "";
};

}  // namespace planning
}  // namespace xju
