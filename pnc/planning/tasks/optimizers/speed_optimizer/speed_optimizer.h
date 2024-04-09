/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/tasks/task.h"

namespace xju {
namespace planning {

class SpeedOptimizer : public Task {
 public:
  SpeedOptimizer(const pnc::TaskConfig& config,
                 const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
               Frame* const frame) override;
  std::string DebugString() const;
  void DrawDebugInfo();

 private:
  double MinVelocity(double t, double init_v, double init_a);
  void CreateLogFile(std::fstream &log_file,std::string name);

 private:
  std::vector<std::pair<double, double>> ds_bounds_;
  std::vector<std::pair<double, double>> s_bounds_;
};

}  // namespace planning
}  // namespace xju
