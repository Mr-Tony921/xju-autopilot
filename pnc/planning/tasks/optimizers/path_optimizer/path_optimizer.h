/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/tasks/task.h"
#include "planning/tasks/optimizers/path_optimizer/vehicle_model_optimizer.h"
#include "planning/tasks/optimizers/path_optimizer/mass_point_model_optimizer.h"
#include "planning/tasks/optimizers/path_optimizer/vehicle_model_optimizer_osqp.h"

namespace xju {
namespace planning {

class PathOptimizer : public Task {
 public:
  PathOptimizer(const pnc::TaskConfig& config, 
                const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info, 
               Frame* const frame) override;

 private:
  bool OptimizeMultiThread();
  bool OptimizeSequence();
  void DrawSLDebugInfo();
  void DrawCartesianDebugInfo();
  void LogPlannedPath();
  void CreateLogFile(std::fstream& log_file, std::string name);
  void LogInfoToFile();

 private:
  VehicleModelOptimizer vehicle_model_optimizer_;
  // MassPointModelOptimizer vehicle_model_optimizer_;
  std::vector<VehicleModelOptimizer> 
      vehicle_model_optimizers_{VehicleModelOptimizer(), VehicleModelOptimizer()};
};

} // namespace planning
} // namespace xju
