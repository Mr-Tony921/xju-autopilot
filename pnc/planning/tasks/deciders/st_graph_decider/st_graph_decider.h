/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "pnc_point.pb.h"
#include "planning/tasks/task.h"
#include "planning/common/path/path_data.h"
#include "planning/common/speed/speed_limit.h"
#include "planning_task_config.pb.h"

namespace xju {
namespace planning {

class StGraphDecider : public Task {
 public:
  StGraphDecider(const pnc::TaskConfig& config, 
                 const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* const frame) override;
  
//  private:
//   double GetMinSonSTGraph(
//       const std::vector<const STBoundary*>& st_boundaries);

 private:
 void DrawDebugInfo();

 private:
  StGraphDeciderConfig st_graph_decider_config_;
  PathData* path_data_;
  
};
} // namespace planning
} // namespace xju
