/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <deque>

#include "planning/common/frame/frame.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning/common/speed/speed_limit.h"

namespace xju {
namespace planning {
class SpeedLimitDecider {
 public:
  SpeedLimitDecider(
      const StGraphDeciderConfig& st_graph_config,
      std::shared_ptr<ReferenceLineInfo> const reference_line_info, 
      Frame* const frame);

  bool GetSpeedLimit(SpeedLimit* const speed_limit);

 private:
  SpeedLimitDecider() = default;
  SpeedLimitDecider(const SpeedLimitDecider& speed_limit_decider);
  double MaxKappaInSlidingWindow(
      const std::vector<pnc::PathPoint>& planned_path, const int i);

 private:
  const StGraphDeciderConfig& st_graph_config_;
  std::shared_ptr<ReferenceLineInfo> reference_line_info_;
  Frame* frame_;
  std::deque<int> index_q_;
  int sliding_window_size_;
};

} // namespace planning    
} // namespace xju
