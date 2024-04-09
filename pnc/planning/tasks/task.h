/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "planning/common/frame/frame.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "task_config.pb.h"

namespace xju {
namespace planning {

class Task {
 public:
  Task(const pnc::TaskConfig& config, 
       const std::shared_ptr<PlanningInternal>& internal);
  virtual ~Task() = default;
  virtual void Init(const pnc::TaskConfig& config) = 0;
  virtual void Reset() = 0;

  virtual bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* const frame);
  virtual bool Process(Frame* frame);
  const std::string& name() const {
    return name_;
  }

 protected:
  std::string name_;
  pnc::TaskConfig config_;
  std::shared_ptr<PlanningInternal> internal_;
  Frame* frame_;
  std::shared_ptr<ReferenceLineInfo> reference_line_info_ = nullptr;
};

} // namespace planning
} // namespace xju
