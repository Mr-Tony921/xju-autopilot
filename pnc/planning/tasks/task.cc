/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/task.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

Task::Task(const pnc::TaskConfig& config, 
           const std::shared_ptr<PlanningInternal>& internal)
    : config_(config), internal_(internal) {
  name_ = pnc::TaskType_Name(config.task_type());
}

bool Task::Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* const frame) {
  ADEBUG << name_ << " Process";
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return true;
}

bool Task::Process(Frame* frame) {
  frame_ = frame;
  return true;
}

} // namespace planning
} // namespace xju
