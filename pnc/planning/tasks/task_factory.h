/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <unordered_map>

#include "planning/tasks/task.h"
#include "planning_config.pb.h"
#include "task_config.pb.h"

namespace xju {
namespace planning {

class TaskFactory {
 public:
  static bool Init(const PlanningConfig& config, const std::shared_ptr<PlanningInternal>& internal);

  static std::shared_ptr<Task> CreateTask(
      const pnc::TaskConfig& config);

  static std::shared_ptr<Task> CreateTask(
      const pnc::TaskType& tske_type);

 private:
  static void RegisterTasks(const std::shared_ptr<PlanningInternal>& internal);

 private:
  static std::unordered_map<pnc::TaskType, pnc::TaskConfig, std::hash<int>> default_task_coinfig_map_;
  static std::unordered_map<pnc::TaskType, std::shared_ptr<Task>> task_map_;
};

} // namespace planning
} // namespace xju
