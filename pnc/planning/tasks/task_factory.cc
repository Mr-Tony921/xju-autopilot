/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/task_factory.h"

#include "planning/tasks/deciders/lateral_shift_decider/lateral_shift_decider.h"
#include "planning/tasks/deciders/path_bound_decider/path_bound_decider.h"
#include "planning/tasks/optimizers/path_optimizer/path_optimizer.h"
#include "planning/tasks/deciders/path_decider/path_decider.h"
#include "planning/tasks/deciders/st_graph_decider/st_graph_decider.h"
#include "planning/tasks/deciders/speed_decider/speed_decider.h"
#include "planning/tasks/optimizers/speed_optimizer/speed_optimizer.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

std::unordered_map<pnc::TaskType, pnc::TaskConfig, std::hash<int>> TaskFactory::default_task_coinfig_map_;
std::unordered_map<pnc::TaskType, std::shared_ptr<Task>> TaskFactory::task_map_;

bool TaskFactory::Init(
    const PlanningConfig& config,
    const std::shared_ptr<PlanningInternal>& internal) {
  for (const auto& default_task_config : config.default_task_config()) {
    default_task_coinfig_map_[default_task_config.task_type()] = default_task_config;
  }
  RegisterTasks(internal);
  return true;
}

std::shared_ptr<Task> TaskFactory::CreateTask(const pnc::TaskConfig& config) {
  auto task_iter = task_map_.find(config.task_type());
  if (task_iter == task_map_.end()) {
    return nullptr;
  }

  auto default_task_config_iter = default_task_coinfig_map_.find(config.task_type());
  if (default_task_config_iter == default_task_coinfig_map_.end()) {
    return nullptr;
  }

  pnc::TaskConfig task_config = default_task_config_iter->second;
  task_config.MergeFrom(config);
  auto task_ptr = task_iter->second;
  task_ptr->Reset();
  task_ptr->Init(task_config);
  return task_ptr;
}

std::shared_ptr<Task> TaskFactory::CreateTask(const pnc::TaskType& task_type) {
  auto task_iter = task_map_.find(task_type);
  if (task_iter == task_map_.end()) {
    return nullptr;
  }
  auto task_ptr = task_iter->second;
  task_ptr->Reset();
  return task_ptr;
}

void TaskFactory::RegisterTasks(const std::shared_ptr<PlanningInternal>& internal) {
  task_map_[pnc::TaskType::LATERAL_SHIFT_DECIDER] = std::make_shared<LateralShiftDecider>(
      default_task_coinfig_map_[pnc::TaskType::LATERAL_SHIFT_DECIDER], internal);
  task_map_[pnc::TaskType::PATH_BOUND_DECIDER] = std::make_shared<PathBoundDecider>(
      default_task_coinfig_map_[pnc::TaskType::PATH_BOUND_DECIDER], internal);
  task_map_[pnc::TaskType::PATH_OPTIMIZER] = std::make_shared<PathOptimizer>(
      default_task_coinfig_map_[pnc::TaskType::PATH_OPTIMIZER], internal);
  task_map_[pnc::TaskType::PATH_DECIDER] = std::make_shared<PathDecider>(
      default_task_coinfig_map_[pnc::TaskType::PATH_DECIDER], internal);
  task_map_[pnc::TaskType::ST_GRAPH_DECIDER] = std::make_shared<StGraphDecider>(
      default_task_coinfig_map_[pnc::TaskType::ST_GRAPH_DECIDER], internal);
  task_map_[pnc::TaskType::SPEED_DECIDER] = std::make_shared<SpeedDecider>(
      default_task_coinfig_map_[pnc::TaskType::SPEED_DECIDER], internal);
  task_map_[pnc::TaskType::SPEED_OPTIMIZER] = std::make_shared<SpeedOptimizer>(
      default_task_coinfig_map_[pnc::TaskType::SPEED_OPTIMIZER], internal);

  for (auto task : task_map_) {
    AINFO << "RegisterTask: " << pnc::TaskType_Name(task.first);
  }
}

} // namespace planning
} // namespace xju