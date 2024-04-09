
/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/stage.h"

#include <unordered_map>
#include <limits>

#include "common/logger/logger.h"
#include "common/time/time.h"
#include "common/math/vec2d.h"
#include "common/util/point_factory.h"
#include "planning/common/constraint_checker/constraint_checker.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/tasks/task_factory.h"
#include "stage.h"

namespace xju {
namespace planning {

Stage::Stage(const pnc::StageConfig& config,
             const std::shared_ptr<PlanningInternal>& internal) 
             : config_(config), internal_(internal) {
  name_ = pnc::StageType_Name(config.stage_type());
  next_stage_type_ = config_.stage_type();
  std::unordered_map<pnc::TaskType, const pnc::TaskConfig*, std::hash<int>> task_config_map;
  for (const auto& task_config : config_.task_config()) {
    task_config_map[task_config.task_type()] = &task_config;
  }

  for (int i = 0; i < config_.task_type_size(); ++i) {
    auto task_type = config_.task_type(i);
    auto iter = task_map_.find(task_type);
    if (iter == task_map_.end()) {
      std::shared_ptr<Task> task_ptr;
      if (task_config_map.find(task_type) != task_config_map.end()) {
        task_ptr = TaskFactory::CreateTask(*task_config_map[task_type]);
      } else {
        task_ptr = TaskFactory::CreateTask(task_type);
      }
      task_list_.push_back(task_ptr);
      task_map_[task_type] = std::move(task_ptr);
    } else {
      task_list_.push_back(iter->second);
    }
  }
}

bool Stage::ProcessTaskOnReferenceLine(std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* frame) {
  // if (!reference_line_info->drivable()) {
  //   return false;
  // }
  for (const auto& task_ptr : task_list_) {
    double start = pnc::Time::NowInSeconds();
    bool status = task_ptr->Process(reference_line_info, frame);
    double cost_time = (pnc::Time::NowInSeconds() - start) * 1000.0;
    ADEBUG << "Cost Time of Task " << task_ptr->name() << " is: " << cost_time << "ms.";
    RecordDebugInfo(reference_line_info, task_ptr->name(), cost_time);
    if (status == false) {
      AERROR << "Task " << task_ptr->name() << " Process Failed!";
      reference_line_info->set_drivable(false);
      reference_line_info->set_cost(std::numeric_limits<double>::infinity());
      return false;
    }
  }

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(&trajectory)) {
    AERROR << "Fail to aggregate planning trajectory.";
    reference_line_info->set_drivable(false);
    reference_line_info->set_cost(std::numeric_limits<double>::infinity());
    return false;
  }
  // if (!ConstraintChecker::ValidTrajectroy(trajectory)) {
  //   AERROR << "Current planning trajectory is not valid.";
  //   reference_line_info->set_drivable(false);
  //   reference_line_info->set_cost(std::numeric_limits<double>::infinity());
  //   return false;
  // }
  reference_line_info->set_trajectory(trajectory);
  reference_line_info->set_drivable(true);
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  return true;
}

bool Stage::ProcessTaskOnOpenSpace(Frame* frame) {

  for (const auto& task_ptr : task_list_) {
    bool status = task_ptr->Process(frame);
    if (status == false) {
      return false;
    }
  }
  return true;
}

void Stage::RecordDebugInfo(std::shared_ptr<ReferenceLineInfo> reference_line_info,
                            const std::string& name,
                            const double time_diff_ms) {

  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

Stage::Status Stage::FinishScenario() {
  next_stage_type_ = pnc::StageType::NO_STAGE;
  return Stage::Status::FINISHED;
}

} // namespace planning
} // namespace xju
