/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/scenario.h"

#include "common/file/file.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

Scenario::Scenario(const pnc::ScenarioConfig& config,
                   std::shared_ptr<PlanningInternal>& internal) 
    : config_(config), internal_(internal) {
  name_ = pnc::ScenarioType_Name(config.scenario_type());
}

bool Scenario::Init() {
  for (const auto& satge_config : config_.stage_config()) {
    stage_config_map_[satge_config.stage_type()] = satge_config;
  }
  current_stage_ = CreateStage(stage_config_map_[config_.stage_type(0)], internal_);
  return true;
}

bool Scenario::LoadConfig(const std::string& config_file, pnc::ScenarioConfig* config) {
  pnc::File::GetProtoConfig<pnc::ScenarioConfig>(config_file, config);
  return true;
}

Scenario::Status Scenario::Process(Frame* frame) {
  internal_->mutable_planning_status()->mutable_scenario_status()
      ->set_scenario_type(config_.scenario_type());
  if (current_stage_ == nullptr) {
    scenario_status_ = Status::UNKNOWN;
    return scenario_status_;
  }
  
  if (current_stage_->stage_type() == pnc::StageType::NO_STAGE) {
    scenario_status_ = Status::FINISHED;
    return scenario_status_;
  }
  ADEBUG << "Current stage: " << pnc::StageType_Name(current_stage_->stage_type());
  Stage::Status stage_status = current_stage_->Process(frame);

  while (stage_status == Stage::Status::NEXT) {
    pnc::StageType next_stage_type = current_stage_->next_stage_type();
    if (stage_config_map_.find(next_stage_type) == stage_config_map_.end()) {
      scenario_status_ = Status::UNKNOWN;
      return scenario_status_;
    }
    current_stage_ = CreateStage(stage_config_map_[next_stage_type], internal_);
    if (current_stage_ == nullptr) {
      scenario_status_ = Status::UNKNOWN;
      return scenario_status_;
    }
    stage_status = current_stage_->Process(frame);
  }

  switch (stage_status) {
    case Stage::Status::ERROR: {
      scenario_status_ = Status::UNKNOWN;
      break;
    }
    case Stage::Status::PROCESSING: {
      scenario_status_ = Status::PROCESSING;
      break;
    }
    case Stage::Status::FINISHED: {
      pnc::StageType next_stage_type = current_stage_->next_stage_type();
      if (next_stage_type != current_stage_->stage_type()) {
        if (next_stage_type == pnc::StageType::NO_STAGE) {
          scenario_status_ = Status::FINISHED;
          return scenario_status_;
        }

        if (stage_config_map_.find(next_stage_type) == stage_config_map_.end()) {
          scenario_status_ = Status::UNKNOWN;
          return scenario_status_;
        }
        current_stage_ = CreateStage(stage_config_map_[next_stage_type], internal_);
        if (current_stage_ == nullptr) {
          scenario_status_ = Status::UNKNOWN;
          return scenario_status_;
        }
        if (current_stage_->stage_type() != pnc::StageType::NO_STAGE) {
          scenario_status_ = Status::PROCESSING;
        } else {
          scenario_status_ = Status::FINISHED;
        }
        break;
      }
    }
    default: {
      scenario_status_ = Status::UNKNOWN;
    }
  }
  return scenario_status_;
}

} // namespace planning
} // namespace xju