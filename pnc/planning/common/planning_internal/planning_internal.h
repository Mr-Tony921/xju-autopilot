/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning_status.pb.h"
#include "debug.pb.h"
#include "planning/common/trajectory/publishable_trajectory.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

class PlanningInternal {
 public:
  PlanningInternal() = default;
  ~PlanningInternal() = default;

  void Init() {
    Clear();
  }

  void Clear() {
    clear_planning_status();
    clear_publishable_trajectory();
    is_report_mrm_ = false;
    fallback_counter_ = 0;
  }

  const PlanningStatus& planning_status() const {
    return planning_status_;
  }

  const PlanningStatus& last_planning_status() const {
    return last_planning_status_;
  }

  const PublishableTrajectory& publishable_trajectory() const {
    return publishable_trajectory_;
  }

  const pnc::Debug debug_info() const {
    return debug_info_;
  }

  PlanningStatus* mutable_planning_status() {
    return &planning_status_;
  }

  PlanningStatus* mutable_last_planning_status() {
    return &last_planning_status_;
  }

  PublishableTrajectory* mutable_publishable_trajectory() {
    return &publishable_trajectory_;
  }

  pnc::Debug* mutable_debug_info() {
    return &debug_info_;
  }

  void clear_planning_status() {
    planning_status_.Clear();
    last_planning_status_.Clear();
  }

  void clear_publishable_trajectory() {
    publishable_trajectory_.Clear();
  }
  
  void clear_debug_info() {
    debug_info_.Clear();
  }

  void set_publishable_trajectory(const PublishableTrajectory& trajectory) {
    publishable_trajectory_ = trajectory;
  }

  const bool is_report_mrm() const {
    return is_report_mrm_;
  }

  void set_is_report_mrm(const bool is_report_mrm) {
    is_report_mrm_ = is_report_mrm;
  }

  const int fallback_counter() const {
    return fallback_counter_;
  }

  void set_fallback_counter(const int fallback_counter) {
    fallback_counter_ = fallback_counter;
  }


 private:
  PlanningStatus planning_status_;
  PlanningStatus last_planning_status_;
  PublishableTrajectory publishable_trajectory_;
  pnc::Debug debug_info_;

  bool is_report_mrm_ = false;
  int fallback_counter_ = 0;
};

} // namespace planning
} // namespace xju
