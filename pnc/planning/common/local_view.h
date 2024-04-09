/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <shared_mutex>

#include "chassis.pb.h"
#include "em_lane.pb.h"
#include "localization.pb.h"
#include "prediction_obstacle.pb.h"
#include "routing.pb.h"
namespace xju {
namespace planning {

struct LocalView {
  static inline std::shared_mutex prediction_obstacles_mutex = {};
  static inline std::shared_mutex chassis_info_mutex = {};
  static inline std::shared_mutex localization_mutex = {};
  static inline std::shared_mutex em_lanes_mutex = {};
  static inline std::shared_mutex routing_mutex = {};

  std::shared_ptr<pnc::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<pnc::ChassisInfo> chassis_info;
  std::shared_ptr<pnc::Localization> localization;
  std::shared_ptr<pnc::EmLanes> em_lanes;
  std::shared_ptr<pnc::Routing> routing;

  LocalView& operator=(const LocalView& local_view_) {
    if (local_view_.prediction_obstacles) {
      std::shared_lock<std::shared_mutex> lock(prediction_obstacles_mutex);
      if (this->prediction_obstacles) {
        *(this->prediction_obstacles) = *(local_view_.prediction_obstacles);
      } else {
        this->prediction_obstacles = std::make_shared<pnc::PredictionObstacles>(
            *(local_view_.prediction_obstacles));
      }
    }
    if (local_view_.chassis_info) {
      std::shared_lock<std::shared_mutex> lock(chassis_info_mutex);
      if (this->chassis_info) {
        *(this->chassis_info) = *(local_view_.chassis_info);
      } else {
        this->chassis_info =
            std::make_shared<pnc::ChassisInfo>(*(local_view_.chassis_info));
      }
    }
    if (local_view_.localization) {
      std::shared_lock<std::shared_mutex> lock(localization_mutex);
      if (this->localization) {
        *(this->localization) = *(local_view_.localization);
      } else {
        this->localization =
            std::make_shared<pnc::Localization>(*(local_view_.localization));
      }
    }
    if (local_view_.em_lanes) {
      std::shared_lock<std::shared_mutex> lock(em_lanes_mutex);
      if (this->em_lanes) {
        *(this->em_lanes) = *(local_view_.em_lanes);
      } else {
        this->em_lanes =
            std::make_shared<pnc::EmLanes>(*(local_view_.em_lanes));
      }
    }
    if (local_view_.routing) {
      std::shared_lock<std::shared_mutex> lock(routing_mutex);
      if (this->routing) {
        *(this->routing) = *(local_view_.routing);
      } else {
        this->routing = std::make_shared<pnc::Routing>(*(local_view_.routing));
      }
    }
    return *this;
  }
};

}  // namespace planning
}  // namespace xju
