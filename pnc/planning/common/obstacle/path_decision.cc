/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/obstacle/path_decision.h"

#include "common/logger/logger.h"

namespace xju {
namespace planning {

Obstacle *PathDecision::AddObstacle(const Obstacle &obstacle) {
  return obstacles_.Add(obstacle.id(), obstacle);
}

const IndexedObstacles &PathDecision::obstacles() const { return obstacles_; }

Obstacle *PathDecision::Find(const std::string &object_id) {
  return obstacles_.Find(object_id);
}

const Obstacle *PathDecision::Find(const std::string &object_id) const {
  return obstacles_.Find(object_id);
}

bool PathDecision::SetLongitudinalDecision(
    const std::string& object_id, const ObjectDecisionType& decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "fialed to find obstacle.";
    return false;
  }
  return obstacle->set_longitudinal_decision(decision);
}

bool PathDecision::SetLateralDecision(
    const std::string& object_id, const ObjectDecisionType& decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "fialed to find obstacle.";
    return false;
  }
  return obstacle->set_lateral_decision(decision);
}

void PathDecision::SetSTBoundary(const std::string &id,
                                 const STBoundary &boundary) {
  auto *obstacle = obstacles_.Find(id);

  if (!obstacle) {
    AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->set_st_boundary(boundary);
  }
}

void PathDecision::EraseStBoundaries() {
  for (const auto *obstacle : obstacles_.Items()) {
    auto *obstacle_ptr = obstacles_.Find(obstacle->id());
    obstacle_ptr->EraseStBoundary();
  }
}

} // namespace planning
} // namespace xju