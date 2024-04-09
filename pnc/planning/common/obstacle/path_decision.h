/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

#include "planning/common/indexed_list.h"
#include "planning/common/obstacle/obstacle.h"

namespace xju {
namespace planning {

class PathDecision {
 public:
  PathDecision() = default;

  Obstacle *AddObstacle(const Obstacle &obstacle);

  const IndexedList<std::string, Obstacle> &obstacles() const;

  const Obstacle *Find(const std::string &object_id) const;

  Obstacle *Find(const std::string &object_id);

  bool SetLongitudinalDecision(const std::string& object_id,
                               const ObjectDecisionType& decision);
  bool SetLateralDecision(const std::string& object_id,
                          const ObjectDecisionType& decision);

  void SetSTBoundary(const std::string &id, const STBoundary &boundary);
  void EraseStBoundaries();

 private:
  IndexedObstacles obstacles_;
};

} // namespace planning
} // namespace xju
