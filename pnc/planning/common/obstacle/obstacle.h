/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <memory>
#include <list>

#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "planning/common/speed/st_boundary.h"
#include "planning/common/indexed_list.h"
#include "pnc_point.pb.h"
#include "perception_obstacle.pb.h"
#include "prediction_obstacle.pb.h"
#include "object_decision.pb.h"
#include "localization.pb.h"
#include "sl_boundary.pb.h"

namespace xju {
namespace planning {

class Obstacle {
 public:
  Obstacle() = default;
  ~Obstacle() = default;

  Obstacle(const std::string& id,
           const pnc::PerceptionObstacle& perception_obstacle,
           const bool is_static);
  
  Obstacle(const std::string& id,
           const pnc::PerceptionObstacle& perception_obstacle,
           const pnc::Trajectory& tajectory,
           const bool is_static);

  const std::string& id() const { return id_; }
  void set_id(const std::string id) { id_ = id; }
  double speed() const { return speed_; }
  double acceleration() const { return acceleration_; }
  bool is_static() const { return is_static_; }
  bool is_virtual() const { return is_virtual_; }
  bool is_caution() const { return is_caution_; }

  pnc::Box2d bounding_box() const { return bounding_box_; }
  pnc::Box2d GetBoundingBoxAtTime(const double t);
  pnc::Box2d GetBoundingBox(const pnc::TrajectoryPoint& point) const;
  pnc::Box2d GetBoundingBox(const pnc::PathPoint& point) const;

  pnc::TrajectoryPoint GetPointAtTime(const double relative_time) const;

  pnc::Polygon2d polygon() const { return polygon_; }

  pnc::Trajectory trajectory() const { return trajectory_; }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }
  const pnc::PerceptionObstacle& perception_obstacle() const {
    return perception_obstacle_;
  }

  int32_t perception_id() const { return perception_id_; }

  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const pnc::PredictionObstacles& predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const pnc::Box2d& obstacle_box);

  static bool IsValidPerceptionObstacle(
      const pnc::PerceptionObstacle& perception_obstacle);
  

  static bool IsValidTrajectoryPoint(const pnc::TrajectoryPoint& point);

  bool set_longitudinal_decision(const ObjectDecisionType& decision);

  bool set_lateral_decision(const ObjectDecisionType& decision);
  
  const ObjectDecisionType& longitudinal_decision() const {
    return longitudinal_decision_;
  }
  const ObjectDecisionType& lateral_decision() const {
    return lateral_decision_;
  }

  void set_sl_boundary(const SLBoundary& sl_boundary) { 
    sl_boundary_ = sl_boundary;
  }
  const SLBoundary& sl_boundary() const { return sl_boundary_; }
  void set_st_boundary(const STBoundary& st_boundary) { 
    st_boundary_ = st_boundary;
  }
  const STBoundary& st_boundary() const { return st_boundary_; }

  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;
  bool IsLongitudinalDecision(const ObjectDecisionType& decision);
  bool IsLateralDecision(const ObjectDecisionType& decision);
  bool HasLongitudinalDecision() const;
  bool HasLateralDecision() const;

  std::string DebugString() const;

  STBoundary CalculateSTBoundary(const std::vector<pnc::PathPoint>& path);
  void EraseStBoundary() { st_boundary_ = STBoundary(); }

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  bool is_caution_ = false;
  double speed_ = 0.0;
  double acceleration_ = 0.0;

  pnc::PerceptionObstacle perception_obstacle_;
  pnc::Box2d bounding_box_;
  pnc::Polygon2d polygon_;

  pnc::Trajectory trajectory_;

  SLBoundary sl_boundary_;
  STBoundary st_boundary_;

  ObjectDecisionType longitudinal_decision_;
  ObjectDecisionType lateral_decision_;
  
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

} // namespace planning
} // namespace xju
