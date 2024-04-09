/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/obstacle/obstacle.h"

#include "common/logger/logger.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/time/time.h"
#include "common/util/coordinate.h"
#include "common/vehicle_state/vehicle_state_provider.h"
namespace xju {
namespace planning {

namespace {
const double kStBoundaryDeltaS = 0.2;        // meters
const double kStBoundarySparseDeltaS = 1.0;  // meters
const double kStBoundaryDeltaT = 0.05;       // seconds
}  // namespace


Obstacle::Obstacle(const std::string& id,
                   const pnc::PerceptionObstacle& perception_obstacle,
                   const bool is_static)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle),
      bounding_box_(perception_obstacle_.position().x(),
                    perception_obstacle_.position().y(),
                    perception_obstacle_.heading(),
                    perception_obstacle_.length(),
                    perception_obstacle_.width()) {
  std::vector<pnc::Vec2d> polygon_points;
  for (const auto& point : perception_obstacle.polygon_point()) {
    polygon_points.emplace_back(point.x(), point.y());
  }
  polygon_ = pnc::Polygon2d(polygon_points);
  is_static_ = is_static;
  is_virtual_ = (perception_obstacle.id() < 0);
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());
  acceleration_ = std::hypot(perception_obstacle.acceleration().x(),
                      perception_obstacle.acceleration().y());
}

Obstacle::Obstacle(const std::string& id,
                   const pnc::PerceptionObstacle& perception_obstacle,
                   const pnc::Trajectory& trajectory,
                   const bool is_static)
    : Obstacle(id, perception_obstacle, is_static) {
  trajectory_ = trajectory;
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }
    cumulative_s +=
        pnc::Distance(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}

pnc::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    pnc::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_da(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const pnc::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return pnc::Interpolate(*(it_lower - 1), *it_lower, relative_time);
  }
}

pnc::Box2d Obstacle::GetBoundingBoxAtTime(const double t) {
  pnc::TrajectoryPoint point = GetPointAtTime(t);
  return GetBoundingBox(point);
}

pnc::Box2d Obstacle::GetBoundingBox(
    const pnc::TrajectoryPoint& point) const {
  return pnc::Box2d({point.path_point().x(), point.path_point().y()},
                    point.path_point().theta(),
                    perception_obstacle_.length(),
                    perception_obstacle_.width());
}

pnc::Box2d Obstacle::GetBoundingBox(
    const pnc::PathPoint& point) const {
  return pnc::Box2d({point.x(), point.y()},
                    point.theta(),
                    perception_obstacle_.length(),
                    perception_obstacle_.width());
}

std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const pnc::PredictionObstacles& predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;
  pnc::LocalizePose localize_pose = pnc::VehicleStateProvider::localize_pose();

  ADEBUG << "prediction obstacle size: " << predictions.prediction_obstacle_size();

  pnc::LocalizePose from;
  pnc::LocalizePose to;
  from.x = predictions.localization_pose().x();
  from.y = predictions.localization_pose().y();
  from.heading = predictions.localization_pose().theta();
  to.x = localize_pose.x;
  to.y = localize_pose.y;
  to.heading = localize_pose.heading;
  
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    pnc::PredictionObstacle temp_prediction_obstacle;
    temp_prediction_obstacle.CopyFrom(prediction_obstacle);
    if (!IsValidPerceptionObstacle(temp_prediction_obstacle.perception_obstacle())) {
      AERROR << "Invalid perception obstacle: "
             << temp_prediction_obstacle.perception_obstacle().DebugString();
      continue;
    }
    const auto perception_id =
        std::to_string(temp_prediction_obstacle.perception_obstacle().id());

    pnc::PathPoint point;
    point.set_x(temp_prediction_obstacle.perception_obstacle().position().x());
    point.set_y(temp_prediction_obstacle.perception_obstacle().position().y());
    point.set_theta(temp_prediction_obstacle.perception_obstacle().heading());
    ADEBUG << " trans before " << point.x() << " " << point.y() << " " << point.theta();
    pnc::PathPointPropagate(from, to, &point);
    ADEBUG << " trans after " << point.x() << " " << point.y() << " " << point.theta();

    temp_prediction_obstacle.mutable_perception_obstacle()->mutable_position()->set_x(point.x()),
    temp_prediction_obstacle.mutable_perception_obstacle()->mutable_position()->set_y(point.y()),
    temp_prediction_obstacle.mutable_perception_obstacle()->set_heading(point.theta());
    
    if (temp_prediction_obstacle.prediction_trajectory().empty()) {
      obstacles.emplace_back(
          new Obstacle(perception_id, temp_prediction_obstacle.perception_obstacle(),
                       temp_prediction_obstacle.is_static()));
      continue;
    }

    int trajectory_index = 0;
    for (auto prediction_trajectory =
             temp_prediction_obstacle.mutable_prediction_trajectory()->begin();
         prediction_trajectory !=
         temp_prediction_obstacle.mutable_prediction_trajectory()->end();
         ++prediction_trajectory) {
      bool is_valid_trajectory = true;
      for (auto point = prediction_trajectory->mutable_trajectory()
                             ->mutable_trajectory_point()
                             ->begin();
           point != prediction_trajectory->mutable_trajectory()
                        ->mutable_trajectory_point()
                        ->end(); ++point) {
        if (!IsValidTrajectoryPoint(*point)) {
          AERROR << "obj:" << perception_id << " TrajectoryPoint: "
                 << prediction_trajectory->trajectory().ShortDebugString()
                 << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }

        pnc::PathPointPropagate(from, to, point->mutable_path_point());
      }
      if (!is_valid_trajectory) {
        continue;
      }
      const std::string obstacle_id =
          perception_id + "_" + std::to_string(trajectory_index);

      obstacles.emplace_back(new Obstacle(
          obstacle_id, temp_prediction_obstacle.perception_obstacle(),
          prediction_trajectory->trajectory(),
          temp_prediction_obstacle.is_static()));
      ++trajectory_index;
    }
  }
  return obstacles;
}

bool Obstacle::IsValidTrajectoryPoint(const pnc::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().theta()) ||
           std::isnan(point.v()) ||
           std::isnan(point.a()) ||
           std::isnan(point.da()) || std::isnan(point.relative_time()));
}

bool Obstacle::IsValidPerceptionObstacle(const pnc::PerceptionObstacle& obstacle) {
  if (obstacle.length() <= 0.0) {
    AERROR << "invalid obstacle length:" << obstacle.length();
    return false;
  }
  if (obstacle.width() <= 0.0) {
    AERROR << "invalid obstacle width:" << obstacle.width();
    return false;
  }
  if (obstacle.height() <= 0.0) {
    AERROR << "invalid obstacle height:" << obstacle.height();
    return false;
  }
  if (obstacle.has_velocity()) {
    if (std::isnan(obstacle.velocity().x()) ||
        std::isnan(obstacle.velocity().y())) {
      AERROR << "invalid obstacle velocity:"
             << obstacle.velocity().DebugString();
      return false;
    }
  }
  for (auto pt : obstacle.polygon_point()) {
    if (std::isnan(pt.x()) || std::isnan(pt.y())) {
      AERROR << "invalid obstacle polygon point:" << pt.DebugString();
      return false;
    }
  }
  return true;
}

std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const pnc::Box2d& obstacle_box) {
  // create a "virtual" perception_obstacle
  pnc::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_heading(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(1);
  perception_obstacle.set_type(
      pnc::PerceptionObstacle::UNKNOWN_UNMOVABLE);

  std::vector<pnc::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}


bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

bool Obstacle::set_longitudinal_decision(const ObjectDecisionType& decision) {
  if (!IsLongitudinalDecision(decision)) {
    AERROR << decision.ShortDebugString() << " is not longitudinal.";
    return false;
  }
  longitudinal_decision_ = decision;
  return true;
}

bool Obstacle::set_lateral_decision(const ObjectDecisionType& decision) {
  if (!IsLateralDecision(decision)) {
    AERROR << decision.ShortDebugString() << " is not lateral.";
    return false;
  }
  lateral_decision_ = decision;
  return true;
}

bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || 
         decision.has_follow() || decision.has_yield();
}

bool Obstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_overtake() || 
         decision.has_nudge();
}

bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

std::string Obstacle::DebugString() const {
  std::stringstream ss;
  ss << "TODO: add debug content";
  return ss.str();
}

} // namespace planning
} // namespace xju