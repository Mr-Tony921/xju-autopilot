/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "pnc_map/reference_line.h"

#include <algorithm>
#include <iterator>
#include <limits>

#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/logger/logger.h"
#include "common/math/cartesian_frenet_conversion.h"

namespace xju {
namespace pnc_map {

const std::string ReferenceLine::id() const {
  return id_;
}

const std::string ReferenceLine::left_lane_id() const {
  return left_lane_id_;
}

const std::string ReferenceLine::right_lane_id() const {
  return right_lane_id_;
}

const std::vector<pnc::PathPoint>& 
ReferenceLine::reference_points() const {
  return path_points_;
}

double ReferenceLine::length() const {
  return length_;
}

bool ReferenceLine::is_emergency_lane() const {
  return is_emergency_lane_;
}

bool ReferenceLine::is_death_lane() const {
  return is_death_lane_;
}

bool ReferenceLine::is_recommended_lane() const {
  return is_emergency_lane_;
}

const double ReferenceLine::max_speed_limit() const {
  return max_speed_limit_;
}

const double ReferenceLine::min_speed_limit() const {
  return min_speed_limit_;
}

const pnc::LaneOrder ReferenceLine::lane_order() const {
  return lane_order_;
}
  
bool ReferenceLine::GetLaneWidth(
    const double s, double& left, double& right) const{
  if (lane_width_section_.empty()) {
    left = lane_width_ / 2.0;
    right = lane_width_ / 2.0;
    return true;
  }
  LookUpWidthByS(lane_width_section_, s, left, right);
  return true;
}

bool ReferenceLine::GetRoadWidth(
    const double s, double& left, double& right) const{
  if (road_width_section_.empty()) {
    left = left_road_width_;
    right = right_road_width_;
    return true;
  }
  LookUpWidthByS(road_width_section_, s, left, right);
  return true;
}

bool ReferenceLine::GetLaneMarkingType(
    const double s, pnc::LaneMarkingType& left,  pnc::LaneMarkingType& right) const {
  if (lane_marking_type_section_.empty()) {
    left = left_lane_marking_type_;
    right = right_lane_marking_type_;
    return true;
  } else {
    auto func = [](const LaneMarkingTypeSection& mc, const double s) {
      return mc.first < s;
    };
    auto it_lower = std::lower_bound(
        lane_marking_type_section_.begin(), lane_marking_type_section_.end(), s, func);
    if (it_lower == lane_marking_type_section_.begin()) {
      left = lane_marking_type_section_.front().second.first;
      right = lane_marking_type_section_.front().second.second;
      return true;
    }
    
    if (it_lower == lane_marking_type_section_.end()) {
      left = lane_marking_type_section_.back().second.first;
      right = lane_marking_type_section_.back().second.second;
      return true;
    }

    double lower_s = (it_lower -1)->first;
    double upper_s = it_lower->first;

    if (2.0 * s < lower_s + upper_s) {
      left = (it_lower -1)->second.first;
      right = (it_lower -1)->second.second;
    } else {
      left = it_lower->second.first;
      right = it_lower->second.second;
    }
    return true;
  }
}

bool ReferenceLine::GetRestrictedLaneType(
    const double s, pnc::RestrictedLaneType& type) const {
  if (restricted_lane_type_section_.empty()) {
    type = restricted_lane_type_;
    return true;
  } else {
    auto func = [](const RestrictedLaneTypeSection& ac, const double s) {
      return ac.first < s;
    };
    auto it_lower = std::lower_bound(
        restricted_lane_type_section_.begin(), 
        restricted_lane_type_section_.end(), s, func);
    if (it_lower == restricted_lane_type_section_.begin()) {
      type = restricted_lane_type_section_.front().second;
      return true;
    }

    if (it_lower == restricted_lane_type_section_.end()) {
      type = restricted_lane_type_section_.back().second;
      return true;
    }

    double lower_s = (it_lower -1)->first;
    double upper_s = it_lower->first;

    if (2.0 * s < lower_s + upper_s) {
      type = (it_lower -1)->second;
    } else {
      type = it_lower->second;
    }
    return true;
  }
}

pnc::PathPoint ReferenceLine::GetReferencePoint(
    const double s) const {
  CHECK_GE(num_of_points_, 0U);

  if (num_of_points_ < 2) {
    return path_points_.front();
  }

  auto func = [](const pnc::PathPoint& pt, const double s) {
    return pt.s() < s;
  };

  auto it_lower = std::lower_bound(path_points_.begin(), path_points_.end(), s, func);
  if (it_lower == path_points_.begin()) {
    return path_points_.front();
  } else if (it_lower == path_points_.end()) {
    return path_points_.back();
  } else {
    return pnc::Interpolate(*(it_lower - 1), *it_lower, s);
  }
}

pnc::PathPoint ReferenceLine::GetReferencePoint(
    const double x, const double y) const {
  CHECK_GE(num_of_points_, 0U);

  if (num_of_points_ < 2) {
    return path_points_.front();
  }
  
  pnc::SLPoint sl_point;
  XYToSL({x,y}, &sl_point);
  return GetReferencePoint(sl_point.s());
}

pnc::FrenetFramePoint ReferenceLine::GetFrenetPoint(
    const pnc::PathPoint& path_point) const {
  if (path_points_.empty()) {
    return pnc::FrenetFramePoint();
  }

  pnc::SLPoint sl_point;
  XYToSL(path_point, &sl_point);
  pnc::FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();
  const double kappa = path_point.kappa();
  const double l = frenet_frame_point.l();

  pnc::PathPoint ref_point = GetReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.theta();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = pnc::CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      pnc::CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(
    const pnc::TrajectoryPoint& trajectory_point) const{
  ACHECK(!path_points_.empty());

  pnc::SLPoint sl_point;
  XYToSL(trajectory_point.path_point(), &sl_point);

  std::array<double, 3> s_condition;
  std::array<double, 3> l_condition;
  pnc::PathPoint ref_point = GetReferencePoint(sl_point.s());
  pnc::CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), ref_point.x(), ref_point.y(), ref_point.theta(),
      ref_point.kappa(), ref_point.dkappa(), trajectory_point.path_point().x(),
      trajectory_point.path_point().y(), trajectory_point.v(), trajectory_point.a(),
      trajectory_point.path_point().theta(), trajectory_point.path_point().kappa(),
      &s_condition, &l_condition);

  return std::make_pair(s_condition, l_condition);
}

bool ReferenceLine::XYToSL(const pnc::Vec2d& xy_point, 
                           pnc::SLPoint* const sl_point) const {
  double s = 0.0;
  double l = 0.0;
  if (!segments_.GetProjection(xy_point, &s, &l)) {
    AERROR << "Cannot get nearest point fromt reference line.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

bool ReferenceLine::SLToXY(const pnc::SLPoint& sl_point, 
                           pnc::Vec2d* const xy_point) const {
  if (segments_.size() < 1) {
    AERROR << "The reference line has too few points.";
    return false;
  }

  const auto matched_point = GetReferencePoint(sl_point.s());
  xy_point->set_x(matched_point.x() - sl_point.l() * std::sin(matched_point.theta()));
  xy_point->set_y(matched_point.y() + sl_point.l() * std::cos(matched_point.theta()));
  return true;
}

bool ReferenceLine::GetSLBoundary(
    const pnc::Box2d& box, planning::SLBoundary& sl_boundary) const {
  
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<pnc::Vec2d> corners;
  box.GetAllCorners(&corners);

  // The order must be counter-clockwise
  std::vector<pnc::SLPoint> sl_corners;
  for (const auto& point : corners) {
    pnc::SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AERROR << "Failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % corners.size();
    const auto& p0 = corners[index0];
    const auto& p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    pnc::SLPoint sl_point_mid;
    if (!XYToSL(p_mid, &sl_point_mid)) {
      AERROR << "Failed to get projection for point: " << p_mid.DebugString()
             << " on reference line.";
      return false;
    }

    pnc::Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(),
                  sl_corners[index1].l() - sl_corners[index0].l());

    pnc::Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(),
                  sl_point_mid.l() - sl_corners[index0].l());

    *sl_boundary.add_boundary_point() = sl_corners[index0];

    // sl_point is outside of polygon; add to the vertex list
    if (v0.CrossProd(v1) < 0.0) {
      *sl_boundary.add_boundary_point() = sl_point_mid;
    }
  }

  for (const auto& sl_point : sl_boundary.boundary_point()) {
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }

  sl_boundary.set_start_s(start_s);
  sl_boundary.set_end_s(end_s);
  sl_boundary.set_start_l(start_l);
  sl_boundary.set_end_l(end_l);
  return true;
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const{
  ACHECK(!speed_limit_.empty());
  for(const auto& speed_limit : speed_limit_){
    if(s >= speed_limit.start_s && s <= speed_limit.end_s){
      return speed_limit.speed_limit;
    }
  }
  if(s < speed_limit_.front().start_s){
    return speed_limit_.front().speed_limit;
  }
  return speed_limit_.back().speed_limit;
}

void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  std::vector<SpeedLimit> new_speed_limit;
  for (const auto& limit : speed_limit_) {
    if (start_s >= limit.end_s || end_s <= limit.start_s) {
      new_speed_limit.emplace_back(limit);
    } else {
      // start_s < speed_limit.end_s && end_s > speed_limit.start_s
      double min_speed = std::min(limit.speed_limit, speed_limit);
      if (start_s >= limit.start_s) {
        new_speed_limit.emplace_back(limit.start_s, start_s, min_speed);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed);
        }
      } else {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed);
        }
      }
      start_s = limit.end_s;
      end_s = std::max(end_s, limit.end_s);
    }
  }
  speed_limit_.clear();
  if (end_s > start_s) {
    new_speed_limit.emplace_back(start_s, end_s, speed_limit);
  }
  for (const auto& limit : new_speed_limit) {
    if (limit.start_s < limit.end_s) {
      speed_limit_.emplace_back(limit);
    }
  }
  std::sort(speed_limit_.begin(), speed_limit_.end(),
            [](const SpeedLimit& a, const SpeedLimit& b) {
              if (a.start_s != b.start_s) {
                return a.start_s < b.start_s;
              }
              if (a.end_s != b.end_s) {
                return a.end_s < b.end_s;
              }
              return a.speed_limit < b.speed_limit;
            });
}

bool ReferenceLine::IsOnLane(const pnc::Vec2d& xy) const {
  pnc::SLPoint sl_point;
  if (!XYToSL(xy, &sl_point)) {
    return false;
  }
  return IsOnLane(sl_point);
}

bool ReferenceLine::IsOnLane(const pnc::SLPoint& sl_point) const {
  if (sl_point.s() <= 0.0 || sl_point.s() > length_) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s(), left_width, right_width)) {
    return false;
  }

  return sl_point.l() >= -right_width && sl_point.l() <= left_width;
}

bool ReferenceLine::IsOnLane(const planning::SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length_) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double left_width = 0.0;
  double right_width = 0.0;
  GetLaneWidth(middle_s, left_width, right_width);
  return sl_boundary.start_l() <= left_width &&
         sl_boundary.end_l() >= -right_width;
}

bool ReferenceLine::IsOnRoad(const pnc::Vec2d& xy) const {
  pnc::SLPoint sl_point;
  if (!XYToSL(xy, &sl_point)) {
    return false;
  }
  return IsOnRoad(sl_point);
}

bool ReferenceLine::IsOnRoad(const pnc::SLPoint& sl_point) const {
  if (sl_point.s() <= 0.0 || sl_point.s() > length_) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetRoadWidth(sl_point.s(), left_width, right_width)) {
    return false;
  }

  return sl_point.l() >= -right_width && sl_point.l() <= left_width;
}

bool ReferenceLine::IsOnRoad(const planning::SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length_) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(middle_s, left_width, right_width);
  return sl_boundary.start_l() <= left_width &&
         sl_boundary.end_l() >= -right_width;
}

void ReferenceLine::LookUpWidthByS(
    const std::vector<WidthSection>& width_section, const double s,
    double& left, double& right) const {
  auto func = [](const WidthSection& ws, const double s) {
    return ws.first < s;
  };
  auto it_lower = std::lower_bound(width_section.begin(), width_section.end(), s, func);
  if (it_lower == width_section.begin()) {
    left = width_section.front().second.first;
    right = width_section.front().second.second;
    return;
  }
  if (it_lower == width_section.end()) {
    left = width_section.back().second.first;
    right = width_section.back().second.second;
    return;
  }

  double lower_s = (it_lower - 1)->first;
  double upper_s = it_lower->first;
  std::pair<double, double> lower_pair = (it_lower - 1)->second;
  std::pair<double, double> upper_pair = it_lower->second;

  left = pnc::lerp(lower_s, lower_pair.first, upper_s, upper_pair.first, s);
  right = pnc::lerp(lower_s, lower_pair.second, upper_s, upper_pair.second, s);
}

void ReferenceLine::set_id(const std::string& id) {
  id_ = id;
}

void ReferenceLine::set_left_lane_id(const std::string& id) {
  left_lane_id_ = id;
}

void ReferenceLine::set_right_lane_id(const std::string& id) {
  right_lane_id_ = id;
}

void ReferenceLine::set_path_points(
    const std::vector<pnc::PathPoint>& points) {
  path_points_ = points;
  num_of_points_ = path_points_.size();
  double s = 0.0;
  accumulated_s_.push_back(s);

  if (num_of_points_ < 2) {
    return;
  }
  
  for (int i = 1; i < num_of_points_; ++i) {
    segments_.EmplaceBack(path_points_[i - 1], path_points_[i]);
    s += pnc::Distance(path_points_[i - 1], path_points_[i]);
    accumulated_s_.push_back(s);
  }
  length_ = accumulated_s_.back();
}

void ReferenceLine::set_lane_width(const double width) {
  lane_width_ = width;
}

void ReferenceLine::set_left_road_width(const double width) {
  left_road_width_ = width;
}

void ReferenceLine::set_right_road_width(const double width) {
  right_road_width_ = width;
}

void ReferenceLine::set_min_speed_limit(const double limit) {
  min_speed_limit_ = limit;
}

void ReferenceLine::set_max_speed_limit(const double limit) {
  max_speed_limit_ = limit;
}

void ReferenceLine::set_left_lane_marking_type(
    const pnc::LaneMarkingType& type) {
  left_lane_marking_type_ = type;
}

void ReferenceLine::set_right_lane_marking_type(
    const pnc::LaneMarkingType& type) {
  right_lane_marking_type_ = type;
}

void ReferenceLine::set_restricted_lane_type(
    const pnc::RestrictedLaneType& type) {
  restricted_lane_type_ = type;
}

void ReferenceLine::set_lane_order(const pnc::LaneOrder& order) {
  lane_order_ = order;
}

void ReferenceLine::set_is_emergency_lane(const bool value) {
  is_emergency_lane_ = value;
}

void ReferenceLine::set_is_death_lane(const bool value) {
  is_death_lane_ = value;
}

void ReferenceLine::set_is_recommended_lane(const bool value) {
  is_recommended_lane_ = value;
}

} // namespace pnc_map
} // namespace xju
