/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <string>

#include "common/math/line_segments.h"
#include "common/math/box2d.h"
#include "sl_boundary.pb.h"
#include "pnc_point.pb.h"
#include "em_lane.pb.h"

namespace xju {
namespace pnc_map {

namespace {
  using WidthSection = std::pair<double, std::pair<double, double>>;
  using LaneMarkingTypeSection = std::pair<double, 
      std::pair<pnc::LaneMarkingType, pnc::LaneMarkingType>>;
  using RestrictedLaneTypeSection = std::pair<double, pnc::RestrictedLaneType>;
}

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;

  const std::string id() const;
  const std::string left_lane_id() const;
  const std::string right_lane_id() const;
  bool is_emergency_lane() const;
  bool is_death_lane() const;
  bool is_recommended_lane() const;
  double length() const;
  const pnc::LaneOrder lane_order() const;

  const std::vector<pnc::PathPoint>& reference_points() const;
  
  bool GetLaneWidth(const double s, double& left, double& right) const;
  bool GetRoadWidth(const double s, double& left, double& right) const;

  const double max_speed_limit() const;
  const double min_speed_limit() const;

  bool GetLaneMarkingType(
      const double s, pnc::LaneMarkingType& left,  pnc::LaneMarkingType& right) const;

  bool GetRestrictedLaneType(const double s, pnc::RestrictedLaneType& type) const;

  pnc::PathPoint GetReferencePoint(const double s) const;
  pnc::PathPoint GetReferencePoint(const double x, const double y) const;

  bool XYToSL(const pnc::Vec2d& xy_point, pnc::SLPoint* const sl_point) const;

  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, pnc::SLPoint* const sl_point) const {
    return XYToSL(pnc::Vec2d(xy.x(), xy.y()), sl_point);
  }

  bool SLToXY(const pnc::SLPoint& sl_point, pnc::Vec2d* const xy_point) const;

  pnc::FrenetFramePoint GetFrenetPoint(const pnc::PathPoint& path_point) const;

  std::pair<std::array<double, 3>, std::array<double, 3>> 
  ToFrenetFrame(const pnc::TrajectoryPoint& trajectory_point) const;

  bool GetSLBoundary(const pnc::Box2d& box, planning::SLBoundary& sl_boundary) const;

  double GetSpeedLimitFromS(const double s) const;

  void AddSpeedLimit(double start_s, double end_s, double speed_limit);
  
  bool IsOnLane(const pnc::Vec2d& xy) const;
  
  template <class XYPoint> 
  bool IsOnLane(const XYPoint& xy) const {
    return IsOnLane(pnc::Vec2d(xy.x(), xy.y()));
  }

  bool IsOnLane(const pnc::SLPoint& sl_point) const;

  bool IsOnLane(const planning::SLBoundary& sl_boundary) const;

  bool IsOnRoad(const pnc::Vec2d& xy) const;
  
  template <class XYPoint> 
  bool IsOnRoad(const XYPoint& xy) const {
    return IsOnRoad(pnc::Vec2d(xy.x(), xy.y()));
  }

  bool IsOnRoad(const pnc::SLPoint& sl_point) const;
  
  bool IsOnRoad(const planning::SLBoundary& sl_boundary) const;

  void set_id(const std::string& id);

  void set_left_lane_id(const std::string& id);

  void set_right_lane_id(const std::string& id);

  void set_path_points(const std::vector<pnc::PathPoint>& points);

  void set_lane_width(const double width);

  void set_left_road_width(const double width);

  void set_right_road_width(const double width);

  void set_min_speed_limit(const double limit);

  void set_max_speed_limit(const double limit);

  void set_left_lane_marking_type(const pnc::LaneMarkingType& type);

  void set_right_lane_marking_type(const pnc::LaneMarkingType& type);

  void set_restricted_lane_type(const pnc::RestrictedLaneType& type);

  void set_lane_order(const pnc::LaneOrder& order);

  void set_is_emergency_lane(const bool value);

  void set_is_death_lane(const bool value);

  void set_is_recommended_lane(const bool value);

 private:
  void LookUpWidthByS(
      const std::vector<WidthSection>& width_section, const double s,
      double& left, double& right) const;

 private:
  std::string id_;
  std::string left_lane_id_;
  std::string right_lane_id_;
  double length_;
  
  int num_of_points_;
  std::vector<pnc::PathPoint> path_points_;
  // the size is equal to (num_of_points_ - 1)
  pnc::LineSegments segments_;
  // the size is equal to num_of_points_
  std::vector<double> accumulated_s_;
  
  double lane_width_;
  double left_road_width_;
  double right_road_width_;

  double min_speed_limit_;
  double max_speed_limit_;

  pnc::LaneMarkingType left_lane_marking_type_;
  pnc::LaneMarkingType right_lane_marking_type_;
  pnc::RestrictedLaneType restricted_lane_type_;
  pnc::LaneOrder lane_order_;

  bool is_emergency_lane_ = false;
  bool is_death_lane_ = false;
  bool is_recommended_lane_ = false;

  std::vector<WidthSection> lane_width_section_;
  std::vector<WidthSection> road_width_section_;

  std::vector<LaneMarkingTypeSection> lane_marking_type_section_;
  std::vector<RestrictedLaneTypeSection> restricted_lane_type_section_;

  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  std::vector<SpeedLimit> speed_limit_;
};

} // namespace pnc_map
} // namespace xju
