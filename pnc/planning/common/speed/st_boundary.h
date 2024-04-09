/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <string>
#include <limits>

#include "planning/common/speed/st_point.h"
#include "common/math/polygon2d.h"
#include "common/math/line_segment2d.h"
#include "object_decision.pb.h"

namespace xju {
namespace planning {

class STBoundary : public pnc::Polygon2d {
 public:
  STBoundary() = default;
  ~STBoundary() = default;

  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
      bool is_accurate_boundary = true);

  explicit STBoundary(const std::vector<pnc::Vec2d>& points) = delete;

  static STBoundary CreateInstance(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  bool IsEmpty() const { return lower_points_.empty(); };

  bool GetUnblockSRange(const double t, double& lower, double& upper,
                        const ObjectDecisionType& lon_decision) const;
  bool GetBoundarySRange(const double t, double& lower, double& upper) const;
  bool GetBoundarySlopes(const double t, double& ds_lower, double& ds_upper) const;

  enum BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  static std::string TypeName(const BoundaryType& type);
  BoundaryType boundary_type() const { return boundary_type_; };
  const std::string id() const { return id_; };

  void set_id(const std::string& id) { id_ = id; };

  void set_boundary_type(const BoundaryType& type) { boundary_type_ = type; };

  double min_s() const { return min_s_; };
  double min_t() const { return min_t_; };
  double max_s() const { return max_s_; };
  double max_t() const { return max_t_; };

  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }

  bool IsPointInBoundary(const STPoint& st_point) const;
  STBoundary ExpandByS(const double s) const;
  STBoundary ExpandByT(const double t) const;

  STPoint upper_left_point() const;
  STPoint upper_right_point() const;
  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;

  void set_upper_left_point(const STPoint& st_point);
  void set_upper_right_point(const STPoint& st_point);
  void set_bottom_left_point(const STPoint& st_point);
  void set_bottom_right_point(const STPoint& st_point);



  void SetCharacteristicLength(const double characteristic_length);
  std::string DebugString() const;
  
 private:
  bool IsValid(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;
  void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);
  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;
  bool IsPointNear(const pnc::LineSegment2d& seg,
                   const pnc::Vec2d& point, const double max_dist);

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  std::string id_;
  double characteristic_length_ = 0.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
  

};

} // namespace planning
} // namespace xju
