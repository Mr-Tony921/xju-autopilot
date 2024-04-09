/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"

namespace xju {
namespace pnc {

class LineSegments : public std::vector<LineSegment2d> {
 public:
  // reference to apollo/modules/map/pnc_map/path.h
  // Path::GetProjection
  LineSegments() = default;

  ~LineSegments() = default;

  LineSegments(const std::vector<Vec2d>& points);

  void PushBack(const LineSegment2d& line_seg);

  void EmplaceBack(const LineSegment2d& line_seg);

  void EmplaceBack(const Vec2d& start, const Vec2d& end);

  std::vector<double> GetAccumulatedS() const;

  std::vector<Vec2d> GetPoints() const;

  bool GetProjection(const Vec2d& point, double* accumulate_s,
                     double* lateral) const;

  bool GetProjection(const Vec2d& point, double* accumulate_s, double* lateral,
                     double* min_distance) const;

  bool GetXY(double s, double* x, double* y, int* index = nullptr);

  double Length();

 private:
  std::vector<Vec2d> points_;  // size()+1

  std::vector<double> accumulated_s_;  // size()+1

 private:
  void push_back() = delete;

  void emplace_back() = delete;
};

}  // namespace pnc
}  // namespace xju
