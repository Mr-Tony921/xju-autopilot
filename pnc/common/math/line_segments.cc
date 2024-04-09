/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/line_segments.h"

#include "common/logger/logger.h"
#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

LineSegments::LineSegments(const std::vector<Vec2d>& points) {
  points_ = points;
  for (std::size_t i = 0; i < (int)points.size() - 1; i++) {
    std::vector<LineSegment2d>::emplace_back(points[i], points[i + 1]);
  }
  for (auto i = 0; i < points.size(); i++) {
    if (i == 0) {
      accumulated_s_.push_back(0.0);
    } else {
      accumulated_s_.push_back(accumulated_s_.back() +
                               points[i - 1].DistanceTo(points[i]));
    }
  }
}

void LineSegments::PushBack(const LineSegment2d& line_seg) {
  if (empty()) {
    points_.push_back(line_seg.start());
    points_.push_back(line_seg.end());
    accumulated_s_.push_back(0.0);
    accumulated_s_.push_back(line_seg.length());
  } else {
    points_.push_back(line_seg.end());
    accumulated_s_.push_back(accumulated_s_.back() + line_seg.length());
  }
  std::vector<LineSegment2d>::push_back(std::move(line_seg));
}

void LineSegments::EmplaceBack(const LineSegment2d& line_seg) {
  PushBack(line_seg);
}

void LineSegments::EmplaceBack(const Vec2d& start, const Vec2d& end) {
  if (empty()) {
    points_.push_back(start);
    points_.push_back(end);
    accumulated_s_.push_back(0.0);
    accumulated_s_.push_back(start.DistanceTo(end));
  } else {
    points_.push_back(end);
    accumulated_s_.push_back(accumulated_s_.back() + start.DistanceTo(end));
  }
  std::vector<LineSegment2d>::emplace_back(start, end);
}

std::vector<double> LineSegments::GetAccumulatedS() const {
  return accumulated_s_;
}

std::vector<Vec2d> LineSegments::GetPoints() const { return points_; }

bool LineSegments::GetProjection(const Vec2d& point, double* accumulate_s,
                                 double* lateral, double* min_distance) const {
  if (empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }

  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < size(); ++i) {
    const double distance = data()[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = data()[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);

  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == static_cast<int>(size()) - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

bool LineSegments::GetProjection(const Vec2d& point, double* accumulate_s,
                                 double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

bool LineSegments::GetXY(double s, double* x, double* y, int* index) {
  if (size() < 1) {
    return false;
  }
  if (s <= 0) {
    *x = points_.front().x();
    *y = points_.front().y();
    if (index) {
      *index = 0;
    }
  } else if (s >= Length()) {
    *x = points_.back().x();
    *y = points_.back().y();
    if (index) {
      *index = points_.size() - 1;
    }
  } else {
    int start_index = 0;
    for (int i = 0; i < accumulated_s_.size() - 1; i++) {
      if (s >= accumulated_s_[i] && s < accumulated_s_[i + 1]) {
        start_index = i;
        break;
      }
    }
    int end_index = start_index + 1;
    double ratio = (s - accumulated_s_[start_index]) /
                   (accumulated_s_[end_index] - accumulated_s_[start_index]);
    auto v = (points_[end_index] - points_[start_index]) * ratio +
             points_[start_index];
    *x = v.x();
    *y = v.y();
    if (index) {
      *index = start_index;
    }
  }
  return true;
}

double LineSegments::Length() {
  if (empty()) {
    return 0.0;
  }
  return accumulated_s_.back();
}

}  // namespace pnc
}  // namespace xju