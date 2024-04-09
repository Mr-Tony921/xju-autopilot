/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/polygon2d.h"

#include <algorithm>

#include "common/third_party/collision_lib/gjk.h"
#include "common/logger/logger.h"
#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

Polygon2d::Polygon2d(const std::vector<Vec2d>& points) 
    : corners_(std::move(points)) {
  BuildFromPoints();
}

Polygon2d::Polygon2d(const Box2d& box) {
  corners_ = box.corners();
  BuildFromPoints();
}

bool Polygon2d::IsOverlap(Polygon2d& polygon) {
  if (!is_convex_) {
    ComputeConvexHull();
    ACHECK(is_convex_);
  }
  if (polygon.is_convex()) {
    polygon.ComputeConvexHull();
    ACHECK(polygon.is_convex());
  }

  if (polygon.max_x() < min_x_ || polygon.min_x() > max_x_ ||
      polygon.max_y() < min_y_ || polygon.min_y() > max_y_) {
    return false;
  }

  return GjkLevel1(polygon);
}

bool Polygon2d::IsOverlap(const Box2d& box) {
  if (!is_convex_) {
    ComputeConvexHull();
    ACHECK(is_convex_);
  }

  if (box.max_x() < min_x_ || box.min_x() > max_x_ ||
      box.max_y() < min_y_ || box.min_y() > max_y_) {
    return false;
  }

  return GjkLevel1(box);
}

bool Polygon2d::ComputeConvexHull() {
  if (num_of_corners_ < 3) {
    return false;
  }
  std::vector<int> sorted_indices(num_of_corners_);
  for (int i = 0; i < num_of_corners_; ++i) {
    sorted_indices[i] = i;
  }
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const int idx1, const int idx2) {
              const Vec2d& pt1 = corners_[idx1];
              const Vec2d& pt2 = corners_[idx2];
              const double dx = pt1.x() - pt2.x();
              if (std::abs(dx) > kMathEpsilon) {
                return dx < 0.0;
              }
              return pt1.y() < pt2.y();
            });
  int count = 0;
  std::vector<int> results;
  results.reserve(num_of_corners_);
  int last_count = 1;
  for (int i = 0; i < 2 * num_of_corners_; ++i) {
    if (i == num_of_corners_) {
      last_count = count;
    }
    const int idx = sorted_indices[(i < num_of_corners_) ? i : (2 * num_of_corners_ - 1 - i)];
    const Vec2d &pt = corners_[idx];
    while (count > last_count &&
           CrossProd(corners_[results[count - 2]], corners_[results[count - 1]],
                     pt) <= kMathEpsilon) {
      results.pop_back();
      --count;
    }
    results.push_back(idx);
    ++count;
  }
  --count;
  if (count < 3) {
    return false;
  }
  std::vector<Vec2d> result_points;
  result_points.reserve(count);
  for (int i = 0; i < count; ++i) {
    result_points.push_back(corners_[results[i]]);
  }
  *this = Polygon2d(result_points);
  return true;
}

double Polygon2d::DistanceTo(const Polygon2d& polygon) {
  return GjkLevel2(polygon);
}

void Polygon2d::BuildFromPoints() {
  num_of_corners_ = static_cast<int>(corners_.size());
  CHECK_GE(num_of_corners_, 3);

  // Make sure the points are in ccw order
  double area = 0.0;
  for (int i = 1; i < num_of_corners_; ++i) {
    area += CrossProd(corners_[0], corners_[i - 1], corners_[i]);
  }

  if (area < 0) {
    area = -area;
    std::reverse(corners_.begin(), corners_.end());
  }

  area /= 2.0;
  CHECK_GT(area, kMathEpsilon);

  // Check convexity.
  is_convex_ = true;
  for (int i = 0; i < num_of_corners_; ++i) {
    if (CrossProd(corners_[Prev(i)], corners_[i], corners_[Next(i)]) <=
        -kMathEpsilon) {
      is_convex_ = false;
      break;
    }
  }

  for (const auto &corner : corners_) {
    min_x_ = std::min(min_x_, corner.x());
    max_x_ = std::max(max_x_, corner.x());
    min_y_ = std::min(min_y_, corner.y());
    max_y_ = std::max(max_y_, corner.y());
  }
}

int Polygon2d::Next(int at) const { 
  return at >= num_of_corners_ - 1 ? 0 : at + 1;
}

int Polygon2d::Prev(int at) const { 
  return at == 0 ? num_of_corners_ - 1 : at - 1; 
}

bool Polygon2d::GjkLevel1(const Polygon2d& polygon) {
  Gjk::Polygon p, q;
  p.num_vertex = num_of_corners_;
  q.num_vertex = polygon.num_of_corners();

  p.vertices = new double* [p.num_vertex];
  q.vertices = new double* [q.num_vertex];

  for (int i = 0; i < p.num_vertex; ++i) {
    p.vertices[i] = new double[2];
    p.vertices[i][0] = corners_[i].x();
    p.vertices[i][1] = corners_[i].y();
  }

  for (int i = 0; i < q.num_vertex; ++i) {
    q.vertices[i] = new double[2];
    q.vertices[i][0] = polygon.corners()[i].x();
    q.vertices[i][1] = polygon.corners()[i].y();
  }

  bool level_1 = Gjk::GjkLevel1(p, q);

  for (int i = 0; i < p.num_vertex; ++i) {
    delete p.vertices[i];
  }
  for (int i = 0; i < q.num_vertex; ++i) {
    delete q.vertices[i];
  }
  delete p.vertices;
  delete q.vertices;
  
  return level_1;
}

bool Polygon2d::GjkLevel1(const Box2d& box) {
  Gjk::Polygon p, q;
  p.num_vertex = num_of_corners_;
  q.num_vertex = 4;

  p.vertices = new double* [p.num_vertex];
  q.vertices = new double* [q.num_vertex];

  for (int i = 0; i < p.num_vertex; ++i) {
    p.vertices[i] = new double[2];
    p.vertices[i][0] = corners_[i].x();
    p.vertices[i][1] = corners_[i].y();
  }

  for (int i = 0; i < q.num_vertex; ++i) {
    q.vertices[i] = new double[2];
    q.vertices[i][0] = box.corners()[i].x();
    q.vertices[i][1] = box.corners()[i].y();
  }

  bool level_1 = Gjk::GjkLevel1(p, q);

  for (int i = 0; i < p.num_vertex; ++i) {
    delete p.vertices[i];
  }
  for (int i = 0; i < q.num_vertex; ++i) {
    delete q.vertices[i];
  }
  delete p.vertices;
  delete q.vertices;

  return level_1;
}

double Polygon2d::GjkLevel2(const Polygon2d& polygon) {
  Gjk::Polygon p, q;
  p.num_vertex = num_of_corners_;
  q.num_vertex = polygon.num_of_corners();

  p.vertices = new double* [p.num_vertex];
  q.vertices = new double* [q.num_vertex];

  for (int i = 0; i < p.num_vertex; ++i) {
    p.vertices[i] = new double[2];
    p.vertices[i][0] = corners_[i].x();
    p.vertices[i][1] = corners_[i].y();
  }

  for (int i = 0; i < q.num_vertex; ++i) {
    q.vertices[i] = new double[2];
    q.vertices[i][0] = polygon.corners()[i].x();
    q.vertices[i][1] = polygon.corners()[i].y();
  }

  double level_2 = Gjk::GjkLevel2(p, q);

  for (int i = 0; i < p.num_vertex; ++i) {
    delete p.vertices[i];
  }
  for (int i = 0; i < q.num_vertex; ++i) {
    delete q.vertices[i];
  }
  delete p.vertices;
  delete q.vertices;
  
  return level_2;
}

} // namespace pnc
} // namespace xju
