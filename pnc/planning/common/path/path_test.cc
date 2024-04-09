/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/path/path_boundary.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/path/path_data.h"

#include <cmath>

#include "common/math/math_utils.h"
#include "gtest/gtest.h"

namespace xju {
namespace planning {

std::vector<pnc::PathPoint> GeneratePathPoints() {
  std::vector<pnc::PathPoint> points;
  for (int i = 0; i < 201; ++i) {
    pnc::PathPoint pt;
    pt.set_x(0.0);
    pt.set_y(i / 2.0);
    pt.set_z(0.0);
    pt.set_theta(M_PI_2);
    pt.set_kappa(0.0);
    pt.set_dkappa(0.0);
    pt.set_ddkappa(0.0);
    pt.set_s(i / 2.0);
    points.push_back(pt);
  }
  return points;
}

std::vector<std::pair<double, double>> GenerateLaneBoundary() {
  std::vector<std::pair<double, double>> bounds;
  for (int i = 0; i < 10; ++i) {
    bounds.emplace_back(std::make_pair(-1.75, 1.75));
  }
  return bounds;
}

std::vector<std::pair<double, double>> GenerateObstacleBoundary() {
  std::vector<std::pair<double, double>> bounds;
  for (int i = 0; i < 5; ++i) {
    bounds.emplace_back(std::make_pair(-10, 10));
  }

  for (int i = 0; i < 5; ++i) {
    bounds.emplace_back(std::make_pair(0.0, 1.0));
  }
  return bounds;
}

TEST(PathBoundary, Init) {
  auto lane_bound = GenerateLaneBoundary();
  auto obs_bound = GenerateObstacleBoundary();
  PathBoundary path_boundary(10.0, 0.5, lane_bound, obs_bound);
  
  double start_s = path_boundary.start_s();
  double delta_s = path_boundary.delta_s();
  auto lane_boundary = path_boundary.lane_boundary();
  auto obstacle_boundary = path_boundary.obstacle_boundary();
  auto boundary = path_boundary.path_boundary();

  EXPECT_NEAR(start_s, 10.0, pnc::kMathEpsilon);
  EXPECT_NEAR(delta_s, 0.5, pnc::kMathEpsilon);
  EXPECT_EQ(lane_boundary.size(), 10);
  EXPECT_EQ(obstacle_boundary.size(), 10);
  EXPECT_EQ(boundary.size(), 10);

  for (int i = 0; i < 5; ++i) {
    EXPECT_NEAR(boundary[i].first, -1.75, pnc::kMathEpsilon);
    EXPECT_NEAR(boundary[i].second, 1.75, pnc::kMathEpsilon);
  }

  for (int i = 5; i < 10; ++i) {
    EXPECT_NEAR(boundary[i].first, 0.0, pnc::kMathEpsilon);
    EXPECT_NEAR(boundary[i].second, 1.0, pnc::kMathEpsilon);
  }
}

TEST(DiscretizedPath, Init) {
  auto pts = GeneratePathPoints();
  DiscretizedPath path(pts);
  EXPECT_NEAR(path.Length(), 100.0, pnc::kMathEpsilon);

  pnc::PathPoint pt = path.Evaluate(-2.0);
  EXPECT_NEAR(pt.x(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.y(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.z(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.theta(), M_PI_2, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.kappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.dkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.s(), 0.0, pnc::kMathEpsilon);

  pt = path.Evaluate(120.0);
  EXPECT_NEAR(pt.x(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.y(), 100.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.z(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.theta(), M_PI_2, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.kappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.dkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.s(), 100.0, pnc::kMathEpsilon);

  pt = path.Evaluate(10.0);
  EXPECT_NEAR(pt.x(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.y(), 10.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.z(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.theta(), M_PI_2, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.kappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.dkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.s(), 10.0, pnc::kMathEpsilon);

  pt = path.Evaluate(50.5);
  EXPECT_NEAR(pt.x(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.y(), 50.5, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.z(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.theta(), M_PI_2, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.kappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.dkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.s(), 50.5, pnc::kMathEpsilon);
}

TEST(PathData, All) {
  std::vector<pnc::PathPoint> points;
  for (int i = 0; i < 3; ++i) {
    pnc::PathPoint pt;
    pt.set_x(0.0);
    pt.set_y(i / 2.0);
    pt.set_z(0.0);
    pt.set_theta(M_PI_2);
    pt.set_kappa(0.0);
    pt.set_dkappa(0.0);
    pt.set_ddkappa(0.0);
    pt.set_s(i / 2.0);
    points.push_back(pt);
  }

  
  std::vector<pnc::FrenetFramePoint> frenet_points;
  for (int i = 0; i < 3; ++i) {
    pnc::FrenetFramePoint pt;
    pt.set_l(i);
    pt.set_heading_error(M_PI_4);
    pt.set_kappa(i);
    frenet_points.push_back(pt);
  }

  PathData path_data;
  path_data.set_reference_points(points);
  path_data.set_frenet_points(frenet_points);

  auto planned_path = path_data.planned_path();
  EXPECT_EQ(planned_path.size(), 3);
  
  double s = 0.0;
  for (int i = 0; i < 3; ++i) {
    auto pt = planned_path[i];
    auto f_pt = frenet_points[i];

    EXPECT_NEAR(pt.x(), -f_pt.l() * std::sin(M_PI_2), pnc::kMathEpsilon);
    EXPECT_NEAR(pt.y(), i / 2.0, pnc::kMathEpsilon);
    EXPECT_NEAR(pt.z(), 0.0, pnc::kMathEpsilon);
    EXPECT_NEAR(pt.theta(), pnc::NormalizeAngle(M_PI_2 + M_PI_4), pnc::kMathEpsilon);
    EXPECT_NEAR(pt.kappa(), i, pnc::kMathEpsilon);
    // EXPECT_NEAR(pt.dkappa(), 0.0, pnc::kMathEpsilon);
    EXPECT_NEAR(pt.s(), s, pnc::kMathEpsilon);
    
    if (i < 2) {
      s += pnc::Distance(planned_path[i], planned_path[i + 1]);
    }
    
  }

  pnc::PathPoint pt = path_data.GetPathPointByS(0.0);
  EXPECT_NEAR(pt.x(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.y(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.z(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.theta(), pnc::NormalizeAngle(M_PI_2 + M_PI_4), pnc::kMathEpsilon);
  EXPECT_NEAR(pt.kappa(), 0.0, pnc::kMathEpsilon);
    // EXPECT_NEAR(pt.dkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(pt.s(), 0.0, pnc::kMathEpsilon);
}

} // namespace planning
} // namespace xju