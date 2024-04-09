/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/speed/st_boundary.h"

#include <cmath>

#include "common/math/math_utils.h"
#include "common/logger/logger.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "gtest/gtest.h"

namespace xju {
namespace planning {

using xju::pnc::kMathEpsilon;

constexpr int TEST_POINTS_SIZE = 100;
std::vector<std::pair<STPoint, STPoint>> point_pairs;

void CreateSTPointPairs() {
  point_pairs.clear();
  for(size_t i = 0; i <= TEST_POINTS_SIZE; ++i) {
    point_pairs.emplace_back(STPoint(20.0, i * 0.1), STPoint(50.0, i * 0.1));
  }
}

TEST(STBoundaryTest, STBoundary) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);
  EXPECT_NEAR(st_boundary.min_t(), 0.0, kMathEpsilon);
  EXPECT_NEAR(st_boundary.min_s(), 20.0, kMathEpsilon);
  EXPECT_NEAR(st_boundary.max_t(), 10.0, kMathEpsilon);
  EXPECT_NEAR(st_boundary.max_s(), 50.0, kMathEpsilon);

  EXPECT_EQ(st_boundary.lower_points().size(), 2);
  EXPECT_EQ(st_boundary.upper_points().size(), 2);
  EXPECT_EQ(st_boundary.corners().size(), 4);
  EXPECT_TRUE(st_boundary.is_convex());
}

TEST(STBoundaryTest, GetUnblockSRange01) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);
  st_boundary.set_boundary_type(STBoundary::BoundaryType::STOP);

  double test_t = 5;
  double res_lower = -1.0;
  double res_upper = -1.0;
  bool res = st_boundary.GetUnblockSRange(test_t, res_lower, res_upper);

  EXPECT_TRUE(res);
  EXPECT_NEAR(res_lower, 0.0, kMathEpsilon);
  EXPECT_NEAR(res_upper, 20.0, kMathEpsilon);
}

TEST(STBoundaryTest, GetUnblockSRange02) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);
  st_boundary.set_boundary_type(STBoundary::BoundaryType::OVERTAKE);

  double test_t = 5;
  double res_lower = -1.0;
  double res_upper = -1.0;
  bool res = st_boundary.GetUnblockSRange(test_t, res_lower, res_upper);

  EXPECT_TRUE(res);
  EXPECT_NEAR(res_lower, 50.0, kMathEpsilon);
  EXPECT_NEAR(res_upper, 200.0, kMathEpsilon);
}

TEST(STBoundaryTest, GetBoundarySRange) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);

  double test_t = 5;
  double res_lower = -1.0;
  double res_upper = -1.0;
  bool res = st_boundary.GetBoundarySRange(test_t, res_lower, res_upper);

  EXPECT_TRUE(res);
  EXPECT_NEAR(res_lower, 20.0, kMathEpsilon);
  EXPECT_NEAR(res_upper, 50.0, kMathEpsilon);
}

TEST(STBoundaryTest, GetBoundarySlopes) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);

  double test_t = 5;
  double res_ds_lower = -1.0;
  double res_ds_upper = -1.0;
  bool res = st_boundary.GetBoundarySlopes(test_t, res_ds_lower, res_ds_upper);

  EXPECT_TRUE(res);
  EXPECT_NEAR(res_ds_lower, 0.0, kMathEpsilon);
  EXPECT_NEAR(res_ds_upper, 0.0, kMathEpsilon);
}

TEST(STBoundaryTest, IsPointInBoundary) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);

  STPoint p1(0.0, 0.0);
  bool res1 = st_boundary.IsPointInBoundary(p1);
  EXPECT_FALSE(res1);

  STPoint p2(30.0, 5.0);
  bool res2 = st_boundary.IsPointInBoundary(p2);
  EXPECT_TRUE(res2);

  STPoint p3(20.0, 10.0);
  bool res3 = st_boundary.IsPointInBoundary(p3);
  EXPECT_FALSE(res3);
}

TEST(STBoundaryTest, ExpandByS) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);
  double test_s = 1.0;
  STBoundary res = st_boundary.ExpandByS(test_s);
  EXPECT_NEAR(res.min_t(), 0.0, kMathEpsilon);
  EXPECT_NEAR(res.min_s(), 19.0, kMathEpsilon);
  EXPECT_NEAR(res.max_t(), 10.0, kMathEpsilon);
  EXPECT_NEAR(res.max_s(), 51.0, kMathEpsilon);
}

TEST(STBoundaryTest, ExpandByT) {
  CreateSTPointPairs();
  STBoundary st_boundary(point_pairs);
  double test_t = 1.0;
  STBoundary res = st_boundary.ExpandByT(test_t);
  EXPECT_NEAR(res.min_t(), -1.0, kMathEpsilon);
  EXPECT_NEAR(res.min_s(), 20.0, kMathEpsilon);
  EXPECT_NEAR(res.max_t(), 11.0, kMathEpsilon);
  EXPECT_NEAR(res.max_s(), 50.0, kMathEpsilon);
}

} // namespace planning
} // namespace xju