/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/polygon2d.h"

#include "gtest/gtest.h"

namespace xju {
namespace pnc {

TEST(Polygon2dTest, ConstructorFromPoint) {
  std::vector<Vec2d> pts;
  pts.emplace_back(2.0, 1.0);
  pts.emplace_back(-2.0, 1.0);
  pts.emplace_back(-2.0, -1.0);
  pts.emplace_back(2.0, -1.0);
  Polygon2d polygon(pts);
  std::vector<Vec2d> corners = polygon.corners();
  EXPECT_EQ(polygon.num_of_corners(), 4);
  EXPECT_NEAR(corners[0].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), -1.0, kMathEpsilon);
}

TEST(Polygon2dTest, ConstructorFroBox) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  Polygon2d polygon(box);
  std::vector<Vec2d> corners = polygon.corners();
  EXPECT_EQ(polygon.num_of_corners(), 4);
  EXPECT_NEAR(corners[0].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(polygon.min_x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(polygon.max_x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(polygon.min_y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(polygon.max_y(), 1.0, kMathEpsilon);
  EXPECT_TRUE(polygon.is_convex());
}

TEST(Polygon2dTest, Hull) {
  std::vector<Vec2d> pts;
  pts.emplace_back(2.0, 1.0);
  pts.emplace_back(-2.0, 1.0);
  pts.emplace_back(-2.0, -1.0);
  pts.emplace_back(-1.0, 0.0);
  Polygon2d polygon(pts);
  EXPECT_FALSE(polygon.is_convex());
  EXPECT_TRUE(polygon.ComputeConvexHull());

  std::vector<Vec2d> corners = polygon.corners();
  EXPECT_EQ(polygon.num_of_corners(), 3);
  EXPECT_NEAR(corners[0].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), 1.0, kMathEpsilon);
}

} // namespace pnc
} // namespace xju