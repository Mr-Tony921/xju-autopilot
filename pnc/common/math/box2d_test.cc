/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/box2d.h"

#include <cmath>

#include "common/math/math_utils.h"
#include "gtest/gtest.h"

namespace xju {
namespace pnc {

TEST(Box2dTest, Corners) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  std::vector<Vec2d> corners = box.corners();
  EXPECT_NEAR(corners[0].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), -1.0, kMathEpsilon);
}

TEST(Box2dTest, RotateFromCenter) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  box.RotateFromCenter(M_PI_2);
  std::vector<Vec2d> corners = box.corners();
  EXPECT_NEAR(corners[0].x(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), 2.0, kMathEpsilon);
}

TEST(Box2dTest, Shift) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  box.Shift(3.0, 4.0);
  std::vector<Vec2d> corners = box.corners();
  EXPECT_NEAR(corners[0].x(), 5.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 5.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 5.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 5.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), 3.0, kMathEpsilon);
}

TEST(Box2dTest, Extend) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  box.Extend(2.0, 4.0);
  std::vector<Vec2d> corners = box.corners();
  EXPECT_NEAR(corners[0].x(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -3.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -3.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -3.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), -3.0, kMathEpsilon);
}

TEST(Box2dTest, LongitudinalExtend) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  box.LongitudinalExtend(2.0);
  std::vector<Vec2d> corners = box.corners();
  EXPECT_NEAR(corners[0].x(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -3.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 1.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -3.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), -1.0, kMathEpsilon);
}

TEST(Box2dTest, LateralExtend) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  box.LateralExtend(4.0);
  std::vector<Vec2d> corners = box.corners();
  EXPECT_NEAR(corners[0].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[0].y(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[1].y(), 3.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(corners[2].y(), -3.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(corners[3].y(), -3.0, kMathEpsilon);
}

TEST(Box2dTest, GetFunciton) {
  Box2d box(0.0, 0.0, 0.0, 4.0, 2.0);
  EXPECT_NEAR(box.center_x(), 0.0, kMathEpsilon);
  EXPECT_NEAR(box.center_y(), 0.0, kMathEpsilon);
  EXPECT_NEAR(box.length(), 4.0, kMathEpsilon);
  EXPECT_NEAR(box.width(), 2.0, kMathEpsilon);
  EXPECT_NEAR(box.heading(), 0.0, kMathEpsilon);
  EXPECT_NEAR(box.half_length(), 2.0, kMathEpsilon);
  EXPECT_NEAR(box.half_width(), 1.0, kMathEpsilon);
  EXPECT_NEAR(box.sin_heading(), 0.0, kMathEpsilon);
  EXPECT_NEAR(box.cos_heading(), 1.0, kMathEpsilon);
  EXPECT_NEAR(box.min_x(), -2.0, kMathEpsilon);
  EXPECT_NEAR(box.max_x(), 2.0, kMathEpsilon);
  EXPECT_NEAR(box.min_y(), -1.0, kMathEpsilon);
  EXPECT_NEAR(box.max_y(), 1.0, kMathEpsilon);
}

TEST(Box2dTest, IsOverlap) {
  Box2d box1(0.0, 0.0, 0.0, 4.0, 2.0);
  Box2d box2(0.0, 5.0, M_PI_2, 2.0, 2.0);
  Box2d box3(3.0, 1.0, M_PI_2, 2.0, 2.0);
  EXPECT_FALSE(box1.IsOverlap(box2));
  EXPECT_TRUE(box1.IsOverlap(box3));
}

TEST(Box2dTest, AccurateDistance) {
  Box2d box1(0.0, 0.0, 0.0, 4.0, 2.0);
  Box2d box2(0.0, 5.0, M_PI_2, 2.0, 2.0);
  Box2d box3(3.0, 1.0, M_PI_2, 2.0, 2.0);
  Box2d box4(4.0, 3.0, M_PI_4, 2.0 * std::sqrt(2), 2.0 * std::sqrt(2));
  Box2d box5(4.0, -3.0, M_PI_4, 2.0 * std::sqrt(2), 2.0 * std::sqrt(2));
  
  EXPECT_NEAR(Box2d::AccurateDistance(box1, box2), 3.0, kMathEpsilon);
  EXPECT_NEAR(Box2d::AccurateDistance(box1, box3), 0.0, kMathEpsilon);
  EXPECT_NEAR(Box2d::AccurateDistance(box4, box5), 2.0, kMathEpsilon);
}

} // namespace pnc
} // namespace xju