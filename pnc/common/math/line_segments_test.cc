/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "line_segments.h"

#include <cmath>
#include <vector>

#include "common/math/vec2d.h"
#include "gtest/gtest.h"

namespace xju {
namespace pnc {

class LineSegmentsTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::vector<Vec2d> points;
    for (int i = 0; i < 10; i++) {
      points.emplace_back(i, i);
    }
    line_segs_ = LineSegments(points);
  }

 protected:
  LineSegments line_segs_;
};

TEST_F(LineSegmentsTest, Length) {
  auto length = line_segs_.Length();
  EXPECT_NEAR(length, std::hypot(9, 9), kMathEpsilon);
}

TEST_F(LineSegmentsTest, GetXY) {
  double s = 4.808326;
  double x, y;
  int index;
  auto res = line_segs_.GetXY(s, &x, &y, &index);
  EXPECT_TRUE(res);
  EXPECT_NEAR(x, 3.4, kMathEpsilon);
  EXPECT_NEAR(y, 3.4, kMathEpsilon);
  EXPECT_EQ(index, 3);
}

TEST_F(LineSegmentsTest, Size) {
  int size = line_segs_.GetAccumulatedS().size();
  EXPECT_EQ(size, 10);
}

TEST_F(LineSegmentsTest, GetProjection1) {
  Vec2d v(0.0, 9.0);
  double s, l;
  auto res = line_segs_.GetProjection(v, &s, &l);
  EXPECT_TRUE(res);
  EXPECT_NEAR(s, 4.5 * 1.4142135, kMathEpsilon);
  EXPECT_NEAR(l, 4.5 * 1.4142135, kMathEpsilon);
}

TEST_F(LineSegmentsTest, GetProjection2) {
  Vec2d v(4.0, 0.0);
  double s, l, dis;
  auto res = line_segs_.GetProjection(v, &s, &l, &dis);
  EXPECT_TRUE(res);
  EXPECT_NEAR(s, 2 * 1.4142135, kMathEpsilon);
  EXPECT_NEAR(l, -2 * 1.4142135, kMathEpsilon);
  EXPECT_NEAR(dis, 2 * 1.4142135, kMathEpsilon);
}

}  // namespace pnc
}  // namespace xju