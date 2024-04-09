/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "reference_line.h"

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "gtest/gtest.h"
#include "pnc_point.pb.h"
#include "sl_boundary.pb.h"

namespace xju {
namespace pnc_map {

class ReferenceLineTest : public ::testing::Test {
 public:
  virtual void SetUp() { refline_ = GenerateRefline(); }

  ReferenceLine GenerateRefline() {
    ReferenceLine refline;
    refline.set_id("1");
    refline.set_left_lane_id("");
    refline.set_right_lane_id("");
    refline.set_lane_width(3.5);  // m
    refline.set_left_road_width(10.0);
    refline.set_right_road_width(10.0);
    refline.set_min_speed_limit(0.0);   // m/s
    refline.set_max_speed_limit(28.0);  // m/s  (100km/h)
    refline.set_left_lane_marking_type(
        pnc::LaneMarkingType::SINGLE_WHITE_DASHED_LINE);
    refline.set_right_lane_marking_type(
        pnc::LaneMarkingType::SINGLE_WHITE_DASHED_LINE);
    refline.set_accommodation_lane_type(pnc::AccommodationLaneType::CAR);
    refline.set_lane_order(pnc::LaneOrder(1));
    refline.set_is_emergency_lane(false);
    refline.set_is_death_lane(false);
    refline.set_is_recommended_lane(false);

    std::vector<pnc::PathPoint> path_points;
    pnc::PathPoint path_point;
    path_point.set_x(0.0);
    path_point.set_y(0.0);
    path_point.set_z(0.0);
    path_point.set_theta(0.0);
    path_point.set_kappa(0.0);
    path_point.set_dkappa(0.0);
    path_point.set_ddkappa(0.0);
    path_point.set_s(0.0);
    for (double i = 0.0; i < 10.0; i += 1.0) {
      auto point = path_point;
      point.set_x(i);
      double s = 0.0;
      if (i > 0) {
        s = path_points.back().s() +
            std::hypot(path_points.back().x() - point.x(),
                       path_points.back().y() - point.y());
      }
      point.set_s(s);
      path_points.push_back(point);
    }
    refline.set_path_points(path_points);

    return refline;
  }

 protected:
  ReferenceLine refline_;
};

TEST_F(ReferenceLineTest, simple_set_function_test) {
  EXPECT_EQ(refline_.id(), "1");
  EXPECT_EQ(refline_.left_lane_id(), "");
  EXPECT_EQ(refline_.right_lane_id(), "");
  EXPECT_FALSE(refline_.is_emergency_lane());
  EXPECT_FALSE(refline_.is_death_lane());
  EXPECT_FALSE(refline_.is_recommended_lane());
  EXPECT_NEAR(refline_.length(),9.0,pnc::kMathEpsilon);
  EXPECT_EQ(refline_.lane_order(),pnc::LaneOrder(1));
  EXPECT_NEAR(refline_.max_speed_limit(),28.0,pnc::kMathEpsilon);
  EXPECT_NEAR(refline_.min_speed_limit(),0.0,pnc::kMathEpsilon);
}

TEST_F(ReferenceLineTest, XYToSL) {
  pnc::Vec2d v(3.0, 1.5);
  pnc::SLPoint sl_point;
  auto res = refline_.XYToSL(v, &sl_point);
  EXPECT_TRUE(res);
  EXPECT_NEAR(sl_point.s(), 3.0, pnc::kMathEpsilon);
  EXPECT_NEAR(sl_point.l(), 1.5, pnc::kMathEpsilon);
}

TEST_F(ReferenceLineTest, SLToXY) {
  pnc::SLPoint sl_point;
  sl_point.set_s(3.0);
  sl_point.set_l(-3.0);
  pnc::Vec2d v;
  auto res = refline_.SLToXY(sl_point, &v);
  EXPECT_TRUE(res);
  EXPECT_NEAR(v.x(), 3.0, pnc::kMathEpsilon);
  EXPECT_NEAR(v.y(), -3.0, pnc::kMathEpsilon);
}

TEST_F(ReferenceLineTest, GetReferencePoint) {
  double s = 1.5;
  auto path_point = refline_.GetReferencePoint(s);
  EXPECT_NEAR(path_point.x(), 1.5, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.y(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.z(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.theta(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.kappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.dkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.ddkappa(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(path_point.s(), 1.5, pnc::kMathEpsilon);
}

TEST_F(ReferenceLineTest, GetSLBoundary) {
  pnc::Box2d box({4.0, 1.0}, 0.0, 3.0, 2.0);
  planning::SLBoundary sl_boundary;
  auto res = refline_.GetSLBoundary(box, sl_boundary);
  EXPECT_TRUE(res);
  EXPECT_NEAR(sl_boundary.start_s(), 2.5, pnc::kMathEpsilon);
  EXPECT_NEAR(sl_boundary.end_s(), 5.5, pnc::kMathEpsilon);
  EXPECT_NEAR(sl_boundary.start_l(), 0.0, pnc::kMathEpsilon);
  EXPECT_NEAR(sl_boundary.end_l(), 2.0, pnc::kMathEpsilon);
}

TEST_F(ReferenceLineTest, IsOnLane) {
  pnc::Vec2d v(5.0, -1.0);
  auto res = refline_.IsOnLane(v);
  EXPECT_TRUE(res);
}

TEST_F(ReferenceLineTest, IsOnRoad) {
  pnc::Vec2d v(6, -34);
  auto res = refline_.IsOnRoad(v);
  EXPECT_FALSE(res);
}

}  // namespace pnc_map
}  // namespace xju