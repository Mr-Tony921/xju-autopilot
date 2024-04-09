/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/speed/speed_data.h"

#include <cmath>

#include "common/util/point_factory.h"
#include "gtest/gtest.h"

namespace xju {
namespace planning {
  
using xju::pnc::kMathEpsilon;

constexpr int TEST_POINTS_SIZE = 100;
std::vector<pnc::SpeedPoint> test_speed_points;

void GenerateConstantSpeedPoints() {
  test_speed_points.clear();
  double s(0.0);
  double t(0.0);
  double v(10.0);
  double a(0.0);
  double da(0.0);
  for (int i = 0; i <= TEST_POINTS_SIZE; ++i) {
    auto tp_p = pnc::PointFactory::ToSpeedPoint(s, t, v, a, da);
    test_speed_points.push_back(tp_p);

    t += 0.1;
    s = v * t;
  }
}


void GenerateUniformAccelerationPoints() {
  test_speed_points.clear();
  double s(0.0);
  double t(0.0);
  double v(0.0);
  double a(0.0);
  double da(0.0);
  for (int i = 0; i <= TEST_POINTS_SIZE; ++i) {
    auto tp_p = pnc::PointFactory::ToSpeedPoint(s, t, v, a, da);
    test_speed_points.push_back(tp_p);

    t += 0.1;
    a = 1.0;
    da = 0.0;
    v = a * t;
    s = v * v  / (2.0 * a);
    
  }
}

TEST(SpeedDataTest, EvaluateByTime01) {
  GenerateConstantSpeedPoints();
  planning::SpeedData speed_data(test_speed_points);
  pnc::SpeedPoint* speed_point(new pnc::SpeedPoint());
  bool res = speed_data.EvaluateByTime(5.0, speed_point);
  EXPECT_TRUE(res); 
  EXPECT_NEAR(speed_point->s(), 50.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->t(), 5.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->v(), 10.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->a(), 0.0, kMathEpsilon);
}

TEST(SpeedDataTest, EvaluateByTime02) {
  GenerateUniformAccelerationPoints();
  planning::SpeedData speed_data(test_speed_points);
  pnc::SpeedPoint* speed_point(new pnc::SpeedPoint());
  bool res = speed_data.EvaluateByTime(5.0, speed_point);
  EXPECT_TRUE(res); 
  EXPECT_NEAR(speed_point->s(), 12.5, kMathEpsilon);
  EXPECT_NEAR(speed_point->t(), 5.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->v(), 5.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->a(), 1.0, kMathEpsilon);
}

TEST(SpeedDataTest, EvaluateByS01) {
  GenerateConstantSpeedPoints();
  planning::SpeedData speed_data(test_speed_points);
  pnc::SpeedPoint* speed_point(new pnc::SpeedPoint());
  bool res = speed_data.EvaluateByS(10.0, speed_point);
  EXPECT_TRUE(res); 
  EXPECT_NEAR(speed_point->s(), 10.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->t(), 1.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->v(), 10.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->a(), 0.0, kMathEpsilon);
}

TEST(SpeedDataTest, EvaluateByS02) {
  GenerateUniformAccelerationPoints();
  planning::SpeedData speed_data(test_speed_points);
  pnc::SpeedPoint* speed_point(new pnc::SpeedPoint());
  bool res = speed_data.EvaluateByS(12.5, speed_point);
  EXPECT_TRUE(res); 
  EXPECT_NEAR(speed_point->s(), 12.5, kMathEpsilon);
  EXPECT_NEAR(speed_point->t(), 5.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->v(), 5.0, kMathEpsilon);
  EXPECT_NEAR(speed_point->a(), 1.0, kMathEpsilon);
}

TEST(SpeedDataTest, TotalTime) {
  GenerateConstantSpeedPoints();
  planning::SpeedData speed_data(test_speed_points);
  EXPECT_NEAR(speed_data.TotalTime(), 10.0, kMathEpsilon);
}

TEST(SpeedDataTest, TotalLength) {
  GenerateUniformAccelerationPoints();
  planning::SpeedData speed_data(test_speed_points);
  std::cout << "test_speed_points front " << test_speed_points.front().s()
            << ", back " << test_speed_points.back().s() << std::endl;
  EXPECT_NEAR(speed_data.TotalLength(), 50.0, kMathEpsilon);
}

} // namespace planning
} // namespace xju