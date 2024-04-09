/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "common/math/math_utils.h"

#include <cmath>

#include "gtest/gtest.h"
#include "pnc_point.pb.h"

namespace xju {
namespace pnc {

pnc::PathPoint GeneratePathPoint(const double x, const double y) {
  pnc::PathPoint path_point;
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_z(0.0);
  path_point.set_theta(M_PI_2);
  path_point.set_kappa(0.0);
  path_point.set_dkappa(0.0);
  path_point.set_ddkappa(0.0);
  path_point.set_s(0.0);
  return path_point;
}

pnc::TrajectoryPoint GenerateTrajPoint(const double x, const double y) {
    pnc::TrajectoryPoint tp;
    auto path_point = tp.mutable_path_point();
    path_point->set_x(x);
    path_point->set_y(y);
    path_point->set_z(0.0);
    path_point->set_theta(M_PI_2);
    path_point->set_kappa(0.0);
    path_point->set_dkappa(0.0);
    path_point->set_ddkappa(0.0);
    path_point->set_s(0.0);

    tp.set_v(0.0);
    tp.set_a(0.0);
    tp.set_da(0.0);
    tp.set_relative_time(0.0);
    return tp;
}

bool PathPointEqual(const pnc::PathPoint& pt_1, const pnc::PathPoint& pt_2) {
    return (pt_1.x() == pt_2.x()) && (pt_1.y() == pt_2.y());
}

TEST(MathUtilsTest, NormalizeAngle) {
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5));
  EXPECT_DOUBLE_EQ(1.5 - M_PI, NormalizeAngle(1.5 + M_PI));
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5 + M_PI * 2));
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5 - M_PI * 2));
  EXPECT_DOUBLE_EQ(-1.5, NormalizeAngle(-1.5));
  EXPECT_DOUBLE_EQ(-9.0 + M_PI * 2, NormalizeAngle(-9.0));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(-M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(-M_PI * 3));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(M_PI * 3));
  EXPECT_DOUBLE_EQ(0.0, NormalizeAngle(M_PI * 4));
}

TEST(MathUtilsTest, AngleDiff) {
  EXPECT_DOUBLE_EQ(0.0, AngleDiff(1.5, 1.5));
  EXPECT_DOUBLE_EQ(-M_PI, AngleDiff(M_PI, 0.0));
  EXPECT_DOUBLE_EQ(-M_PI, AngleDiff(0.0, M_PI));
}

TEST(MathUtilsTest, ToRadian) {
  EXPECT_DOUBLE_EQ(0.0, ToRadian(0.0));
  EXPECT_DOUBLE_EQ(-M_PI, ToRadian(-180.0));
  EXPECT_DOUBLE_EQ(-M_PI, ToRadian(180.0));
  EXPECT_DOUBLE_EQ(M_PI / 2, ToRadian(90.0));
  EXPECT_DOUBLE_EQ(-M_PI / 2, ToRadian(270.0));
}

TEST(MathUtilsTest, CrossProd) {
  EXPECT_NEAR(CrossProd(Vec2d{0, 0}, Vec2d{0, 1}, Vec2d{1, 0}), -1.0, 1e-5);
  EXPECT_NEAR(CrossProd(Vec2d{0, 0}, Vec2d{1, 0}, Vec2d{0, 1}), 1.0, 1e-5);
  EXPECT_NEAR(CrossProd(Vec2d{0, 1}, Vec2d{0, 0}, Vec2d{1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(CrossProd(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{5, 6}), 0.0, 1e-5);
  EXPECT_NEAR(CrossProd(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{6, 5}), -4.0, 1e-5);
  EXPECT_NEAR(CrossProd(Vec2d{2, 2}, Vec2d{7, 5}, Vec2d{3, 4}), 7.0, 1e-5);
  EXPECT_NEAR(CrossProd(GeneratePathPoint(2, 2), 
                        GeneratePathPoint(7, 5), 
                        GeneratePathPoint(3, 4)), 7.0, 1e-5);

}

TEST(MathUtilsTest, InnerProd) {
  EXPECT_NEAR(InnerProd(Vec2d{0, 0}, Vec2d{0, 1}, Vec2d{1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(InnerProd(Vec2d{0, 0}, Vec2d{1, 0}, Vec2d{0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(InnerProd(Vec2d{0, 1}, Vec2d{0, 0}, Vec2d{1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(InnerProd(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{5, 6}), 16.0, 1e-5);
  EXPECT_NEAR(InnerProd(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{6, 5}), 16.0, 1e-5);
  EXPECT_NEAR(InnerProd(Vec2d{2, 2}, Vec2d{7, 5}, Vec2d{3, 4}), 11.0, 1e-5);
  EXPECT_NEAR(InnerProd(Vec2d{2, 2}, Vec2d{0, 0}, Vec2d{3, 4}), -6.0, 1e-5);
  EXPECT_NEAR(InnerProd(GeneratePathPoint(2, 2), 
                        GeneratePathPoint(0, 0), 
                        GeneratePathPoint(3, 4)), -6.0, 1e-5);

}

TEST(MathUtilsTest, Square) {
  EXPECT_DOUBLE_EQ(121.0, Square(11.0));
  EXPECT_FLOAT_EQ(144.0f, Square(-12.0f));
  EXPECT_EQ(169, Square(-13));
  EXPECT_EQ(2147395600, Square(46340));
  EXPECT_EQ(-2147479015, Square(46341));  // Overflow!
}

TEST(MathUtilsTest, QuaternionToHeading) {

}

TEST(MathUtilsTest, Distance) {
  EXPECT_NEAR(Distance(GeneratePathPoint(0.0, 1.0), 
                       GeneratePathPoint(0.0, 0.0)), 1.0, 1e-5);
  EXPECT_NEAR(Distance(GeneratePathPoint(0.0, -1.0), 
                       GeneratePathPoint(0.0, 0.0)), 1.0, 1e-5);
                       
  EXPECT_NEAR(Distance(GeneratePathPoint(0.0, 0.0), 
                       GeneratePathPoint(3.0, 4.0)), 5.0, 1e-5);  


  EXPECT_NEAR(Distance(GenerateTrajPoint(0.0, 1.0), 
                       GenerateTrajPoint(0.0, 0.0)), 1.0, 1e-5);
  EXPECT_NEAR(Distance(GenerateTrajPoint(0.0, -1.0), 
                       GenerateTrajPoint(0.0, 0.0)), 1.0, 1e-5);
}


// TEST(MathUtilsTest, ProjectPoint) {
//   pnc::PathPoint proj_point;
//   bool ret_1 = ProjectPoint(GeneratePathPoint(0.0, 0.0), 
//                             GeneratePathPoint(2.0, 0.0), 1.0, 1.0,
//                             &proj_point);
//   EXPECT_TRUE(ret_1 && PathPointEqual(proj_point, GeneratePathPoint(1.0, 0.0)));

//   bool ret_2 = ProjectPoint(GeneratePathPoint(2.0, 0.0), 
//                             GeneratePathPoint(2.0, 0.0), 1.0, 1.0,
//                             &proj_point);
//   EXPECT_TRUE(!ret_2);
//   bool ret_3 = ProjectPoint(GeneratePathPoint(1.0, 1.0), 
//                             GeneratePathPoint(2.0, 1.0), 3.0, 3.0,
//                             &proj_point);
//   EXPECT_TRUE(ret_3 && PathPointEqual(proj_point, GeneratePathPoint(3.0, 1.0)));

//   bool ret_4 = ProjectPoint(GeneratePathPoint(0.0, 0.0), 
//                             GeneratePathPoint(1.0, 0.0), -1.0, 1.0,
//                             &proj_point);
//   EXPECT_TRUE(ret_4 && PathPointEqual(proj_point, GeneratePathPoint(-1.0, 0.0)));

//   bool ret_5 = ProjectPoint(GeneratePathPoint(0.0, 0.0), 
//                             GeneratePathPoint(1.0, 0.0), -1.0, -1.0,
//                             &proj_point);
//   EXPECT_TRUE(ret_4 && PathPointEqual(proj_point, GeneratePathPoint(-1.0, 0.0)));

// }
} // namespace pnc    
} // namespace xju


