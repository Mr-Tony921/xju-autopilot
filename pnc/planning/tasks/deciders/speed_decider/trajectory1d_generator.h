/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "common/math/polynomial_x_order.h"
#include "planning/common/path/path_data.h"
#include "planning/common/speed/st_boundary.h"
#include "planning/common/speed/st_graph_data.h"
#include "planning/tasks/deciders/speed_decider/feasible_region.h"
#include "planning_config.pb.h"
#include "planning_status.pb.h"
#include "vehicle_config.pb.h"
#include "planning/tasks/deciders/speed_decider/kinematics_model.h"

namespace xju {
namespace planning {

class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator(const double cruising_speed,
                        const SpeedDeciderConfig& speed_decider_config,
                        const pnc::VehicleConfig& vehicle_config,
                        const std::array<double, 3>& init_s,
                        const PathData& path_data,
                        const StGraphData& st_graph_data,
                        const LaneChangeStatus& lane_change_status);

  ~Trajectory1dGenerator() = default;

  bool GenerateLonTrajectoryBundle(
      std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
          lon_trajectory_bundle_ptr);

  bool TrajectoryEvaluate(
      const std::vector<std::shared_ptr<pnc::PolynomialXOrder>>&
          lon_trajectory_bundle,
      std::shared_ptr<pnc::PolynomialXOrder> final_poly_ptr);

 private:
  Trajectory1dGenerator() = default;
  Trajectory1dGenerator(const Trajectory1dGenerator& trajectory1d_generator);

  bool GenerateSpeedProfilesForCruising(
      std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
          lon_trajectory_bundle_ptr);

  void GenerateSpeedProfilesWithDecision(
      const STBoundary& st_boundary,
      std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
          lon_trajectory_bundle_ptr);
  void GenerateSpeedProfilesForStopping(
      const STBoundary& st_boundary,
      std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
          lon_trajectory_bundle_ptr);

  void GenerateSpeedProfilesForFollow(
      const STBoundary& st_boundary,
      std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
          lon_trajectory_bundle_ptr);

  void GenerateSpeedProfilesForOvertake(
      const STBoundary& st_boundary,
      std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
          lon_trajectory_bundle_ptr);

  bool GetStBoundarySurroundingPoints(const STBoundary& st_boundary,
                                      const bool is_lower_bound,
                                      const double t_min_density,
                                      std::vector<STPoint>* const st_points);

  void CalculateLongitudinalBoundary();
  double CalculateTrajectoryCost(
      const std::vector<std::pair<double, double>>& ref_s_dot,
      const std::shared_ptr<pnc::PolynomialXOrder>& s_poly_ptr);
  double MinVelocityInKinematicConstraints(double t, double init_v,
                                           double init_a);
  void set_weights(const SpeedDeciderWeights& weights_config);

  bool IsCollision(const std::shared_ptr<pnc::PolynomialXOrder>& s_poly_ptr);

  //   double LonVelocityCost(
  //       std::shared_ptr<pnc::PolynomialXOrder> s_poly_ptr,
  //       const std::vector<std::pair<double, double>>& ref_s_dot);

  //   double LonCollisionCost(
  //       std::shared_ptr<pnc::PolynomialXOrder> const s_poly_ptr);

  //   double LonAcceCost(
  //       std::shared_ptr<pnc::PolynomialXOrder> s_poly_ptr);

  //   double LonJerkCost(
  //       std::shared_ptr<pnc::PolynomialXOrder> s_poly_ptr);

  std::vector<std::pair<double, double>> ComputeGuideVelocity();

  void GenerateStopPiecewisMotions(
      std::vector<
          std::tuple<double, double, ConstantAccelerationModelT*>>* const
          piecewise_motions_ptr);

  void CalculateQuinticPolyCurve(const double start_s, const double start_v,
                                 const double start_a, const double end_s,
                                 const double end_v, const double end_a,
                                 const double dt,
                                 pnc::PolynomialXOrder* const result);

  void CalculateQuarticPolyCurve(const double start_s, const double start_v,
                                 const double start_a, const double end_v,
                                 const double end_a, const double dt,
                                 pnc::PolynomialXOrder* const result);

 private:
  SpeedDeciderConfig speed_decider_config_;
  pnc::VehicleConfig vehicle_config_;
  std::array<double, 3> init_s_;
  PathData path_data_;
  StGraphData st_graph_data_;
  LaneChangeStatus lane_change_status_;
  double cruising_speed_;
  bool has_stop_point_ = false;
  double stop_point_s_ = std::numeric_limits<double>::max();
  FeasibleRegion feasible_region_;
  // weight
  double weight_guide_velocity_;
  double weight_lon_accelerate_;
  double weight_centripetal_acc_;
  double weight_lon_jerk_;
  double weight_collision_dist_;
  double weight_follow_dist_;
  
  // longitudinal bounday
  double s_lower_bound_ = -1;
  double speed_lower_bound_ = -0.5;
  double acceleration_lower_bound_;
  double acceleration_upper_bound_;
  double jerk_lower_bound_;
  double jerk_upper_bound_;
};

}  // namespace planning
}  // namespace xju
