/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/st_graph_decider/st_boundary_mapper.h"

#include <vector>
#include <string>

#include "pnc_point.pb.h"
#include "common/math/math_utils.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/math/path_matcher.h"
#include "pnc_map/reference_line.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace {
constexpr double kDoubleEpsilon = 1.0e-6;
constexpr double kMinSampleDistance = 3.0;
} //namespace

namespace xju {
namespace planning {

StBoundaryMapper::StBoundaryMapper(
    const StGraphDeciderConfig& st_graph_decider_config,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info, 
    Frame* const frame)
    : st_graph_config_(st_graph_decider_config),
      reference_line_info_(reference_line_info), 
      frame_(frame) {
  path_data_ = reference_line_info_->path_data();
  vehicle_config_ = pnc::VehicleConfigProvider::GetConfig();
}

bool StBoundaryMapper::MapObstaclesToSTBoundaries() {
  if (path_data_.TotalLength() < kDoubleEpsilon) {
    AERROR << "The length of planned path is too short.";
    return false;
  }
  if (path_data_.planned_path().size() < 2) {
    AERROR << "Number of path points is too few.";
    return false;
  }
  // ADEBUG << path_data_.DebugString();
  // clear obstacle st_boundary
  PathDecision* path_decision = reference_line_info_->path_decision();
  path_decision->EraseStBoundaries();
  
  double init_s = path_data_.frenet_points().front().s();
  double min_stop_obs_s = std::numeric_limits<double>::max();
  std::string nearest_stop_obs_id;
  std::vector<std::string> stop_obstacle_ids;
  
  for (const auto* obstacle_item_ptr : path_decision->obstacles().Items()) {
    Obstacle* obstacle_ptr = path_decision->Find(obstacle_item_ptr->id());
    CHECK(obstacle_ptr != nullptr);
    if (!obstacle_ptr->HasLongitudinalDecision()) {
      ComputeSTBoundary(obstacle_ptr);
      continue;
    }
    const ObjectDecisionType& decision = obstacle_ptr->longitudinal_decision();
    if (decision.has_stop()) {

      // find closest stop decision obstacle
      stop_obstacle_ids.push_back(obstacle_ptr->id());
      const auto& stop_point = decision.stop().stop_point();
      const auto& match_point = pnc::PathMatcher::MatchToPath(
            path_data_.planned_path(), stop_point.x(), stop_point.y());
      double stop_s = std::fmin(match_point.s(), path_data_.TotalLength());
      
      if (stop_s < min_stop_obs_s) {
        min_stop_obs_s = stop_s;
        nearest_stop_obs_id = obstacle_ptr->id();
      }  
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
        ComputeSTBoundaryWithDecision(decision, obstacle_ptr);
    } else if (decision.has_ignore()) {
      ADEBUG << "obstacle longitudinal_decision is ingore " ;
    } else {
      ADEBUG << "obstacle no mapping for decision " ;
    }
  }
  //generate closest stop obstacle's st_boundary
  if (!stop_obstacle_ids.empty()) {
    for(const auto& obs_id : stop_obstacle_ids) {
      if (!nearest_stop_obs_id.empty() && obs_id == nearest_stop_obs_id) {
        Obstacle* stop_obs_ptr = path_decision->Find(obs_id);
        GenerateStopStBoundary(min_stop_obs_s, stop_obs_ptr);
      } 
    }
  }
  return true;
}

void StBoundaryMapper::ComputeSTBoundary(Obstacle* const obstacle_ptr) {
  std::vector<STPoint> upper_st_points;
  std::vector<STPoint> lower_st_points;
  if (!GetOverlapBoundaryPoints(path_data_.planned_path(), *obstacle_ptr,
                                &upper_st_points, &lower_st_points)) {
    AINFO << "Failed get overlap boundary points";
    return;
  }

  STBoundary st_boundary = STBoundary::CreateInstance(
      lower_st_points, upper_st_points);
  ADEBUG << "st_boundary is empty: " << st_boundary.IsEmpty();
  st_boundary.set_id(obstacle_ptr->id());
  obstacle_ptr->set_st_boundary(st_boundary);
  return;
}

void StBoundaryMapper::ComputeSTBoundaryWithDecision(
    const ObjectDecisionType& decision, 
    Obstacle* const obstacle) const {
  ADEBUG << "ComputeSTBoundaryWithDecision";
  return; 
}

void StBoundaryMapper::GenerateStopStBoundary(
    double lower_s, Obstacle* const obstacle_ptr) {
  std::vector<STPoint> upper_st_points;
  std::vector<STPoint> lower_st_points;
  lower_s = lower_s - vehicle_config_.wheel_base() -
            vehicle_config_.front_overhang_length();
  lower_s = std::max(0.0, lower_s);

  double upper_s = lower_s + obstacle_ptr->bounding_box().length();
  upper_s = std::fmax(upper_s, path_data_.TotalLength());

  lower_st_points.emplace_back(lower_s, 0.0);
  lower_st_points.emplace_back(lower_s, FLAGS_trajectory_time_length);
  upper_st_points.emplace_back(upper_s, 0.0);
  upper_st_points.emplace_back(upper_s, FLAGS_trajectory_time_length);

  STBoundary st_boundary = STBoundary::CreateInstance(
      lower_st_points, upper_st_points);
  st_boundary.set_id(obstacle_ptr->id());
  st_boundary.set_boundary_type(STBoundary::BoundaryType::STOP);
  obstacle_ptr->set_st_boundary(st_boundary);
  ADEBUG << "Generate Stop StBoundary" << ", id: " << obstacle_ptr->id();
}

bool StBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<pnc::PathPoint>& planned_path, const Obstacle& obstacle,
    std::vector<STPoint>* const upper_st_points,
    std::vector<STPoint>* const lower_st_points) {
  double init_s = planned_path.front().s();
  double end_s = planned_path.back().s();
  double distance = 0.0;
  if (obstacle.is_static() || !obstacle.HasTrajectory()) {
    ADEBUG << "obs: " << obstacle.id() << " is static obs";

    const pnc::Box2d& obs_box = obstacle.bounding_box();
    for (double pt_s = init_s; pt_s <= end_s; pt_s += distance) {
      const pnc::PathPoint& path_point = path_data_.GetPathPointByS(pt_s);
      distance = CalculateBoxDistance(path_point, obs_box);
      ADEBUG << "The distance between path_point_s: " << pt_s
             << " and obs id: " << obstacle.id() << " is: " << distance;
      if (distance <= kDoubleEpsilon) {
        double lower_s = pt_s;
        lower_s = std::fmax(0.0, lower_s);
        double upper_s = pt_s + obs_box.length();
        upper_s = std::fmin(upper_s, path_data_.TotalLength());

        lower_st_points->emplace_back(lower_s, 0.0);
        lower_st_points->emplace_back(lower_s, FLAGS_trajectory_time_length);
        upper_st_points->emplace_back(upper_s, 0.0);
        upper_st_points->emplace_back(upper_s, FLAGS_trajectory_time_length);
        break;
      }
      distance = std::fmax(distance, kMinSampleDistance);
    }
  } else {
    double init_s_frenet_point = path_data_.frenet_points().front().s();
    // ADEBUG << "path data init s in frenet points is: " <<
    // init_s_frenet_point;
    double end_s_frenet_point = path_data_.frenet_points().back().s();
    // ADEBUG << "path data end s in frenet points is: " << end_s_frenet_point;
    pnc::Trajectory obs_trajectory = obstacle.trajectory();
    // ADEBUG << "obs_trajectory_point s in reference line is: "
    //        <<obs_trajectory_point.path_point().s();
    pnc::SLPoint obs_trj_sl_point;
    reference_line_info_->reference_line().XYToSL(
        {obs_trajectory.trajectory_point(0).path_point().x(),
         obs_trajectory.trajectory_point(0).path_point().y()},
        &obs_trj_sl_point);
    if (obs_trj_sl_point.s() < init_s_frenet_point ||
        obs_trj_sl_point.s() > end_s_frenet_point) {
      ADEBUG << "obstacle trajectory point is out of range";
      return (lower_st_points->size() > 1 && upper_st_points->size() > 1);;
    }

    for (int i = 0; i < obs_trajectory.trajectory_point_size(); ++i) {
      const auto& obs_trajectory_point = obs_trajectory.trajectory_point(i);
      double traj_point_time = obs_trajectory_point.relative_time();
      if (traj_point_time < -1.0) {
        continue;
      }

      double obs_traj_x = obs_trajectory_point.path_point().x();
      double obs_traj_y = obs_trajectory_point.path_point().y();
      auto match_point = pnc::PathMatcher::MatchToPath(
          path_data_.planned_path(), obs_traj_x, obs_traj_y);
      const pnc::Box2d& obs_box = obstacle.GetBoundingBox(obs_trajectory_point);
      if (!CheckOverlap(match_point, obs_box)) {
        // ADEBUG << "obstacle trajectory point is out of range";
        continue;
      }
      for (double pt_s = init_s; pt_s <= end_s; pt_s += distance) {
        const pnc::PathPoint& path_point = path_data_.GetPathPointByS(pt_s);
        distance = CalculateBoxDistance(path_point, obs_box);
        // ADEBUG << "The distance between path_point_s: " << pt_s
        //        << " and obs id: " << obstacle.id()
        //        << " 's trajectory point: " << i << " is: " << distance;
        if (distance <= kDoubleEpsilon) {
          // ADEBUG << "collision point s:" << pt_s;
          bool find_low = false;
          bool find_up = false;
          double backward_distance = -kMinSampleDistance;
          double forward_distance = vehicle_config_.length() +
                                    vehicle_config_.width() + 
                                    obs_box.length() + obs_box.width();
          double fine_tuning_step_length = 0.1;
          double lower_s = std::fmax(0.0, pt_s + backward_distance);
          double upper_s = pt_s + forward_distance;

          while (lower_s < upper_s) {
            if (find_low && find_up) {
              break;
            }
            if (!find_low) {
              const auto& path_point = path_data_.GetPathPointByS(lower_s);
              if (!CheckOverlap(path_point, obs_box)) {
                lower_s += fine_tuning_step_length;
              } else {
                find_low = true;
                // ADEBUG << "lower path_point: "
                //        << " traj_point_time: " << traj_point_time
                //        << " x: " << path_point.x() << "  y: " << path_point.y()
                //        << "  z: " << path_point.z()
                //        << "  theta: " << path_point.theta()
                //        << "  k: " << path_point.kappa()
                //        << "  dk: " << path_point.dkappa()
                //        << "  ddk: " << path_point.ddkappa()
                //        << "  s: " << path_point.s()
              }
            }
            if (!find_up) {
              const auto& path_point = path_data_.GetPathPointByS(upper_s);
              if (!CheckOverlap(path_point, obs_box)) {
                upper_s -= fine_tuning_step_length;
              } else {
                find_up = true;
                // ADEBUG << "upper path_point: "
                //        << " traj_point_time: " << traj_point_time
                //        << " x: " << path_point.x() << "  y: " << path_point.y()
                //        << "  z: " << path_point.z()
                //        << "  theta: " << path_point.theta()
                //        << "  k: " << path_point.kappa()
                //        << "  dk: " << path_point.dkappa()
                //        << "  ddk: " << path_point.ddkappa()
                //        << "  s: " << path_point.s()
              }
            }
          }
          // lower_s = std::fmin(path_data_.TotalLength(), lower_s);
          upper_s = std::fmin(path_data_.TotalLength(), upper_s);
          upper_s = std::fmax(upper_s, lower_s);
          if (find_low && find_up) {
            lower_st_points->emplace_back(lower_s, traj_point_time);
            upper_st_points->emplace_back(upper_s, traj_point_time);
            // ADEBUG << "traj_point_time: " << traj_point_time  << ", and upper_s - lower_s: " << upper_s - lower_s;
          }
          break;
        }
        distance = std::fmax(distance, kMinSampleDistance);
      }
    }
  }
  return (lower_st_points->size() > 1 && upper_st_points->size() > 1);
}

double StBoundaryMapper::CalculateBoxDistance(
    const pnc::PathPoint& path_point, const pnc::Box2d& obs_box) {

  double l_buffer = st_graph_config_.boundary_buffer_l();
  const pnc::Box2d& car_box = GenerateCarBox(path_point, l_buffer);
  double distance = pnc::Box2d::AccurateDistance(car_box, obs_box);
  return distance;
}

bool StBoundaryMapper::CheckOverlap(const pnc::PathPoint& path_point, 
                                    const pnc::Box2d& obs_box) {
  double l_buffer = st_graph_config_.boundary_buffer_l();

  const pnc::Box2d& car_box = GenerateCarBox(path_point, l_buffer);
  bool is_overlap = obs_box.IsOverlap(car_box);
  return  is_overlap;
}

//calcuate car's box
pnc::Box2d StBoundaryMapper::GenerateCarBox(
    const pnc::PathPoint& path_point, const double l_buffer) {
  double l = vehicle_config_.back_edge_to_center() - 
      vehicle_config_.back_overhang_length();
  double center_x = path_point.x() + cos(path_point.theta()) * l;
  double center_y = path_point.y() + sin(path_point.theta()) * l;

  return pnc::Box2d(center_x, center_y, path_point.theta(), 
                    vehicle_config_.length(), 
                    vehicle_config_.width() + l_buffer);
}

} // namespace planning
} // namespace xju

