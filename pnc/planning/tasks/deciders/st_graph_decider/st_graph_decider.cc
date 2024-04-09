/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/st_graph_decider/st_graph_decider.h"

#include <limits>
#include <string>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/time/time.h"
#include "common/util/vehicle_helper.h"
#include "planning/common/obstacle/obstacle.h"
#include "planning/common/speed/st_boundary.h"
#include "planning/common/speed/st_graph_data.h"
#include "planning/common/speed/st_point.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning/tasks/deciders/st_graph_decider/speed_limit_decider.h"
#include "planning/tasks/deciders/st_graph_decider/st_boundary_mapper.h"

namespace xju {
namespace planning {

StGraphDecider::StGraphDecider(
    const pnc::TaskConfig& config, 
    const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {
  CHECK(config.has_st_graph_decider_config());
  st_graph_decider_config_ = config.st_graph_decider_config();
}

void StGraphDecider::Init(const pnc::TaskConfig& config) {
  config_ = config;
}

void StGraphDecider::Reset() {
  reference_line_info_ = nullptr;
  frame_ = nullptr;
  path_data_ = nullptr;
}

bool StGraphDecider::Process(
    std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* const frame) {
  if (reference_line_info == nullptr || frame == nullptr) {
    AERROR << "Pointer of reference line info is nullptr";
    return false;
  }
  Task::Process(reference_line_info, frame);
  const auto& reference_line = reference_line_info_->reference_line();
  const auto& planning_start_point =
      reference_line_info_->planning_start_point();
  // 1. Map obstacles into st graph
  const double time_1 =  pnc::Time::NowInSeconds();
  std::vector<const STBoundary*> boundaries;
  std::vector<std::string> obstacles_id_set;

  StBoundaryMapper boundary_mapper(
      st_graph_decider_config_, reference_line_info, frame);
  if (!boundary_mapper.MapObstaclesToSTBoundaries()) {
    AERROR << "Map obstacles to st boundaries failed";
    return false;
  } 
  const double time_2 =  pnc::Time::NowInSeconds();
  ADEBUG << "The cost time for st_boundayies mapping = " 
         << (time_2 - time_1) * 1e3 << " ms";
         
  PathDecision* path_decision = reference_line_info_->path_decision();
  for (const auto* obstacle_ptr : path_decision->obstacles().Items()) {
    const auto& st_boundary = obstacle_ptr->st_boundary();
    if (!st_boundary.IsEmpty()) {
      ADEBUG << st_boundary.DebugString();
      boundaries.push_back(&(obstacle_ptr->st_boundary()));
      obstacles_id_set.push_back(obstacle_ptr->id());
    }
  }
  ADEBUG << "map obstacle to St_Graph size: " << boundaries.size();

  //2. Create speed limit along path
  SpeedLimitDecider speed_limit_decider(
      st_graph_decider_config_, reference_line_info, frame);
  SpeedLimit speed_limit;
  if(!speed_limit_decider.GetSpeedLimit(&speed_limit)) {
    AERROR << "Failed to get speed limit";
    return false;
  }

  // Load generated st graph data 
  double cruising_speed = reference_line_info_->cruise_speed();
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  double path_data_length = reference_line_info_->path_data().TotalLength();
  st_graph_data->LoadData(
    boundaries, planning_start_point, speed_limit, obstacles_id_set,
    cruising_speed, path_data_length);
  // DrawDebugInfo();
  return true;
}

// double StGraphDecider::GetMinSonSTGraph(
//     const std::vector<const STBoundary*>& st_boundaries) {
//   static constexpr double kDoubleEpsilon = 1e-6;
//   double min_s_on_graph = std::numeric_limits<double>::max();
//   for (const auto& boundary_ptr : st_boundaries) {
//     const double left_bottom_point_s = boundary_ptr->bottom_left_point().s();
//     const double right_bottom_point_s = boundary_ptr->bottom_right_point().s();
//     const double lowest_s = std::fmin(left_bottom_point_s, 
//                                      right_bottom_point_s);
//     if (min_s_on_graph > lowest_s) {
//         min_s_on_graph = lowest_s;
//     }
//   }
//   return min_s_on_graph < kDoubleEpsilon ? 0.0 : min_s_on_graph;
// }

void StGraphDecider::DrawDebugInfo() {
  pnc::Debug* debug_info = internal_->mutable_debug_info();
  pnc::Chart* chart_1 = debug_info->add_charts();
  chart_1->set_title("path_data && obs trajectory");
  const auto& planned_path = reference_line_info_->path_data().planned_path();
  const auto& front_pt = planned_path.front();
  chart_1->mutable_options()->mutable_x()->set_min(-10);
  chart_1->mutable_options()->mutable_x()->set_max(planned_path.back().x() -
                                                   front_pt.x());
  chart_1->mutable_options()->mutable_x()->set_label_string("x(m)");

  pnc::Polygon* adc_polygon = chart_1->add_polygon();
  adc_polygon->set_label("adc");
  const auto& adc_box = pnc::VehicleHelper::CarBox(front_pt);

  for (const auto& corner : adc_box.corners()) {
    pnc::Point2D* point = adc_polygon->add_point();
    point->set_x(corner.x() - front_pt.x());
    point->set_y(corner.y() - front_pt.y());
  }
  double max_y = 0.0;
  pnc::Line* planned_path_line = chart_1->add_line();
  (*planned_path_line->mutable_properties())["color"] = "r";
  (*planned_path_line->mutable_properties())["linestyle"] = "dashed";
  for (const auto& pt : planned_path) {
    pnc::Point2D* point = planned_path_line->add_point();
    point->set_x(pt.x() - front_pt.x());
    point->set_y(pt.y() - front_pt.y());
    if (max_y < std::fabs(pt.y() - front_pt.y())) {
      max_y = std::fabs(pt.y() - front_pt.y());
    }
  }
  chart_1->mutable_options()->mutable_y()->set_min(-std::fmax(10, max_y));
  chart_1->mutable_options()->mutable_y()->set_max(std::fmax(10, max_y));
  chart_1->mutable_options()->mutable_y()->set_label_string("y(m)");

  const auto* path_decision = reference_line_info_->path_decision();

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    pnc::Polygon* obs_polygon = chart_1->add_polygon();
    (*obs_polygon->mutable_properties())["color"] = "b";
    obs_polygon->set_label("obs_ " + obstacle->id());
    const auto& obs_box = obstacle->bounding_box();
    for (const auto& corner : obs_box.corners()) {
      pnc::Point2D* point = obs_polygon->add_point();
      point->set_x(corner.x() - front_pt.x());
      point->set_y(corner.y() - front_pt.y());
    }
    if (obstacle->HasTrajectory()) {
      pnc::Line* obs_traj_line = chart_1->add_line();
      (*obs_traj_line->mutable_properties())["color"] = "b";
      (*obs_traj_line->mutable_properties())["linestyle"] = "dashed";
      const auto& obs_traj = obstacle->trajectory();
      for (size_t i = 0; i < obs_traj.trajectory_point_size(); ++i) {
        pnc::Point2D* point = obs_traj_line->add_point();
        point->set_x(obs_traj.trajectory_point(i).path_point().x() -
                     front_pt.x());
        point->set_y(obs_traj.trajectory_point(i).path_point().y() -
                     front_pt.y());
      }
    }
  }
}

} // namespace planning
} // namespace xju
