/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/tasks/deciders/speed_decider/speed_decider.h"

#include "common/math/polynomial_x_order.h"
#include "common/time/time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "object_decision.pb.h"
#include "planning/common/frame/frame.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/common/speed/st_point.h"
#include "planning/tasks/deciders/speed_decider/trajectory1d_generator.h"
#include "pnc_point.pb.h"

namespace xju {
namespace planning {

SpeedDecider::SpeedDecider(const pnc::TaskConfig& config,
                           const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {
  CHECK(config.has_speed_decider_config());
  speed_decider_config_ = config.speed_decider_config();
}

void SpeedDecider::Init(const pnc::TaskConfig& config) { 
  config_ = config; 
  speed_decider_config_ = config.speed_decider_config();
}

void SpeedDecider::Reset() {
  reference_line_info_ = nullptr;
  frame_ = nullptr;
}

bool SpeedDecider::Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
                           Frame* const frame) {
  if (reference_line_info == nullptr || frame == nullptr) {
    AERROR << "The pointer to reference line info is nullptr";
    return false;
  }
  Task::Process(reference_line_info, frame);
  auto final_poly_ptr = std::make_shared<pnc::PolynomialXOrder>();
  const double time_1 = pnc::Time::NowInSeconds();
  if (!GenerateLonPolynomial(final_poly_ptr)) {
    AERROR << "Generate longitudinal polynomial failed";
    return false;
  }
  const double time_2 = pnc::Time::NowInSeconds();
  ADEBUG << "The cost time for Generate longitudinal polynomial = "
         << (time_2 - time_1) * 1e3 << " ms";
  if (!MakeObjectsDecision(final_poly_ptr)) {
    AERROR << "Make Objects Decision failed";
    return false;
  };
  if (!UpdateSpeedData(final_poly_ptr)) {
    AERROR << "Update Speed Data failed";
    return false;
  }
  DrawDebugInfo();
  return true;
}

bool SpeedDecider::GenerateLonPolynomial(
    std::shared_ptr<pnc::PolynomialXOrder> final_poly_ptr) {
  const pnc::TrajectoryPoint& planning_start_point =
      frame_->planning_start_point();
  std::array<double, 3> init_s{0.0, planning_start_point.v(),
                               planning_start_point.a()};
  const StGraphData& st_graph_data = reference_line_info_->st_graph_data();
  double cruising_speed = reference_line_info_->cruise_speed();
  const auto& vehicle_config = pnc::VehicleConfigProvider::GetConfig();

  const auto& lane_change_status = internal_->planning_status().lane_change_status();
  Trajectory1dGenerator trajectory1d_generator(
      cruising_speed, speed_decider_config_, vehicle_config, init_s,
      reference_line_info_->path_data(), st_graph_data, lane_change_status);

  std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_boundle;
  if (!trajectory1d_generator.GenerateLonTrajectoryBundle(
          &lon_trajectory_boundle)) {
    AERROR << "generate longitudianl trajectory boundle failed.";
    return false;
  }
  ADEBUG << "longitudianl trajectory boundle size is: "
         << lon_trajectory_boundle.size();

  if (!trajectory1d_generator.TrajectoryEvaluate(lon_trajectory_boundle,
                                                 final_poly_ptr)) {
    AERROR << "the cost of all longitudianl trajectories is less than zero";
    return false;
  }
  ADEBUG << "final polynomial end state: s = "
         << final_poly_ptr->end_state().first[0]
         << "m ,v= " << final_poly_ptr->end_state().first[1]
         << "m/s ,t = " << final_poly_ptr->end_state().second 
         << "s, label= " << final_poly_ptr->label();
  return true;
}

bool SpeedDecider::MakeObjectsDecision(
    const std::shared_ptr<pnc::PolynomialXOrder>& final_poly_ptr) {
  if (final_poly_ptr->Coef().empty()) {
    return false;
  }
  PathDecision* path_decision = reference_line_info_->path_decision();
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->id());
    const auto& st_boundary = mutable_obstacle->st_boundary();

    if (st_boundary.IsEmpty() || st_boundary.max_s() < 0.0 ||
        st_boundary.max_t() < 0.0 ||
        st_boundary.min_t() > FLAGS_trajectory_time_length) {
      // st_boundary.set_boundary_type(STBoundary::BoundaryType::IGNORE);
      continue;
    }
    if (st_boundary.boundary_type() == STBoundary::BoundaryType::STOP) {
      continue;
    }
    const SpeedDecider::STLocation& location =
        GetSTLocation(st_boundary, final_poly_ptr);
    switch (location) {
      case SpeedDecider::STLocation::BELOW: {
        ObjectDecisionType follow_decision;
        follow_decision.mutable_follow();
        mutable_obstacle->set_longitudinal_decision(follow_decision);
        break;
      }
      case SpeedDecider::STLocation::ABOVE: {
        ObjectDecisionType overtake_decision;
        overtake_decision.mutable_overtake();
        mutable_obstacle->set_longitudinal_decision(overtake_decision);
        break;
      }
      default: {
        break;
      }
    }
    // ADEBUG << "st_boundary id: " << st_boundary.id()
    //        << " boundary_type is: " << st_boundary.boundary_type();
  }
  return true;
}

SpeedDecider::STLocation SpeedDecider::GetSTLocation(
    const STBoundary& st_boundary,
    const std::shared_ptr<pnc::PolynomialXOrder>& final_poly_ptr) {
  if (st_boundary.IsEmpty()) {
    return BELOW;
  }
  double t_0 = st_boundary.bottom_left_point().t();
  double s_lower = st_boundary.bottom_left_point().s();
  double s_upper = st_boundary.upper_left_point().s();
  double traj_s = final_poly_ptr->Eval(0, t_0);
  return (traj_s > s_upper) ? ABOVE : BELOW;
}

bool SpeedDecider::UpdateSpeedData(
    const std::shared_ptr<pnc::PolynomialXOrder>& final_poly_ptr) {
  if (final_poly_ptr->Coef().empty()) {
    AERROR << "get final polynomial failed";
    return false;
  }
  SpeedData* speed_data = reference_line_info_->mutable_speed_data();
  speed_data->clear();
  const pnc::TrajectoryPoint& planning_start_point =
      frame_->planning_start_point();
  speed_data->AppendSpeedPoint(0.0, 0.0, planning_start_point.v(),
                               planning_start_point.a(),
                               planning_start_point.da());

  double s = 0.0, v = 0.0, a = 0.0, da = 0.0;
  double end_sample_time = final_poly_ptr->end_state().second;
  double t_step = speed_decider_config_.dense_time_resolution();
  for (double t = t_step;
       t <= FLAGS_trajectory_time_length;
       t += t_step) {
    if (t <= end_sample_time ) {
      s = std::max(0.0, final_poly_ptr->Eval(0, t));
      v = std::max(0.0, final_poly_ptr->Eval(1, t));
      a = final_poly_ptr->Eval(2, t);
      da = final_poly_ptr->Eval(3, t);
    } else {
      s += v * t_step;
      a = 0;
      da = 0; 
    }

    speed_data->AppendSpeedPoint(t, s, v, a, da);
  }
  if (speed_data->size() < 2) {
    return false;
  }
  // ADEBUG << speed_data->DebugString();
  return true;
};

void SpeedDecider::DrawDebugInfo() {
  pnc::Debug* debug_info = internal_->mutable_debug_info();
  pnc::Chart* chart_1 = debug_info->add_charts();
  // draw st_boundaries and speed_data s;
  chart_1->set_title("s-t");
  chart_1->mutable_options()->mutable_x()->set_min(0);
  chart_1->mutable_options()->mutable_x()->set_max(9);
  chart_1->mutable_options()->mutable_x()->set_label_string("t(s)");
  const auto& st_graph_data = reference_line_info_->st_graph_data();

  chart_1->mutable_options()->mutable_y()->set_min(-5);
  chart_1->mutable_options()->mutable_y()->set_max(st_graph_data.path_length()*1.1);
  chart_1->mutable_options()->mutable_y()->set_label_string("s(m)");
  const auto& st_boundaries = st_graph_data.st_boundaries();

  for (const auto* st_boundary : st_boundaries) {
    pnc::Polygon* polygon = chart_1->add_polygon();
    polygon->set_label("st " + st_boundary->id());
    (*polygon->mutable_properties())["color"] = "b";
    const std::vector<STPoint>& lower_points = st_boundary->lower_points();
    const std::vector<STPoint>& upper_points = st_boundary->upper_points();
    for (auto it = lower_points.begin(); it != lower_points.end(); ++it) {
      pnc::Point2D* point2d = polygon->add_point();
      point2d->set_x(it->t());
      point2d->set_y(it->s());
    }
    for (auto it = upper_points.rbegin(); it != upper_points.rend(); ++it) {
      pnc::Point2D* point2d = polygon->add_point();
      point2d->set_x(it->t());
      point2d->set_y(it->s());
    }
  }
  pnc::Line* s_line = chart_1->add_line();
  s_line->set_label("lattice");
  for (const auto& speed_point : reference_line_info_->speed_data()) {
    pnc::Point2D* point2d = s_line->add_point();
    point2d->set_x(speed_point.t());
    point2d->set_y(speed_point.s());
  }
  
  pnc::Line* planed_path_length = chart_1->add_line();
  planed_path_length->set_label("path_length");
  (*planed_path_length->mutable_properties())["color"] = "g";
  (*planed_path_length->mutable_properties())["linestyle"] = "dashed";
  for (const auto& speed_point : reference_line_info_->speed_data()) {
    pnc::Point2D* point2d = planed_path_length->add_point();
    point2d->set_x(speed_point.t());
    point2d->set_y(st_graph_data.path_length());
  }

  // //draw speed data velocity
  pnc::Chart* chart_2 = debug_info->add_charts();
  chart_2->set_title("v-t");
  chart_2->mutable_options()->mutable_x()->set_min(0);
  chart_2->mutable_options()->mutable_x()->set_max(9);
  chart_2->mutable_options()->mutable_x()->set_label_string("t(s)");
  chart_2->mutable_options()->mutable_y()->set_label_string("v(m/s)");
  chart_2->mutable_options()->mutable_y()->set_min(-3);
  chart_2->mutable_options()->mutable_y()->set_max(
      reference_line_info_->cruise_speed() * 1.2);
  pnc::Line* v_line = chart_2->add_line();
  v_line->set_label("lattice");
  for (const auto& speed_point : reference_line_info_->speed_data()) {
    pnc::Point2D* point2d = v_line->add_point();
    point2d->set_x(speed_point.t());
    point2d->set_y(speed_point.v());
  }

  // //draw speed data velocity
  pnc::Chart* chart_3 = debug_info->add_charts();
  chart_3->set_title("a-t");
  chart_3->mutable_options()->mutable_x()->set_min(0);
  chart_3->mutable_options()->mutable_x()->set_max(9);
  chart_3->mutable_options()->mutable_x()->set_label_string("t(s)");
  chart_3->mutable_options()->mutable_y()->set_label_string("a(m/s2)");
  chart_3->mutable_options()->mutable_y()->set_min(
      FLAGS_longitudinal_acceleration_lower_bound);
  chart_3->mutable_options()->mutable_y()->set_max(
      FLAGS_longitudinal_acceleration_upper_bound);
  pnc::Line* a_line = chart_3->add_line();
  a_line->set_label("lattice");
  for (const auto& speed_point : reference_line_info_->speed_data()) {
    pnc::Point2D* point2d = a_line->add_point();
    point2d->set_x(speed_point.t());
    point2d->set_y(speed_point.a());
  }
}

}  // namespace planning
}  // namespace xju