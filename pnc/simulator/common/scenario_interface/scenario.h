/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
// #include <unordered_map>
#include <vector>

#include "common/math/vec2d.h"

namespace xju {
namespace simulator {

struct Lanelet {
  int id;
  std::vector<pnc::Vec2d> left_boundary;
  std::vector<pnc::Vec2d> right_boundary;
  int left_id = -1;
  int right_id = -1;
  std::vector<int> successor_ids;
  std::vector<int> predecessor_ids;
  std::string lanelet_type;
  int traffic_sign_id = -1;
  int traffic_light_id = -1;
};

struct TrafficSign {
  int id;
  // TODO
};

struct TrafficLight {
  int id;
  // TODO
};

struct Intersection {
  int id;
  struct Incoming {
    int incoming_lane_id;
    std::vector<int> succ_stright_lane_ids;
    std::vector<int> succ_left_lane_ids;
    std::vector<int> succ_right_lane_ids;
  };
  std::vector<Incoming> incomings;
};

struct StaticObstacle {
  int id;
  std::string type;
  double length;
  double width;
};

struct DynamicObstacle {
  int id;
  std::string type;
  double length;
  double width;
  struct Trajectory {
    double t;
    double v;
    double a;
    double x;
    double y;
    double theta;
  };
  std::vector<Trajectory> traj;
};

struct PlanningProblem {
  double init_x = 0;
  double init_y = 0;
  double init_heading = 0;
  double init_v = 0;
  void Reset() {
    init_x = 0;
    init_y = 0;
    init_heading = 0;
    init_v = 0;
  }
};

struct Scenario {
  std::vector<Lanelet> lanelets;
  std::vector<TrafficSign> traffic_signs;
  std::vector<TrafficLight> traffic_lights;
  std::vector<Intersection> intersections;
  std::vector<DynamicObstacle> dynamic_obstacles;
  PlanningProblem planning_problem;

  void Reset() {
    lanelets.clear();
    traffic_signs.clear();
    traffic_lights.clear();
    intersections.clear();
    dynamic_obstacles.clear();
    planning_problem.Reset();
  }
};

}  // namespace simulator
}  // namespace xju