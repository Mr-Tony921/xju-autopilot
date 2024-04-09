/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/common/speed/st_graph_data.h"
namespace xju {
namespace planning {
void StGraphData::LoadData(const std::vector<const STBoundary*>& st_boundaries,
                           const pnc::TrajectoryPoint& init_point,
                           const SpeedLimit& speed_limit,
                           const std::vector<std::string>& obstacles_id_set,
                           const double cruise_speed,
                           const double path_data_length) {
  init_ = true;
  st_boundaries_ = st_boundaries;
  init_point_ = init_point;
  speed_limit_ = speed_limit;
  obstacles_id_set_ = obstacles_id_set;
  cruise_speed_ = cruise_speed;
  path_data_length_ = path_data_length;
}

  bool StGraphData::is_initialized() const { return init_; }

  const std::vector<const STBoundary*>& StGraphData::st_boundaries() const {
    return st_boundaries_;
  }


  const pnc::TrajectoryPoint& StGraphData::init_point() const { return init_point_; } 

  double StGraphData::cruise_speed() const { return cruise_speed_; }

  double StGraphData::path_length() const { return path_data_length_; }

  const std::vector<std::string>& StGraphData::obstacles_id_set() const {
    return obstacles_id_set_;
  }
  
    const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

} // namespace planning
} // namespace xju
