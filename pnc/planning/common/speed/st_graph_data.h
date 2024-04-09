/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "planning/common/speed/st_boundary.h"
#include "planning/common/speed/speed_limit.h"
#include "pnc_point.pb.h"

namespace xju {
namespace planning {

class StGraphData {
 public:
  StGraphData() = default;
  ~StGraphData() = default;

  void LoadData(const std::vector<const STBoundary*>& st_boundaries,
                const pnc::TrajectoryPoint& init_point,
                const SpeedLimit& speed_limit,
                const std::vector<std::string>& obstacles_id_set,
                const double cruise_speed,
                const double path_data_length);

  bool is_initialized() const;

  const std::vector<const STBoundary*>& st_boundaries() const;


  const pnc::TrajectoryPoint& init_point() const;

  double cruise_speed() const;

  double path_length() const;

  const std::vector<std::string>& obstacles_id_set() const;

  const SpeedLimit& speed_limit() const ;

 private:
  bool init_ = false;
  std::vector<const STBoundary*> st_boundaries_;
  std::vector<std::string> obstacles_id_set_;
  pnc::TrajectoryPoint init_point_;
  SpeedLimit speed_limit_;
  double cruise_speed_ = 0.0;
  double path_data_length_ = 0.0;
};

} // namespace planning
} // namespace xju
