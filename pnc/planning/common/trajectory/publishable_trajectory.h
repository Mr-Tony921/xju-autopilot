/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning.pb.h"

namespace xju {
namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;
  ~PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);

  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const {
    return header_time_;
  }

  void set_header_time(const double header_time) {
    header_time_ = header_time;
  }

  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb);

 private:
  double header_time_ = 0.0;
};

} // namespace planning
} // namespace xju
