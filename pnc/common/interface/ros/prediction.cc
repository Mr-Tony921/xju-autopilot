/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void PredictionTrajectoryConversion(
    const perception_em_msgs::msg::PredictionTrajectory& msg,
    PredictionTrajectory* const proto_ptr) {
  proto_ptr->set_probability(msg.probability);

  auto proto_trajectory = proto_ptr->mutable_trajectory();
  // proto_trajectory->set_name();
  proto_trajectory->clear_trajectory_point();
  for (const auto& msg_obj : msg.trajectory) {
    auto proto_obj = proto_trajectory->add_trajectory_point();
    TrajectoryPointConversion(msg_obj, proto_obj);
  }
}

void PredictionObstacleConversion(
    const perception_em_msgs::msg::PredictionObstacle& msg,
    PredictionObstacle* const proto_ptr) {
  PerceptionObstacleConversion(msg.perception_obstacle,
                               proto_ptr->mutable_perception_obstacle());
  proto_ptr->clear_prediction_trajectory();                             
  for (const auto& msg_obj : msg.prediction_trajectory) {
    auto proto_obj = proto_ptr->add_prediction_trajectory();
    PredictionTrajectoryConversion(msg_obj, proto_obj);
  }
  proto_ptr->set_is_static(msg.is_static);
  proto_ptr->set_is_caution(msg.is_caution);
}

void PredictionConversion(
    const perception_em_msgs::msg::PredictionObstacles::SharedPtr msg_ptr,
    PredictionObstacles* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());

  proto_ptr->mutable_localization_pose()->set_x(msg_ptr->localize_state.x);
  proto_ptr->mutable_localization_pose()->set_y(msg_ptr->localize_state.y);
  proto_ptr->mutable_localization_pose()->set_theta(msg_ptr->localize_state.theta);
  
  proto_ptr->clear_prediction_obstacle();
  for (const auto& msg_obj : msg_ptr->prediction_obstacle) {
    auto proto_obj = proto_ptr->add_prediction_obstacle();
    PredictionObstacleConversion(msg_obj, proto_obj);
  }
  // proto_ptr->set_error_code();
}

}  // namespace pnc
}  // namespace xju