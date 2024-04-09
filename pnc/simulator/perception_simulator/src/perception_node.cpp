/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "perception_node.h"

#include "common/file/file.h"
#include "common/gflags/global_gflags.h"
#include "common/math/box2d.h"
#include "common/math/curve_math.h"
#include "common/math/line_segments.h"
#include "common/time/time.h"
#include "common/util/coordinate.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"

namespace xju {
namespace simulator {

void PerceptionNode::InitParam() {
  // load yaml param
  this->declare_parameter<std::string>("gflag_config_file_path", "");
  this->declare_parameter<std::string>("hdmap_config_file", "");
  this->declare_parameter<bool>("publish_em_obs", false);
  std::string gflag_config_file_path;
  this->get_parameter("gflag_config_file_path", gflag_config_file_path);
  this->get_parameter("hdmap_config_file", hdmap_config_file_);
  this->get_parameter("publish_em_obs", publish_em_obs_);

  // init gflag param
  pnc::File::GetGflagConfig(gflag_config_file_path);
}

void PerceptionNode::ObjectTimerCallback() {
  // update object
  UpdateObject();

  // publish object
  PublishObject();

  // publish object marker
  PublishObjectMarker();
}

void PerceptionNode::UpdateObject() {
  // 遍历所有障碍物，如果有障碍物运行时间已经超过life time，就删除这个障碍物
  auto now_time = this->get_clock()->now();
  for (auto it = objects_.begin(); it != objects_.end();) {
    if ((now_time - it->second->start_time).seconds() > it->second->life_time) {
      it = objects_.erase(it);
    } else {
      ++it;
    }
  }

  double delta_t = (now_time - sim_time_).seconds();

  for (const auto &obj : objects_) {
    obj.second->remaining_time =
        obj.second->life_time - (now_time - obj.second->start_time).seconds();

    // update x_ y_ heading
    if (obj.second->object_path_class == ObjectPathClass::LINE) {
      obj.second->x += obj.second->v * std::cos(obj.second->heading) * delta_t;
      obj.second->y += obj.second->v * std::sin(obj.second->heading) * delta_t;
      // traj
      obj.second->trajectory.clear();
      for (double t = 0.0; t <= obj.second->traj_time;
           t += obj.second->traj_time_resolution) {
        obj.second->trajectory.emplace_back(
            obj.second->x + obj.second->v * std::cos(obj.second->heading) * t,
            obj.second->y + obj.second->v * std::sin(obj.second->heading) * t,
            obj.second->heading, obj.second->v, t);
      }
    } else if (obj.second->object_path_class == ObjectPathClass::LANE) {
      // TODO
      // cal current lane
      // update lane id if first update
      if (obj.second->nearest_lane_id < 0) {
        obj.second->nearest_lane_id = hdmap_interface_->GetNearestLaneId(
            obj.second->x, obj.second->y, obj.second->heading,
            obj.second->nearest_lane_id);
      }

      auto points = hdmap_interface_->GetHDMap()
                        ->hdmap_lanes_map[obj.second->nearest_lane_id]
                        .center_line;
      double succ_lane_length = 0.0;
      auto succ_ids =
          hdmap_interface_->GetSuccessorIds(obj.second->nearest_lane_id);
      while (succ_lane_length < obj.second->v * obj.second->traj_time &&
             succ_ids) {
        int id = succ_ids->front();
        const auto &succ_lane = hdmap_interface_->GetLane(id);
        points.insert(points.end(), succ_lane.center_line.begin(),
                      succ_lane.center_line.end());
        succ_lane_length += succ_lane.length;
        succ_ids = hdmap_interface_->GetSuccessorIds(id);
      }

      // cal projection s on lane_center
      pnc::LineSegments line_segs(points);

      double s, l;
      int index;
      line_segs.GetProjection({obj.second->x, obj.second->y}, &s, &l);
      double next_s = s + obj.second->v * delta_t;
      double next_x, next_y;
      line_segs.GetXY(next_s, &next_x, &next_y);
      obj.second->heading = (pnc::Vec2d(next_x, next_y) -
                             pnc::Vec2d(obj.second->x, obj.second->y))
                                .Angle();
      obj.second->x = next_x;
      obj.second->y = next_y;

      // fix static obs direction problem
      if (obj.second->v < 1e-3) {
        double t_next_x, t_next_y;
        line_segs.GetXY(next_s + 2.0, &t_next_x, &t_next_y);
        obj.second->heading = (pnc::Vec2d(t_next_x, t_next_y) -
                               pnc::Vec2d(obj.second->x, obj.second->y))
                                  .Angle();
      }

      // traj
      obj.second->trajectory.clear();
      for (double t = 0.0; t <= obj.second->traj_time;
           t += obj.second->traj_time_resolution) {
        double step_s = next_s + obj.second->v * t;
        if (step_s > line_segs.Length()) {
          break;
        }
        line_segs.GetXY(step_s, &next_x, &next_y, &index);

        double step_heading = 0.0;
        if (index < points.size() - 1) {
          step_heading = (points[index + 1] - points[index]).Angle();
        } else {
          step_heading = (points[index] - points[index - 1]).Angle();
        }
        obj.second->trajectory.emplace_back(next_x, next_y, step_heading,
                                            obj.second->v, t);
      }

    } else {
      // TODO
    }

    // obj lane id
    if (hdmap_interface_) {
      auto id = hdmap_interface_->GetNearestLaneId(obj.second->x, obj.second->y,
                                                   obj.second->heading,
                                                   obj.second->nearest_lane_id);
      obj.second->nearest_lane_id = id;
    }
  }

  // update sim_time
  sim_time_ = now_time;
}

void PerceptionNode::PublishObject() {
  // change local to vehicle
  std::vector<std::shared_ptr<Object>> objs;
  for (const auto &object : objects_) {
    auto obj = std::make_shared<Object>(*(object.second));

    // x y heading
    double vehicle_x = obj->x, vehicle_y = obj->y,
           vehicle_heading = obj->heading;
    pnc::TransformLocalToVehicleCoord(loc_x_, loc_y_, loc_heading_, &vehicle_x,
                                      &vehicle_y, &vehicle_heading);
    obj->x = vehicle_x;
    obj->y = vehicle_y;
    obj->heading = vehicle_heading;

    // record vehicle coridate to debug
    object.second->vehicle_x = vehicle_x;
    object.second->vehicle_y = vehicle_y;
    object.second->vehicle_heading = vehicle_heading;

    // prediction_traj
    for (std::size_t i = 0; i < obj->trajectory.size(); i++) {
      vehicle_x = obj->trajectory[i].x;
      vehicle_y = obj->trajectory[i].y;
      vehicle_heading = obj->trajectory[i].theta;
      pnc::TransformLocalToVehicleCoord(loc_x_, loc_y_, loc_heading_,
                                        &vehicle_x, &vehicle_y,
                                        &vehicle_heading);
      obj->trajectory[i].x = vehicle_x;
      obj->trajectory[i].y = vehicle_y;
      obj->trajectory[i].theta = vehicle_heading;
    }
    objs.push_back(obj);
  }

  // s and kappa
  for (const auto &obj : objs) {
    // s
    if (!(obj->trajectory.empty())) {
      obj->trajectory.front().s = 0.0;
    }
    for (int i = 1; i < obj->trajectory.size(); i++) {
      obj->trajectory[i].s =
          std::hypot(obj->trajectory[i - 1].x - obj->trajectory[i].x,
                     obj->trajectory[i - 1].y - obj->trajectory[i].y);
    }
    // kappa
    for (int i = 1; i < int(obj->trajectory.size()) - 1; i++) {
      pnc::Vec2d p1(obj->trajectory[i - 1].x, obj->trajectory[i - 1].y);
      pnc::Vec2d p2(obj->trajectory[i].x, obj->trajectory[i].y);
      pnc::Vec2d p3(obj->trajectory[i + 1].x, obj->trajectory[i + 1].y);
      double kappa = pnc::CurveMath::ComputeCurvature(
          std::move(p1), std::move(p2), std::move(p3));
      obj->trajectory[i].kappa = std::move(kappa);
    }
    if (obj->trajectory.size() >= 2) {
      obj->trajectory.front().kappa = obj->trajectory[1].kappa;
      obj->trajectory.back().kappa =
          obj->trajectory[obj->trajectory.size() - 2].kappa;
    }
  }

  if (publish_em_obs_) {
    perception_em_msgs::msg::EmObstacles em_obs = GenerateEmObs(objs);
    em_object_pub_->publish(std::move(em_obs));
  } else {
    perception_em_msgs::msg::PredictionObstacles pred_obs =
        GeneratePredObs(objs);
    pred_object_pub_->publish(std::move(pred_obs));
  }
}

perception_em_msgs::msg::PredictionObstacles PerceptionNode::GeneratePredObs(
    const std::vector<std::shared_ptr<Object>> &objs) {
  perception_em_msgs::msg::PredictionObstacles pred_obs;
  pred_obs.header.timestamp_sec = pnc::Time::NowInSeconds();
  static int pred_obs_seq_num = 1;
  pred_obs.header.sequence_num = pred_obs_seq_num++;
  pred_obs.header.module_name = "prediction";
  pred_obs.localize_state.x = loc_x_;
  pred_obs.localize_state.y = loc_y_;
  pred_obs.localize_state.theta = loc_heading_;
  for (const auto &obj : objs) {
    perception_em_msgs::msg::PredictionObstacle po;
    // po.perception_obstacle
    po.perception_obstacle.timestamp = pnc::Time::NowInSeconds();
    po.perception_obstacle.id = obj->id;
    po.perception_obstacle.position.x = obj->x;
    po.perception_obstacle.position.y = obj->y;
    po.perception_obstacle.position.z = 0.0;
    po.perception_obstacle.velocity.x = obj->v * std::cos(obj->heading);
    po.perception_obstacle.velocity.y = obj->v * std::sin(obj->heading);
    po.perception_obstacle.velocity.z = 0.0;
    po.perception_obstacle.acceleration.x = 0.0;
    po.perception_obstacle.acceleration.y = 0.0;
    po.perception_obstacle.acceleration.z = 0.0;
    // po.perception_obstacle.position_covariance
    // po.perception_obstacle.velocity_covariance
    // po.perception_obstacle.acceleration_covariance
    po.perception_obstacle.theta = obj->heading;
    po.perception_obstacle.length = obj->length;
    po.perception_obstacle.width = obj->width;
    po.perception_obstacle.height = obj->height;
    if (obj->object_class == ObjectClass::UNKNOW) {
      po.perception_obstacle.type =
          perception_em_msgs::msg::EmObstacle::UNKNOWN;
      po.perception_obstacle.sub_type =
          perception_em_msgs::msg::EmObstacle::ST_UNKNOWN;
    } else if (obj->object_class == ObjectClass::STATIC) {
      po.perception_obstacle.type =
          perception_em_msgs::msg::EmObstacle::UNKNOWN_UNMOVABLE;
      po.perception_obstacle.sub_type =
          perception_em_msgs::msg::EmObstacle::ST_UNKNOWN_UNMOVABLE;
    } else if (obj->object_class == ObjectClass::PEOPLE) {
      po.perception_obstacle.type =
          perception_em_msgs::msg::EmObstacle::PEDESTRIAN;
      po.perception_obstacle.sub_type =
          perception_em_msgs::msg::EmObstacle::ST_PEDESTRIAN;
    } else if (obj->object_class == ObjectClass::BICYCLE) {
      po.perception_obstacle.type =
          perception_em_msgs::msg::EmObstacle::BICYCLE;
      po.perception_obstacle.sub_type =
          perception_em_msgs::msg::EmObstacle::ST_CYCLIST;
    } else if (obj->object_class == ObjectClass::CAR) {
      po.perception_obstacle.type =
          perception_em_msgs::msg::EmObstacle::VEHICLE;
      po.perception_obstacle.sub_type =
          perception_em_msgs::msg::EmObstacle::ST_CAR;
    } else if (obj->object_class == ObjectClass::TRUCK) {
      po.perception_obstacle.type =
          perception_em_msgs::msg::EmObstacle::VEHICLE;
      po.perception_obstacle.sub_type =
          perception_em_msgs::msg::EmObstacle::ST_TRUCK;
    }
    po.perception_obstacle.lane_ids.push_back(obj->nearest_lane_id);  //?
    pnc::Box2d box(obj->x, obj->y, obj->heading, obj->length, obj->width);
    std::vector<pnc::Vec2d> corners = box.corners();
    for (const auto &corner : corners) {
      common_msgs::msg::Point3D point;
      point.x = corner.x();
      point.y = corner.y();
      point.z = 0.0;
      po.perception_obstacle.polygon_point.push_back(std::move(point));
    }
    // po.perception_obstacle.tracking_time
    // po.perception_obstacle.fusion_state
    // po.perception_obstacle.measurements
    // po.perception_obstacle.trajectory

    // po.prediction_trajectory
    // default 1 trajectory
    perception_em_msgs::msg::PredictionTrajectory pred_traj;
    pred_traj.probability = 1.0;
    for (const auto &traj_point : obj->trajectory) {
      common_msgs::msg::TrajectoryPoint traj_point_msg;
      traj_point_msg.path_point.x = traj_point.x;
      traj_point_msg.path_point.y = traj_point.y;
      traj_point_msg.path_point.z = 0.0;
      traj_point_msg.path_point.kappa = traj_point.kappa;
      traj_point_msg.path_point.dkappa = 0.0;
      traj_point_msg.path_point.ddkappa = 0.0;
      traj_point_msg.path_point.s = traj_point.s;
      traj_point_msg.v = traj_point.v;
      traj_point_msg.a = 0.0;
      traj_point_msg.da = 0.0;
      traj_point_msg.relative_time = traj_point.t;
      pred_traj.trajectory.push_back(traj_point_msg);
    }
    po.prediction_trajectory.push_back(pred_traj);

    // po.intent
    // po.is_static
    if (obj->v < 1e-3) {
      po.is_static = true;
    } else {
      po.is_static = false;
    }
    // po.is_caution
    po.is_caution = false;

    pred_obs.prediction_obstacle.push_back(po);
  }

  return pred_obs;
}

perception_em_msgs::msg::EmObstacles PerceptionNode::GenerateEmObs(
    const std::vector<std::shared_ptr<Object>> &objs) {
  perception_em_msgs::msg::EmObstacles em_obs;
  em_obs.header.timestamp_sec = pnc::Time::NowInSeconds();
  static int em_obs_seq_num = 1;
  em_obs.header.sequence_num = em_obs_seq_num++;
  em_obs.header.module_name = "em_obstacles";
  em_obs.localize_state.x = loc_x_;
  em_obs.localize_state.y = loc_y_;
  em_obs.localize_state.theta = loc_heading_;
  for (const auto &obj : objs) {
    perception_em_msgs::msg::EmObstacle po;
    po.timestamp = pnc::Time::NowInSeconds();
    po.id = obj->id;
    po.position.x = obj->x;
    po.position.y = obj->y;
    po.position.z = 0.0;
    po.velocity.x = obj->v * std::cos(obj->heading);
    po.velocity.y = obj->v * std::sin(obj->heading);
    po.velocity.z = 0.0;
    po.acceleration.x = 0.0;
    po.acceleration.y = 0.0;
    po.acceleration.z = 0.0;
    // po.position_covariance
    // po.velocity_covariance
    // po.acceleration_covariance
    po.theta = obj->heading;
    po.length = obj->length;
    po.width = obj->width;
    po.height = obj->height;
    if (obj->object_class == ObjectClass::UNKNOW) {
      po.type = perception_em_msgs::msg::EmObstacle::UNKNOWN;
      po.sub_type = perception_em_msgs::msg::EmObstacle::ST_UNKNOWN;
    } else if (obj->object_class == ObjectClass::STATIC) {
      po.type = perception_em_msgs::msg::EmObstacle::UNKNOWN_UNMOVABLE;
      po.sub_type = perception_em_msgs::msg::EmObstacle::ST_UNKNOWN_UNMOVABLE;
    } else if (obj->object_class == ObjectClass::PEOPLE) {
      po.type = perception_em_msgs::msg::EmObstacle::PEDESTRIAN;
      po.sub_type = perception_em_msgs::msg::EmObstacle::ST_PEDESTRIAN;
    } else if (obj->object_class == ObjectClass::BICYCLE) {
      po.type = perception_em_msgs::msg::EmObstacle::BICYCLE;
      po.sub_type = perception_em_msgs::msg::EmObstacle::ST_CYCLIST;
    } else if (obj->object_class == ObjectClass::CAR) {
      po.type = perception_em_msgs::msg::EmObstacle::VEHICLE;
      po.sub_type = perception_em_msgs::msg::EmObstacle::ST_CAR;
    } else if (obj->object_class == ObjectClass::TRUCK) {
      po.type = perception_em_msgs::msg::EmObstacle::VEHICLE;
      po.sub_type = perception_em_msgs::msg::EmObstacle::ST_TRUCK;
    }
    po.lane_ids.push_back(obj->nearest_lane_id);  //?
    pnc::Box2d box(obj->x, obj->y, obj->heading, obj->length, obj->width);
    std::vector<pnc::Vec2d> corners = box.corners();
    for (const auto &corner : corners) {
      common_msgs::msg::Point3D point;
      point.x = corner.x();
      point.y = corner.y();
      point.z = 0.0;
      po.polygon_point.push_back(std::move(point));
    }
    // po.tracking_time
    // po.fusion_state
    // po.measurements
    // po.trajectory

    em_obs.em_obstacle.push_back(po);
  }
  return em_obs;
}

void PerceptionNode::PublishObjectMarker() {
  visualization_msgs::msg::MarkerArray marker_array;
  // 有障碍物数量发生变化，要进行一次显示清空
  if (old_obj_size_ != objects_.size()) {
    // clear old marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = 3;
    marker_array.markers.push_back(delete_marker);
    object_marker_pub_->publish(marker_array);
  }
  marker_array.markers.clear();

  // model
  for (const auto &it : objects_) {
    auto obj = it.second;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "local";
    marker.ns = "object_model";
    if (marker_array.markers.empty()) {
      marker.id = 0;
    } else {
      marker.id = marker_array.markers.back().id + 1;
    }
    marker.type = 1;
    marker.action = 0;
    marker.scale.x = obj->length;
    marker.scale.y = obj->width;
    marker.scale.z = obj->height;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.color.r = 0.0;
    marker.color.a = 0.8;
    marker.pose.position.x = obj->x;
    marker.pose.position.y = obj->y;
    marker.pose.position.z = 0.0;
    tf2::Quaternion temp_qua;
    temp_qua.setRPY(0.0, 0.0, obj->heading);
    marker.pose.orientation.x = temp_qua.x();
    marker.pose.orientation.y = temp_qua.y();
    marker.pose.orientation.z = temp_qua.z();
    marker.pose.orientation.w = temp_qua.w();
    marker_array.markers.push_back(std::move(marker));
  }

  // info
  for (const auto &it : objects_) {
    auto obj = it.second;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "local";
    marker.ns = "object_info";
    if (marker_array.markers.empty()) {
      marker.id = 0;
    } else {
      marker.id = marker_array.markers.back().id + 1;
    }

    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.z = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.position.x = obj->x;
    marker.pose.position.y = obj->y;
    marker.pose.position.z = 0.0;
    std::stringstream ss;
    ss << "id:" << obj->id << " type:" << Name(obj->object_class) << "\n"
       << "t:" << obj->remaining_time << " v:" << obj->v << "\n"
       << "lane_id:" << obj->nearest_lane_id
       << " heading:" << obj->vehicle_heading << "\n"
       << "x:" << obj->vehicle_x << " y:" << obj->vehicle_y;
    marker.text = ss.str();
    marker_array.markers.push_back(std::move(marker));
  }

  // trajectory
  if (!publish_em_obs_) {
    for (const auto &it : objects_) {
      auto obj = it.second;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "local";
      marker.ns = "object_trajectory";
      if (marker_array.markers.empty()) {
        marker.id = 0;
      } else {
        marker.id = marker_array.markers.back().id + 1;
      }
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      for (const auto &traj_point : obj->trajectory) {
        geometry_msgs::msg::Point point;
        point.x = traj_point.x;
        point.y = traj_point.y;
        marker.points.emplace_back(std::move(point));
      }
      marker_array.markers.push_back(std::move(marker));
    }
  }
  object_marker_pub_->publish(marker_array);

  old_obj_size_ = objects_.size();
}

void PerceptionNode::ObjectCommandCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  AINFO << "Receive rviz cmd: " << msg->header.frame_id;
  int id = int(msg->pose.covariance[0]);
  // if (msg->header.frame_id == "delete_object") {
  //   if (objects_.find(id) != objects_.end()) {
  //     objects_.erase(id);
  //   }
  // } else if (msg->header.frame_id == "start_simulation") {
  //   start_simulation_flag_ = true;
  //   start_sim_time_ = this->get_clock()->now();
  //   sim_time_ = start_sim_time_;
  //   for (const auto &obj : objects_) {
  //     obj.second->Reset();
  //   }
  // } else if (msg->header.frame_id == "stop_simulation") {
  //   start_simulation_flag_ = false;
  //   for (const auto &obj : objects_) {
  //     obj.second->Reset();
  //   }
  // } else
  if (msg->header.frame_id == "add_object") {
    objects_[id] = std::make_shared<Object>(*msg);
    objects_[id]->start_time = this->get_clock()->now();
  } else {
    // TODO
  }
}

void PerceptionNode::LocCallback(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg) {
  loc_x_ = msg->local_localize_result.position.x;
  loc_y_ = msg->local_localize_result.position.y;
  loc_heading_ = msg->local_localize_result.euler_ypr.z;
}

}  // namespace simulator
}  // namespace xju