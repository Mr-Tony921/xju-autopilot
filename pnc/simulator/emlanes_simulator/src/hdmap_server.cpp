
/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "simulator/emlanes_simulator/src/hdmap_server.h"

#include "common/file/file.h"
#include "common/math/math_utils.h"
#include "common/math/line_segments.h"
#include "common/math/curve_math.h"
#include "common/util/coordinate.h"
#include "tf2/utils.h"
#include "common/time/time.h"

namespace xju {
namespace simulator {

void HDMapServerNode::InitParam() {
  // load yaml param
  this->declare_parameter<std::string>("gflag_config_file_path", "");
  this->declare_parameter<std::string>("hdmap_config_file", "");
  std::string gflag_config_file_path;
  this->get_parameter("gflag_config_file_path", gflag_config_file_path);
  this->get_parameter("hdmap_config_file", hdmap_config_file_);
  // init gflag param
  xju::pnc::File::GetGflagConfig(gflag_config_file_path);
}

void HDMapServerNode::LocalizationCallback(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg) {
  loc_received_ = true;
  vehicle_pose_.x = msg->local_localize_result.position.x;
  vehicle_pose_.y = msg->local_localize_result.position.y;
  vehicle_pose_.z = msg->local_localize_result.position.z;
  vehicle_pose_.yaw = msg->local_localize_result.euler_ypr.z;
}

void HDMapServerNode::PublishHDMapMarker() {
  if (!hdmap_interface_) {
    return;
  }
  const auto &hdmap = hdmap_interface_->GetHDMap();
  // hdmap display
  visualization_msgs::msg::MarkerArray marker_array;
  // left_lane_boundary
  for (const auto &it : hdmap->hdmap_lanes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "local";
    marker.ns = "left_boundary";
    if (marker_array.markers.empty()) {
      marker.id = 0;
    } else {
      marker.id = marker_array.markers.back().id + 1;
    }
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    for (const auto &vec_point : it.left_boundary) {
      geometry_msgs::msg::Point point;
      point.x = vec_point.x();
      point.y = vec_point.y();
      marker.points.emplace_back(std::move(point));
    }
    marker_array.markers.push_back(marker);
  }

  // right_lane_boundary
  for (const auto &it : hdmap->hdmap_lanes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "local";
    marker.ns = "right_boundary";
    if (marker_array.markers.empty()) {
      marker.id = 0;
    } else {
      marker.id = marker_array.markers.back().id + 1;
    }
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    for (const auto &vec_point : it.right_boundary) {
      geometry_msgs::msg::Point point;
      point.x = vec_point.x();
      point.y = vec_point.y();
      marker.points.emplace_back(std::move(point));
    }
    marker_array.markers.push_back(marker);
  }

  // // lane_center
  // for (const auto &it : hdmap->hdmap_lanes) {
  //   visualization_msgs::msg::Marker marker;
  //   marker.header.frame_id = "local";
  //   marker.ns = "center_line";
  //   if (marker_array.markers.empty()) {
  //     marker.id = 0;
  //   } else {
  //     marker.id = marker_array.markers.back().id + 1;
  //   }
  //   marker.type = visualization_msgs::msg::Marker::POINTS;
  //   marker.action = visualization_msgs::msg::Marker::ADD;
  //   marker.scale.x = 0.1;
  //   marker.scale.y = 0.1;
  //   marker.color.r = 0.0;
  //   marker.color.g = 1.0;
  //   marker.color.b = 0.0;
  //   marker.color.a = 0.3;
  //   for (const auto &vec_point : it.center_line) {
  //     geometry_msgs::msg::Point point;
  //     point.x = vec_point.x();
  //     point.y = vec_point.y();
  //     marker.points.emplace_back(std::move(point));
  //   }
  //   marker_array.markers.push_back(marker);
  // }

  // std::shared_ptr<pnc::Vec2d> last_point = nullptr;
  // int arrow_num = 0;
  // for (const auto &it : hdmap->hdmap_lanes) {
  //   for (const auto &vec_point : it.center_line) {
  //     arrow_num++;
  //     if (arrow_num % 800 != 0) {
  //       if (!last_point) {
  //         last_point = std::make_shared<pnc::Vec2d>(vec_point);
  //       } else {
  //         *last_point = vec_point;
  //       }
  //       continue;
  //     }

  //     visualization_msgs::msg::Marker marker;
  //     marker.header.frame_id = "local";
  //     marker.ns = "lane_dir";
  //     if (marker_array.markers.empty()) {
  //       marker.id = 0;
  //     } else {
  //       marker.id = marker_array.markers.back().id + 1;
  //     }
  //     marker.type = visualization_msgs::msg::Marker::ARROW;
  //     marker.action = visualization_msgs::msg::Marker::ADD;
  //     marker.scale.x = 6;
  //     marker.scale.y = 2;
  //     marker.scale.z = 0.01;
  //     marker.color.r = 0.0;
  //     marker.color.g = 1.0;
  //     marker.color.b = 0.0;
  //     marker.color.a = 0.2;

  //     marker.pose.position.x = vec_point.x();
  //     marker.pose.position.y = vec_point.y();

  //     if (!last_point) {
  //       last_point = std::make_shared<pnc::Vec2d>(vec_point);
  //     } else {
  //       double theta = (vec_point - *last_point).Angle();
  //       tf2::Quaternion temp_qua;
  //       temp_qua.setRPY(0.0, 0.0, theta);
  //       marker.pose.orientation.x = temp_qua.x();
  //       marker.pose.orientation.y = temp_qua.y();
  //       marker.pose.orientation.z = temp_qua.z();
  //       marker.pose.orientation.w = temp_qua.w();
  //       *last_point = vec_point;
  //     }

  //     marker_array.markers.push_back(marker);
  //   }
  // }

  // // lane_id
  // for (const auto &it : hdmap->hdmap_lanes) {
  //   visualization_msgs::msg::Marker marker;
  //   marker.header.frame_id = "local";
  //   marker.ns = "id";
  //   if (marker_array.markers.empty()) {
  //     marker.id = 0;
  //   } else {
  //     marker.id = marker_array.markers.back().id + 1;
  //   }
  //   marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  //   marker.action = visualization_msgs::msg::Marker::ADD;
  //   marker.scale.z = 0.6;
  //   marker.color.r = 0.0;
  //   marker.color.g = 1.0;
  //   marker.color.b = 1.0;
  //   marker.color.a = 1.0;
  //   marker.pose.position.x = it.center_line.front().x();
  //   marker.pose.position.y = it.center_line.front().y();
  //   marker.pose.position.z = 0.0;
  //   // std::stringstream ss;
  //   // ss << "id:" << std::to_string(obj->id)
  //   //    << " type:" << Name(obj->object_class) << "\n"
  //   //    << "t:" << std::to_string(obj->remaining_time)
  //   //    << " v:" << std::to_string(obj->v);
  //   // marker.text = ss.str();
  //   marker.text = std::to_string(it.id);
  //   marker_array.markers.push_back(marker);
  // }

  hdmap_marker_pub_->publish(marker_array);
}

void HDMapServerNode::EMLaneTimerCallback() {
  if (!loc_received_) return;
  perception_em_msgs::msg::EmLanes em_lane_msg;
  // header (do not use in reference_line_probider.cc, give only basic elements)
  em_lane_msg.header.timestamp_sec = pnc::Time::NowInSeconds();
  em_lane_msg.header.frame_id = "vehicle";
  // em_lane_msg.module_name = "hdmap_server";
  static uint32_t sequence_num = 1;
  em_lane_msg.header.sequence_num = sequence_num++;
  em_lane_msg.localize_state.x = vehicle_pose_.x;
  em_lane_msg.localize_state.y = vehicle_pose_.y;
  em_lane_msg.localize_state.theta = vehicle_pose_.yaw;

  // find rightest lane id
  static int last_ego_lane_id = -1;
  auto current_id = hdmap_interface_->GetNearestLaneId(
      vehicle_pose_.x, vehicle_pose_.y, vehicle_pose_.yaw, last_ego_lane_id);
  last_ego_lane_id = current_id;
  while (true) {
    auto right_id = hdmap_interface_->GetRightId(current_id);
    if (right_id <= 0) break;
    current_id = right_id;
  }

  // lane_mark and lane_info
  uint8_t lane_order = 1; // FIRST_LANE
  int lane_mark_id = 0;
  while (true) {
    auto left_id = hdmap_interface_->GetLeftId(current_id);

    perception_em_msgs::msg::LaneMark lane_mark {};
    lane_mark.type = 1;           // LBR_MARKING_SINGLE_DASHED_LINE
    lane_mark.color = 2;          // COLOR_WHITE
    lane_mark.quality = 2;        // HIGH0
    lane_mark.id = lane_mark_id;  // hdmap_interface_ no defined
    lane_mark.quality_status = 0; // NORMAL
    lane_mark.source_type = 1;    // HDMAP
    lane_mark.position = 0;       // POSITION_UNKNOWN
    lane_mark.pts_vehicle_coord = GetLanePoints(current_id, 0);
    GetCurveFromPts(lane_mark.pts_vehicle_coord, &lane_mark.curve_vehicle_coord);
    lane_mark.linetype_sections.resize(1);
    lane_mark.linetype_sections.front().type = 1; // LBR_MARKING_SINGLE_DASHED_LINE
    lane_mark.linetype_sections.front().start_point = lane_mark.curve_vehicle_coord.start;
    lane_mark.linetype_sections.front().end_point = lane_mark.curve_vehicle_coord.end;
    em_lane_msg.lane_mark.emplace_back(lane_mark);

    perception_em_msgs::msg::LaneInfo lane_info {};
    // lane_info.lane_id = current_id;
    lane_info.lane_id = lane_order; // only for simulation
    lane_info.leftline_id = lane_mark_id + 1;
    lane_info.rightline_id = lane_mark_id;
    lane_info.pts_center = GetLanePoints(current_id, 1);
    GetCurveFromPts(lane_info.pts_center, &lane_info.curve_center);
    lane_info.lane_type = 0;          // LAT_NORMA
    lane_info.max_speed_limit = 28.0; // 100km/h
    lane_info.min_speed_limit = 0.0;
    lane_info.width = 3.5;
    // hdmap_lane_data
    // incoming_lane_ids
    lane_info.restricted_lane_type = 0; // RESTRICTED_LANE_TYPE_NONE
    lane_info.leftline_quality = 2;     // HIGH0
    lane_info.rightline_quality = 2;    // HIGH0
    lane_info.is_lane_ending = 0;       // false
    lane_info.lane_end_length = 9999;
    lane_info.lane_order = lane_order;
    em_lane_msg.lane_info.emplace_back(lane_info);

    if (left_id <= 0) {
      lane_mark.id = lane_mark_id + 1;
      lane_mark.pts_vehicle_coord = GetLanePoints(current_id, 2);
      GetCurveFromPts(lane_mark.pts_vehicle_coord, &lane_mark.curve_vehicle_coord);
      lane_mark.linetype_sections.resize(1);
      lane_mark.linetype_sections.front().type = 1; // LBR_MARKING_SINGLE_DASHED_LINE
      lane_mark.linetype_sections.front().start_point = lane_mark.curve_vehicle_coord.start;
      lane_mark.linetype_sections.front().end_point = lane_mark.curve_vehicle_coord.end;
      em_lane_msg.lane_mark.emplace_back(lane_mark);
      break;
    }
    current_id = left_id;
    ++lane_order;
    ++lane_mark_id;
  }

  // error_code (default is 0)
  em_lane_msg.error_code = 0;

  em_lane_pub_->publish(em_lane_msg);
}

auto HDMapServerNode::GetLanePoints(int current_id, uint8_t line_choise) // right:0 center:1 left:2
    -> std::vector<common_msgs::msg::Point3D> {
  auto insert_points = [](const Lane& lane_source, uint8_t line_choise,
                          std::vector<pnc::Vec2d>* target) {
    switch (line_choise) {
      case 0:
        target->insert(target->end(), lane_source.right_boundary.begin(),
                       lane_source.right_boundary.end());
        break;
      case 1:
        target->insert(target->end(), lane_source.center_line.begin(),
                       lane_source.center_line.end());
        break;
      case 2:
        target->insert(target->end(), lane_source.left_boundary.begin(),
                       lane_source.left_boundary.end());
        break;
      default:
        target->insert(target->end(), lane_source.center_line.begin(),
                       lane_source.center_line.end());
        break;
    }
  };

  const auto& lane = hdmap_interface_->GetLane(current_id);
  std::vector<pnc::Vec2d> points;
  const std::vector<int>* predecessor_ids = hdmap_interface_->GetPredecessorIds(current_id);
  if (predecessor_ids && !predecessor_ids->empty()) {
    int id = predecessor_ids->front();
    const auto &pre_lane = hdmap_interface_->GetLane(id);
    insert_points(pre_lane, line_choise, &points);
  }

  insert_points(lane, line_choise, &points);

  double succ_lane_length = 0.0;
  const std::vector<int>* successor_ids = hdmap_interface_->GetSuccessorIds(current_id);
  while (successor_ids && succ_lane_length < 100 && !successor_ids->empty()) {
    int id = successor_ids->front();
    const auto &succ_lane = hdmap_interface_->GetLane(id);
    insert_points(succ_lane, line_choise, &points);
    succ_lane_length += succ_lane.length;
    successor_ids = hdmap_interface_->GetSuccessorIds(id);
  }

  double s, l;
  pnc::LineSegments line_segments(points);
  line_segments.GetProjection({vehicle_pose_.x, vehicle_pose_.y}, &s, &l);

  double x, y;
  double start_s = s + START_DIST;
  int start_index = 0;
  line_segments.GetXY(start_s, &x, &y, &start_index);

  double end_s = s + END_DIST;
  int end_index = 0;
  line_segments.GetXY(end_s, &x, &y, &end_index);

  std::vector<pnc::Vec2d> new_points(points.begin() + start_index,
                                     points.begin() + end_index + 1);

  // change local to vehicle
  double xx, yy;
  for (auto &p : new_points) {
    xx = p.x();
    yy = p.y();
    pnc::TransformLocalToVehicleCoord(vehicle_pose_.x, vehicle_pose_.y,
                                      vehicle_pose_.yaw, &xx, &yy);
    p.set_x(xx);
    p.set_y(yy);
  }

  std::vector<common_msgs::msg::Point3D> msg_points(new_points.size());
  common_msgs::msg::Point3D p3;
  p3.z = 0;
  size_t idx = 0;
  for (auto const& p : new_points) {
    p3.x = p.x();
    p3.y = p.y();
    msg_points[idx++] = p3;
  }

  return msg_points;
}

void HDMapServerNode::GetCurveFromPts(
    const std::vector<common_msgs::msg::Point3D>& points,
    perception_em_msgs::msg::CubicCurve* const curve) {
  Eigen::Matrix<float, 4, 1> coeff;
  // std::vector<Eigen::Matrix<float, 2, 1>> selected_xy_points;
  std::vector<Eigen::Matrix<float, 2, 1>> xy_points;

  xy_points.resize(points.size());
  for (auto i = 0; i < points.size(); ++i) {
    xy_points[i](0) = points[i].x;
    xy_points[i](1) = points[i].y;
  }

  if (!pnc::CurveMath::PolyFit(xy_points, 3, &coeff)) {
    AWARN << "PolyFit failed!";
    return;
  }

  // if (!pnc::CurveMath::RansacFitting<float>(xy_points, &selected_xy_points, &coeff, 200, 8, 0.1f)) {
  //   AWARN << "RansacFitting failed!";
  //   return;
  // }

  curve->c3 = coeff(3);
  curve->c2 = coeff(2);
  curve->c1 = coeff(1);
  curve->c0 = coeff(0);
  curve->start = points.front().x;
  curve->end = points.back().x;
}

}  // namespace simulator
}  // namespace xju