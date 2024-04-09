/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void EmLanesConversion(
    const perception_em_msgs::msg::EmLanes::SharedPtr msg_ptr, 
    EmLanes* proto_ptr) {
  // header
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());

  // lane_mark
  proto_ptr->mutable_lane_mark()->Clear();
  for (auto const& msg_lane_mark : msg_ptr->lane_mark) {
    auto* proto_lane_mark = proto_ptr->add_lane_mark();
    proto_lane_mark->set_id(msg_lane_mark.id);
    proto_lane_mark->set_type(static_cast<LaneMarkingType>(msg_lane_mark.type));
    proto_lane_mark->set_color(static_cast<LaneMarkingColor>(msg_lane_mark.color));
    proto_lane_mark->set_quality(static_cast<LaneMarkingQuality>(msg_lane_mark.quality));
    proto_lane_mark->mutable_curve_vehicle_coord()->set_c0(msg_lane_mark.curve_vehicle_coord.c0);
    proto_lane_mark->mutable_curve_vehicle_coord()->set_c1(msg_lane_mark.curve_vehicle_coord.c1);
    proto_lane_mark->mutable_curve_vehicle_coord()->set_c2(msg_lane_mark.curve_vehicle_coord.c2);
    proto_lane_mark->mutable_curve_vehicle_coord()->set_c3(msg_lane_mark.curve_vehicle_coord.c3);
    proto_lane_mark->mutable_curve_vehicle_coord()->set_start(msg_lane_mark.curve_vehicle_coord.start);
    proto_lane_mark->mutable_curve_vehicle_coord()->set_end(msg_lane_mark.curve_vehicle_coord.end);
    proto_lane_mark->clear_pts_vehicle_coord();
    for (auto const& p : msg_lane_mark.pts_vehicle_coord) {
      auto *pp = proto_lane_mark->add_pts_vehicle_coord();
      Point3DConversion(p, pp);
    }
    proto_lane_mark->set_quality_status(
        static_cast<LaneMarkingQualityStatus>(msg_lane_mark.quality_status));
    proto_lane_mark->set_source_type(
        static_cast<LaneMarkingSourceType>(msg_lane_mark.source_type));
    proto_lane_mark->clear_linetype_sections();
    for (auto const& s : msg_lane_mark.linetype_sections) {
      auto* ss = proto_lane_mark->add_linetype_sections();
      ss->set_type(static_cast<LaneMarkingType>(s.type));
      ss->set_start_point(s.start_point);
      ss->set_end_point(s.end_point);
    }
  }

  // lane_info
  proto_ptr->mutable_lane_info()->Clear();
  for (auto const& msg_lane_info : msg_ptr->lane_info) {
    auto* proto_lane_info = proto_ptr->add_lane_info();
    proto_lane_info->set_lane_id(msg_lane_info.lane_id);
    proto_lane_info->set_leftline_id(msg_lane_info.leftline_id);
    proto_lane_info->set_rightline_id(msg_lane_info.rightline_id);

    proto_lane_info->mutable_curve_center()->set_c0(msg_lane_info.curve_center.c0);
    proto_lane_info->mutable_curve_center()->set_c1(msg_lane_info.curve_center.c1);
    proto_lane_info->mutable_curve_center()->set_c2(msg_lane_info.curve_center.c2);
    proto_lane_info->mutable_curve_center()->set_c3(msg_lane_info.curve_center.c3);
    proto_lane_info->mutable_curve_center()->set_start(msg_lane_info.curve_center.start);
    proto_lane_info->mutable_curve_center()->set_end(msg_lane_info.curve_center.end);
    proto_lane_info->clear_pts_center();
    for (auto const& p : msg_lane_info.pts_center) {
      auto *pp = proto_lane_info->add_pts_center();
      Point3DConversion(p, pp);
    }

    proto_lane_info->set_lane_type(static_cast<LaneType>(msg_lane_info.lane_type));
    proto_lane_info->set_max_speed_limit(msg_lane_info.max_speed_limit);
    proto_lane_info->set_min_speed_limit(msg_lane_info.min_speed_limit);
    proto_lane_info->set_width(msg_lane_info.width);
    proto_lane_info->clear_hdmap_lane_data();
    for (auto const& hd : msg_lane_info.hdmap_lane_data) {
      auto* phd = proto_lane_info->add_hdmap_lane_data();
      phd->set_lane_id(hd.lane_id);
      phd->set_link_id(hd.link_id);
      auto *lphd = phd->mutable_left_linemarking();
      lphd->set_lane_marking_id(hd.left_linemarking.lane_marking_id);
      lphd->clear_pts_vehicle_coord();
      for (auto const& p : hd.left_linemarking.pts_vehicle_coord) {
        auto *pp = lphd->add_pts_vehicle_coord();
        Point3DConversion(p, pp);
      }
      lphd->set_line_type(static_cast<LaneMarkingType>(hd.left_linemarking.line_type));
      auto *rphd = phd->mutable_right_linemarking();
      rphd->set_lane_marking_id(hd.right_linemarking.lane_marking_id);
      rphd->clear_pts_vehicle_coord();
      for (auto const& p : hd.right_linemarking.pts_vehicle_coord) {
        auto *pp = rphd->add_pts_vehicle_coord();
        Point3DConversion(p, pp);
      }
      rphd->set_line_type(static_cast<LaneMarkingType>(hd.right_linemarking.line_type));
      phd->set_max_speed_limit(hd.max_speed_limit);
      phd->set_min_speed_limit(hd.min_speed_limit);
      phd->clear_curvature_set();
      for (auto const& cs : hd.curvature_set) {
        phd->add_curvature_set(cs);
      }
      phd->set_lane_type(static_cast<LaneType>(hd.lane_type));
      phd->set_restricted_lane_type(static_cast<RestrictedLaneType>(hd.restricted_lane_type));
      phd->clear_curvature_set_offset();
      for (auto const& cso : hd.curvature_set_offset) {
        phd->add_curvature_set_offset(cso);
      }
    }
    proto_lane_info->clear_incoming_lane_ids();
    for (auto const& incoming_lane_id: msg_lane_info.incoming_lane_ids) {
      proto_lane_info->add_incoming_lane_ids(incoming_lane_id);
    }
    proto_lane_info->set_restricted_lane_type(
        static_cast<RestrictedLaneType>(msg_lane_info.restricted_lane_type));
    proto_lane_info->set_leftline_quality(
        static_cast<LaneMarkingQuality>(msg_lane_info.leftline_quality));
    proto_lane_info->set_rightline_quality(
        static_cast<LaneMarkingQuality>(msg_lane_info.rightline_quality));
    proto_lane_info->set_is_lane_ending(msg_lane_info.is_lane_ending);
    proto_lane_info->set_lane_end_length(msg_lane_info.lane_end_length);
    proto_lane_info->set_lane_order(static_cast<LaneOrder>(msg_lane_info.lane_order));
  }

  // error_code
  proto_ptr->set_error_code(static_cast<ErrorCode>(msg_ptr->error_code));

  // localize_pose
  proto_ptr->mutable_em_pose()->set_x(msg_ptr->localize_state.x);
  proto_ptr->mutable_em_pose()->set_y(msg_ptr->localize_state.y);
  proto_ptr->mutable_em_pose()->set_theta(msg_ptr->localize_state.theta);
}

} // namespace pnc
} // namespace xju