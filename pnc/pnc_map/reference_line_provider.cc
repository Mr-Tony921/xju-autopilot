/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "pnc_map/reference_line_provider.h"
#include "pnc_map/pnc_map_gflags/pnc_map_gflags.h"

#include "common/logger/logger.h"
#include "common/time/time.h"
#include "common/util/coordinate.h"
#include "pnc_map/data_pool.h"

namespace xju {
namespace pnc_map {

bool ReferenceLineProvider::CreateReferenceLines(
    const std::shared_ptr<pnc::EmLanes>& em_lines,
    const pnc::LocalizePose& pnc_loc,
    std::list<ReferenceLine>* reference_lines) {
  auto get_lane_mark = [&](int id, pnc::LaneMark* const lane_mark) {
    bool found = false;
    for (const auto& tmp : em_lines->lane_mark()) {
      if (tmp.id() == id) {
        found = true;
        *lane_mark = tmp;
        break;
      }
    }

    return found;
  };

  auto get_road_width = [&](int order) {
    double lw = 0.0;
    double rw = 0.0;
    for (const auto& lane : em_lines->lane_info()) {
      if (order > lane.lane_order()) { // right
        rw += lane.width();
      } else if (order < lane.lane_order()) { // left
        lw += lane.width();
      } else { // current
        lw += 0.5 * lane.width();
        rw += 0.5 * lane.width();
      }
    }
    std::pair<double, double> res = std::make_pair(lw, rw);
    return res;
  };

  if (!reference_lines) {
    AERROR << "reference_lines is nullptr!";
    return false;
  }

  auto start = pnc::Time::NowInSeconds();
  reference_lines->clear();
  auto lane_process = pnc_map::LaneProcess::GetInstance();

  static uint64_t seq = 0;
  pnc::LocalizePose em_loc;
  em_loc.x = em_lines->em_pose().x();
  em_loc.y = em_lines->em_pose().y();
  em_loc.heading = em_lines->em_pose().theta();
  lane_process->UpdateTrackingUnits(seq++, em_loc, pnc_loc);
  for (const auto& lane : em_lines->lane_info()) {
    ReferenceLine ref_line{};

    // get lane id
    int lane_id = lane.lane_id();
    int left_id = lane.leftline_id();
    int right_id = lane.rightline_id();

    // get side lane marks
    pnc::LaneMark left_lm, right_lm;
    bool can_get_left = get_lane_mark(left_id, &left_lm);
    bool can_get_right = get_lane_mark(right_id, &right_lm);
    if (!can_get_left || !can_get_right) {
      AERROR << "Lane " << lane_id << " get left or right lanemark failed.";
      continue;
    }

    // lane process
    if (!lane_process->Process(left_lm, right_lm, lane, &ref_line)) {
      AERROR << "Current lane " << lane_id << " generate centerline failed.";
      continue;
    }

    ref_line.set_id(std::to_string(lane_id));
    auto left_lane =
        std::find_if(em_lines->lane_info().begin(), em_lines->lane_info().end(),
                     [&](const pnc::LaneInfo& l) {
                       return l.lane_order() == lane.lane_order() + 1;
                     });
    auto has_left_lane = left_lane != em_lines->lane_info().end();
    auto right_lane =
        std::find_if(em_lines->lane_info().begin(), em_lines->lane_info().end(),
                     [&](const pnc::LaneInfo& l) {
                       return l.lane_order() == lane.lane_order() - 1;
                     });
    auto has_right_lane = right_lane != em_lines->lane_info().end();
    ref_line.set_left_lane_id(has_left_lane ? std::to_string(left_lane->lane_id()) : "");
    ref_line.set_right_lane_id(has_right_lane ? std::to_string(right_lane->lane_id()) : "");
    ref_line.set_lane_width(lane.width());
    auto road_width = get_road_width(lane.lane_order());
    ref_line.set_left_road_width(road_width.first);
    ref_line.set_right_road_width(road_width.second);
    ref_line.set_min_speed_limit(lane.min_speed_limit());
    ref_line.set_max_speed_limit(lane.max_speed_limit());
    ref_line.set_left_lane_marking_type(left_lm.type());
    ref_line.set_right_lane_marking_type(right_lm.type());
    ref_line.set_restricted_lane_type(lane.restricted_lane_type());
    ref_line.set_lane_order(lane.lane_order());
    ref_line.set_is_emergency_lane(lane.lane_type() == pnc::LaneType::LAT_EMERGENCY);
    ref_line.set_is_death_lane(lane.is_lane_ending());
    //TODO: get from map egine
    ref_line.set_is_recommended_lane(false);  // will be deleted !!!

    if (lane.hdmap_lane_data_size() > 0) {
      for (auto const& hdmap_unit : lane.hdmap_lane_data()) {
        if (hdmap_unit.left_linemarking().pts_vehicle_coord().empty() ||
            hdmap_unit.right_linemarking().pts_vehicle_coord().empty())
          continue;
        auto left_start_s = std::hypot(
            hdmap_unit.left_linemarking().pts_vehicle_coord().begin()->x(),
            hdmap_unit.left_linemarking().pts_vehicle_coord().begin()->y());
        auto left_end_s = std::hypot(
            (hdmap_unit.left_linemarking().pts_vehicle_coord().end() - 1)->x(),
            (hdmap_unit.left_linemarking().pts_vehicle_coord().end() - 1)->y());
        auto right_start_s = std::hypot(
            hdmap_unit.right_linemarking().pts_vehicle_coord().begin()->x(),
            hdmap_unit.right_linemarking().pts_vehicle_coord().begin()->y());
        auto right_end_s = std::hypot(
            (hdmap_unit.right_linemarking().pts_vehicle_coord().end() - 1)->x(),
            (hdmap_unit.right_linemarking().pts_vehicle_coord().end() - 1)->y());
        if (left_start_s > ref_line.length() || left_end_s < 0 ||
            right_start_s > ref_line.length() || right_end_s < 0)
          continue;

        ref_line.AddSpeedLimit(std::min(left_start_s, right_start_s),
                               std::max(left_end_s, right_end_s),
                               hdmap_unit.max_speed_limit());
      }
    } else {
      ref_line.AddSpeedLimit(0.0, ref_line.length(), lane.max_speed_limit());
    }
    reference_lines->emplace_back(ref_line);
  }

  time_delay_ms_ = (pnc::Time::NowInSeconds() - start) * 1000.0;
  ADEBUG << "Cost Time of Reference Line Provider is: " << time_delay_ms_ << "ms";

  DataPool::Instance()->set_reference_lines(*reference_lines);
  return true;
}

}  // namespace pnc_map
}  // namespace xju
