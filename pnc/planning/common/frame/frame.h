/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include "planning/common/local_view.h"
#include "planning/common/obstacle/obstacle.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "pnc_map/reference_line.h"
#include "pnc_point.pb.h"
#include "vehicle_state.pb.h"

namespace xju {
namespace planning {

class Frame {
 public:
  Frame() = default;
  ~Frame() = default;

  bool Init(const double current_timestamp,
            const pnc::TrajectoryPoint& planning_start_point,
            const pnc::VehicleState& vehicle_state,
            const std::list<pnc_map::ReferenceLine>& reference_lines,
            const LocalView& local_view);

  std::list<std::shared_ptr<ReferenceLineInfo>>
  GetPrioritizedReferenceLineInfos() {
    return prioritized_reference_line_info_ptrs_;
  }

  void SetPrioritizedReferenceLineInfos(
      const std::list<std::shared_ptr<ReferenceLineInfo>>&
          reference_line_infos) {
    prioritized_reference_line_info_ptrs_ = reference_line_infos;
  }

  std::list<std::shared_ptr<ReferenceLineInfo>> GetReferenceLineInfos() {
    return reference_line_infos_;
  }

  const Obstacle* CreateStopObstacle(
      std::shared_ptr<ReferenceLineInfo> const reference_line_info,
      const std::string& obstacle_id, const double obstacle_s);

  const Obstacle* CreateStaticVirtualObstacle(const std::string& id,
                                              const pnc::Box2d& box);

  std::shared_ptr<ReferenceLineInfo> GetCurrentReferenceLineInfo() {
    return cur_reference_line_info_;
  }

  std::shared_ptr<ReferenceLineInfo> GetLeftReferenceLineInfo() {
    return left_reference_line_info_;
  }

  std::shared_ptr<ReferenceLineInfo> GetRightReferenceLineInfo() {
    return right_reference_line_info_;
  }

  const std::shared_ptr<ReferenceLineInfo> FindDriveReferenceLineInfo();

  const std::vector<const Obstacle*> obstacles() const;

  const double planning_start_time() const;
  const std::string& ego_lane_id() const;
  const std::string& recommended_lane_id() const;

  const pnc::VehicleState& vehicle_state() const;
  const pnc::TrajectoryPoint& planning_start_point() const;
  const bool NearDestination() const;
  const pnc::Vec2d& destination() const;
  const double DistToChangePoint() const;
  const std::vector<std::string>& LanesToChangePoint() const;
  void ClearLanesToChangePoint() { lane_ids_.clear(); };

 private:
  void Reset();
  bool BuildReferenceLineInfo(
      const std::list<pnc_map::ReferenceLine>& reference_lines);
  bool InitFrameData();
  void AlignPredictionTime();
  void AddObstacle(const Obstacle& obstacle);
  void DestinationConvert();
  void CalculationReferenceLineInfo();
  void BuildRoadChangeInfo();
 private:
  double planning_start_time_;
  LocalView local_view_;
  pnc::TrajectoryPoint planning_start_point_;
  pnc::VehicleState vehicle_state_;
  IndexedObstacles obstacles_;
  std::list<std::shared_ptr<ReferenceLineInfo>> reference_line_infos_;
  std::shared_ptr<ReferenceLineInfo> cur_reference_line_info_ = nullptr;
  std::shared_ptr<ReferenceLineInfo> left_reference_line_info_ = nullptr;
  std::shared_ptr<ReferenceLineInfo> right_reference_line_info_ = nullptr;
  std::list<std::shared_ptr<ReferenceLineInfo>>
      prioritized_reference_line_info_ptrs_;
  std::string ego_lane_id_;
  static std::string last_ego_lane_id_;
  std::string recommended_lane_id_;
  pnc::Vec2d destination_xy_;
  static inline pnc::Vec2d change_point_xy_ = {};
  static inline uint32_t change_direction_ = {};
  static inline std::vector<std::string> lane_ids_ = {};
};

}  // namespace planning
}  // namespace xju
