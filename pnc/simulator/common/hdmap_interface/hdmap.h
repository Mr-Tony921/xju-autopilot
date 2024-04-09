/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include <unordered_map>
#include <vector>

#include "common/math/vec2d.h"

namespace xju {
namespace simulator {

struct Lane {
  int id;
  std::vector<pnc::Vec2d> left_boundary;
  std::vector<pnc::Vec2d> right_boundary;
  std::vector<pnc::Vec2d> center_line;
  double length = 0.0;
  int left_id = -1;
  int right_id = -1;
  std::vector<int> successor_ids;
  std::vector<int> predecessor_ids;
};

struct HDMap {
  // hdmap data
  std::vector<Lane> hdmap_lanes;
  std::unordered_map<int, Lane> hdmap_lanes_map;

  //  origin data
  std::unordered_map<int, pnc::Vec2d> points_map;
  std::unordered_map<int, std::vector<int>> lines_map;
  std::unordered_map<int, std::vector<int>> lanes_map;  // left right

  void Reset() {
    hdmap_lanes.clear();
    hdmap_lanes_map.clear();
    points_map.clear();
    lines_map.clear();
    lanes_map.clear();
  }
};

}  // namespace simulator
}  // namespace xju