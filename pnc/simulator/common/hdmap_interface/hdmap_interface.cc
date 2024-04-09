/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "simulator/common/hdmap_interface/hdmap_interface.h"

#include "common/logger/logger.h"
#include "simulator/common/scenario_interface/scenario_interface.h"

using TiXmlDocument = tinyxml2::XMLDocument;
using TiXmlElement = tinyxml2::XMLElement;

namespace xju {
namespace simulator {

bool HDMapInterface::InitHDMap(const char* file_path) {
  hdmap_->Reset();

  if (!LoadHDMap(file_path)) {
    return false;
  }
  if (!UpdateHDMap()) {
    return false;
  }
  return true;
}

bool HDMapInterface::LoadHDMap(const char* file_path) {
  if (!file_path) {
    AERROR << "File path null";
    return false;
  }

  TiXmlDocument file;
  if (0 != file.LoadFile(file_path)) {
    AERROR << "Load xml file failed";
    return false;
  }

  // root
  TiXmlElement* root = file.RootElement();  // osm tag

  if (!root) {
    AERROR << "Root error";
    return false;
  }

  // node
  TiXmlElement* first_child_node = root->FirstChildElement("node");
  if (!first_child_node) {
    AERROR << "First_child_node error";
    return false;
  }

  TiXmlElement* node = first_child_node;
  while (node) {
    if (strcmp(node->Value(), "node") == 0) {
      int id = atoi(node->Attribute("id"));
      double x, y;
      //   TiXmlElement* c_node = node->FirstChildElement("tag");
      TiXmlElement* c_node = node->FirstChildElement();
      while (c_node) {
        if (strcmp(c_node->Value(), "tag") == 0) {
          if (strcmp(c_node->Attribute("k"), "local_x") == 0) {
            x = atof(c_node->Attribute("v"));
          } else if (strcmp(c_node->Attribute("k"), "local_y") == 0) {
            y = atof(c_node->Attribute("v"));
          }
        }
        c_node = c_node->NextSiblingElement();
      }
      hdmap_->points_map[std::move(id)] =
          pnc::Vec2d(std::move(x), std::move(y));
    } else if (strcmp(node->Value(), "way") == 0) {
      int id = atoi(node->Attribute("id"));
      std::vector<int> points_ids;
      //   auto c_node = node->FirstChildElement("nd");
      auto c_node = node->FirstChildElement();
      while (c_node) {
        if (strcmp(c_node->Value(), "nd") == 0) {
          points_ids.push_back(atoi(c_node->Attribute("ref")));
        }
        c_node = c_node->NextSiblingElement();
      }
      hdmap_->lines_map[std::move(id)] = std::move(points_ids);
    } else if (strcmp(node->Value(), "relation") == 0) {
      int id = atoi(node->Attribute("id"));
      int left_line_id, right_line_id;
      //   auto c_node = node->FirstChildElement("member");
      auto c_node = node->FirstChildElement();
      while (c_node) {
        if (strcmp(c_node->Value(), "member") == 0) {
          if (strcmp(c_node->Attribute("role"), "left") == 0) {
            left_line_id = atoi(c_node->Attribute("ref"));
            // AINFO << left_line_id;
          } else if (strcmp(c_node->Attribute("role"), "right") == 0) {
            right_line_id = atoi(c_node->Attribute("ref"));
            // AINFO << right_line_id;
          }
        }
        c_node = c_node->NextSiblingElement();
      }
      hdmap_->lanes_map[std::move(id)] = {left_line_id, right_line_id};
    }
    node = node->NextSiblingElement();
  }
  return true;
}

bool HDMapInterface::UpdateHDMap() {
  for (const auto& it : hdmap_->lanes_map) {
    Lane lane;
    // id
    lane.id = it.first;

    // boundary
    int left_line_id = it.second[0];
    int right_line_id = it.second[1];
    auto left_boundary_size = hdmap_->lines_map[left_line_id].size();
    auto right_boundary_size = hdmap_->lines_map[right_line_id].size();

    if (left_boundary_size != right_boundary_size) {
      AERROR << "HDMap error";
      return false;
    }
    // left_points
    for (const auto& point_id : hdmap_->lines_map[left_line_id]) {
      lane.left_boundary.push_back(hdmap_->points_map[point_id]);
    }
    // right_points
    for (const auto& point_id : hdmap_->lines_map[right_line_id]) {
      lane.right_boundary.push_back(hdmap_->points_map[point_id]);
    }

    // center
    for (int i = 0; i < lane.right_boundary.size(); i++) {
      const auto& vec_point =
          (lane.left_boundary[i] + lane.right_boundary[i]) / 2.0;
      if (i > 0) {
        lane.length += vec_point.DistanceTo(lane.center_line.back());
      }
      lane.center_line.push_back(vec_point);
    }

    hdmap_->hdmap_lanes.push_back(std::move(lane));
  }
  ReSampleBoundaryAndCenterLine();
  UpdateLanesRelation();
  for (const auto& it : hdmap_->hdmap_lanes) {
    hdmap_->hdmap_lanes_map[it.id] = it;
  }
  return true;
}

void HDMapInterface::ReSampleBoundaryAndCenterLine() {
  if (!hdmap_) return;
  auto resample = [](std::vector<pnc::Vec2d>& points, double resolution) {
    std::vector<pnc::Vec2d> new_points;
    for (int i = 0; i < points.size() - 1; i++) {
      new_points.push_back(points[i]);
      double length = (points[i + 1] - points[i]).Length();
      int point_num = int(length / resolution);
      if (point_num > 0) {
        double step_s = length / (point_num + 1);
        double v_length = points[i].DistanceTo(points[i + 1]);
        auto v = points[i + 1] - points[i];
        for (int num = 1; num <= point_num; num++) {
          double ratio = num * step_s / v_length;
          pnc::Vec2d t_point = points[i] + v * ratio;
          new_points.push_back(std::move(t_point));
        }
      }
    }
    if (!points.empty()) new_points.push_back(points.back());
    points.swap(new_points);
  };

  double resolution = 1.0;
  for (auto& lane : hdmap_->hdmap_lanes) {
    resample(lane.center_line, resolution);
    resample(lane.left_boundary, resolution);
    resample(lane.right_boundary, resolution);
  }
}

void HDMapInterface::UpdateLanesRelation() {
  int size = hdmap_->hdmap_lanes.size();
  for (int i = 0; i < size - 1; i++) {
    auto i_front = hdmap_->hdmap_lanes[i].center_line.front();
    auto i_back = hdmap_->hdmap_lanes[i].center_line.back();
    for (int j = i + 1; j < size; j++) {
      auto j_front = hdmap_->hdmap_lanes[j].center_line.front();
      auto j_back = hdmap_->hdmap_lanes[j].center_line.back();
      if (i_back.DistanceTo(j_front) < 1.0) {
        hdmap_->hdmap_lanes[i].successor_ids.push_back(
            hdmap_->hdmap_lanes[j].id);
        hdmap_->hdmap_lanes[j].predecessor_ids.push_back(
            hdmap_->hdmap_lanes[i].id);
        continue;
      }
      if (i_front.DistanceTo(j_back) < 1.0) {
        hdmap_->hdmap_lanes[i].predecessor_ids.push_back(
            hdmap_->hdmap_lanes[j].id);
        hdmap_->hdmap_lanes[j].successor_ids.push_back(
            hdmap_->hdmap_lanes[i].id);
        continue;
      }
      if (i_front.DistanceTo(j_front) > 2.5 &&
          i_front.DistanceTo(j_front) < 4.5 &&
          i_back.DistanceTo(j_back) > 2.5 && i_back.DistanceTo(j_back) < 4.5) {
        // judge left or right
        auto new_i_vec = i_back - i_front;
        auto new_vec = j_back - i_front;
        if (new_i_vec.CrossProd(new_vec) > 0) {
          // j is in the left
          hdmap_->hdmap_lanes[i].left_id = hdmap_->hdmap_lanes[j].id;
          hdmap_->hdmap_lanes[j].right_id = hdmap_->hdmap_lanes[i].id;
        } else {
          // j is in the right
          hdmap_->hdmap_lanes[i].right_id = hdmap_->hdmap_lanes[j].id;
          hdmap_->hdmap_lanes[j].left_id = hdmap_->hdmap_lanes[i].id;
        }
      }
    }
  }
}

const std::vector<int>* const HDMapInterface::GetSuccessorIds(int id) {
  if (!hdmap_) {
    return nullptr;
  }
  if (hdmap_->hdmap_lanes_map.find(id) == hdmap_->hdmap_lanes_map.end()) {
    return nullptr;
  }
  if (hdmap_->hdmap_lanes_map[id].successor_ids.empty()) {
    return nullptr;
  }
  return &(hdmap_->hdmap_lanes_map[id].successor_ids);
}

const std::vector<int>* const HDMapInterface::GetPredecessorIds(int id) {
  if (!hdmap_) {
    return nullptr;
  }
  if (hdmap_->hdmap_lanes_map.find(id) == hdmap_->hdmap_lanes_map.end()) {
    return nullptr;
  }
  if (hdmap_->hdmap_lanes_map[id].predecessor_ids.empty()) {
    return nullptr;
  }
  return &(hdmap_->hdmap_lanes_map[id].predecessor_ids);
}

int HDMapInterface::GetLeftId(int id) {
  if (!hdmap_) return -1;
  return hdmap_->hdmap_lanes_map[id].left_id;
}

int HDMapInterface::GetRightId(int id) {
  if (!hdmap_) return -1;
  return hdmap_->hdmap_lanes_map[id].right_id;
}

const Lane& HDMapInterface::GetLane(int id) {
  return hdmap_->hdmap_lanes_map[id];
}

const Lane& HDMapInterface::GetNearestLane(double x, double y, double theta,
                                           int id) {
  int current_id = GetNearestLaneId(x, y, theta, id);
  return GetLane(current_id);
}

int HDMapInterface::GetNearestLaneId(double x, double y, double theta, int id) {
  pnc::Vec2d vec_point(x, y);
  double dis = std::numeric_limits<double>::max();
  int nearest_lane_id = -1;
  if (id > 0) {
    // current
    if (hdmap_->hdmap_lanes_map.find(id) == hdmap_->hdmap_lanes_map.end()) {
      return id;
    }
    for (const auto& point : hdmap_->hdmap_lanes_map[id].center_line) {
      double t_dis = vec_point.DistanceSquareTo(point);
      if (t_dis < dis) {
        dis = t_dis;
        nearest_lane_id = id;
      }
    }
    // successor
    for (int succ_id : hdmap_->hdmap_lanes_map[id].successor_ids) {
      for (const auto& point : hdmap_->hdmap_lanes_map[succ_id].center_line) {
        double t_dis = vec_point.DistanceSquareTo(point);
        if (t_dis < dis) {
          dis = t_dis;
          nearest_lane_id = succ_id;
        } else {
          // break;
        }
      }
    }
    // left
    auto left_id = hdmap_->hdmap_lanes_map[id].left_id;
    if (left_id > 0) {
      for (const auto& point : hdmap_->hdmap_lanes_map[left_id].center_line) {
        double t_dis = vec_point.DistanceSquareTo(point);
        if (t_dis < dis) {
          dis = t_dis;
          nearest_lane_id = left_id;
        }
      }
    }

    // right
    auto right_id = hdmap_->hdmap_lanes_map[id].right_id;
    if (right_id > 0) {
      for (const auto& point : hdmap_->hdmap_lanes_map[right_id].center_line) {
        double t_dis = vec_point.DistanceSquareTo(point);
        if (t_dis < dis) {
          dis = t_dis;
          nearest_lane_id = right_id;
        }
      }
    }
    // predecessor
    // TODO

  } else {
    for (const auto& lane : hdmap_->hdmap_lanes) {
      for (const auto& point : lane.center_line) {
        double t_dis = vec_point.DistanceSquareTo(point);
        if (t_dis < dis) {
          dis = t_dis;
          nearest_lane_id = lane.id;
        }
      }
    }
  }
  return nearest_lane_id;
}

// bool HDMapInterface::InitHDMapWithScenario(const char* file_path) {
//   hdmap_->Reset();
//   auto scenario_interface = ScenarioInterface::GetInstance();
//   if (!scenario_interface->InitScenario(file_path)) {
//     AERROR << "Scenario interface init failed";
//     return false;
//   }
//   const auto& lanelets = scenario_interface->GetLanelets();
//   for (const auto& lanelet : lanelets) {
//     Lane lane;
//     lane.id = lanelet.id;
//     lane.left_boundary = lanelet.left_boundary;
//     lane.right_boundary = lanelet.right_boundary;
//     lane.left_id = lanelet.left_id;
//     lane.right_id = lanelet.right_id;
//     lane.successor_ids = lanelet.successor_ids;
//     lane.predecessor_ids = lanelet.predecessor_ids;
//     for (int i = 0;
//          i < std::min(lane.left_boundary.size(), lane.right_boundary.size());
//          i++) {
//       auto point = (lane.left_boundary[i] + lane.right_boundary[i]) / 2.0;
//       lane.center_line.push_back(std::move(point));
//       if (i >= 1) {
//         lane.length += lane.center_line[i].DistanceTo(lane.center_line[i - 1]);
//       }
//     }
//     hdmap_->hdmap_lanes.push_back(lane);
//     hdmap_->hdmap_lanes_map[lane.id] = lane;
//   }
//   return true;
// }

}  // namespace simulator
}  // namespace xju