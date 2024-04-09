/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "simulator/common/hdmap_interface/hdmap.h"
#include "simulator/common/tinyxml/tinyxml2.h"

namespace xju {
namespace simulator {

class HDMapInterface {
 public:
  HDMapInterface() { hdmap_ = std::make_shared<HDMap>(); };
  ~HDMapInterface() = default;

  static std::shared_ptr<HDMapInterface> GetInstance() {
    static auto instance = std::make_shared<HDMapInterface>();
    return instance;
  }
  bool InitHDMap(const char* file_path);

  // bool InitHDMapWithScenario(const char* file_path);

  std::shared_ptr<HDMap> const GetHDMap() { return hdmap_; }
  int GetNearestLaneId(double x, double y, double theta, int id = -1);
  const Lane& GetNearestLane(double x, double y, double theta, int id = -1);
  const std::vector<int>* const GetSuccessorIds(int id);
  const std::vector<int>* const GetPredecessorIds(int id);
  int GetLeftId(int id);
  int GetRightId(int id);
  const Lane& GetLane(int id);

 private:
  bool LoadHDMap(const char* file_path);
  bool UpdateHDMap();
  void UpdateLanesRelation();
  void ReSampleBoundaryAndCenterLine();

 private:
  std::shared_ptr<HDMap> hdmap_;
};

}  // namespace simulator
}  // namespace xju
