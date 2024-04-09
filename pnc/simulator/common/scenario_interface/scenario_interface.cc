/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "simulator/common/scenario_interface/scenario_interface.h"

#include "common/logger/logger.h"

using namespace tinyxml2;

namespace xju {
namespace simulator {

static pnc::Vec2d GetXYFromPointXML(TiXmlElement* point) {
  assert(point);
  double x = 0, y = 0;
  auto* node = point->FirstChildElement();
  while (node) {
    if (strcmp(node->Value(), "x") == 0) {
      x = atof(node->GetText());
    } else if (strcmp(node->Value(), "y") == 0) {
      y = atof(node->GetText());
    }
    node = node->NextSiblingElement();
  }
  return {x, y};
}

bool ScenarioInterface::InitScenario(const char* file_path) {
  scenario_->Reset();

  if (!file_path) {
    AERROR << "File path null";
    return false;
  }

  TiXmlDocument file(file_path);
  if (!file.LoadFile()) {
    AERROR << "Load xml file failed";
    return false;
  }

  // root
  TiXmlElement* root = file.RootElement();  // commonRoad tag

  if (!root) {
    AERROR << "Root error";
    return false;
  }

  TiXmlElement* node = root->FirstChildElement();
  while (node) {
    if (strcmp(node->Value(), "lanelet") == 0) {
      Lanelet lanelet;
      lanelet.id = atoi(node->Attribute("id"));
      TiXmlElement* c_node = node->FirstChildElement();
      while (c_node) {
        if (strcmp(c_node->Value(), "leftBound") == 0) {
          auto* cc_node = c_node->FirstChildElement();
          while (cc_node) {
            double x, y;
            auto* ccc_node = cc_node->FirstChildElement();
            while (ccc_node) {
              if (strcmp(ccc_node->Value(), "x") == 0) {
                x = atof(ccc_node->GetText());
              } else if (strcmp(ccc_node->Value(), "y") == 0) {
                y = atof(ccc_node->GetText());
              }
              ccc_node = ccc_node->NextSiblingElement();
            }
            lanelet.left_boundary.emplace_back(x, y);
            cc_node = cc_node->NextSiblingElement();
          }
        } else if (strcmp(c_node->Value(), "rightBound") == 0) {
          auto* cc_node = c_node->FirstChildElement();
          while (cc_node) {
            double x, y;
            auto* ccc_node = cc_node->FirstChildElement();
            while (ccc_node) {
              if (strcmp(ccc_node->Value(), "x") == 0) {
                x = atof(ccc_node->GetText());
              } else if (strcmp(ccc_node->Value(), "y") == 0) {
                y = atof(ccc_node->GetText());
              }
              ccc_node = ccc_node->NextSiblingElement();
            }
            lanelet.right_boundary.emplace_back(x, y);
            cc_node = cc_node->NextSiblingElement();
          }
        } else if (strcmp(c_node->Value(), "predecessor") == 0) {
          lanelet.predecessor_ids.push_back(atoi(c_node->Attribute("ref")));
        } else if (strcmp(c_node->Value(), "successor") == 0) {
          lanelet.successor_ids.push_back(atoi(c_node->Attribute("ref")));
        } else if (strcmp(c_node->Value(), "adjacentLeft") == 0) {
          lanelet.left_id = atoi(c_node->Attribute("ref"));
        } else if (strcmp(c_node->Value(), "adjacentReft") == 0) {
          lanelet.right_id = atoi(c_node->Attribute("ref"));
        } else if (strcmp(c_node->Value(), "laneletType") == 0) {
          lanelet.lanelet_type = c_node->GetText();
        }
        c_node = c_node->NextSiblingElement();
      }
      scenario_->lanelets.push_back(std::move(lanelet));
    } else if (strcmp(node->Value(), "dynamicObstacle") == 0) {
      DynamicObstacle dy_obs;

      scenario_->dynamic_obstacles.push_back(std::move(dy_obs));
    } else if (strcmp(node->Value(), "planningProblem") == 0) {
      TiXmlElement* c_node = node->FirstChildElement();
      if (strcmp(c_node->Value(), "initialState") == 0) {
        auto* cc_node = c_node->FirstChildElement();
        if (strcmp(cc_node->Value(), "position") == 0) {
          auto* ccc_node = cc_node->FirstChildElement();
          if (strcmp(ccc_node->Value(), "point") == 0) {
            auto point = GetXYFromPointXML(ccc_node);
            scenario_->planning_problem.init_x = point.x();
            scenario_->planning_problem.init_y = point.y();
          }
        } else if (strcmp(cc_node->Value(), "orientation") == 0) {
          auto* ccc_node = cc_node->FirstChildElement();
          if (strcmp(ccc_node->Value(), "exact") == 0) {
            scenario_->planning_problem.init_heading =
                atof(ccc_node->GetText());
          }
        } else if (strcmp(cc_node->Value(), "time") == 0) {
          // TODO
        } else if (strcmp(cc_node->Value(), "velocity") == 0) {
          auto* ccc_node = cc_node->FirstChildElement();
          if (strcmp(ccc_node->Value(), "exact") == 0) {
            scenario_->planning_problem.init_v = atof(ccc_node->GetText());
          }
        } else if (strcmp(cc_node->Value(), "yawRate") == 0) {
          // TODO
        } else if (strcmp(cc_node->Value(), "slipAngle") == 0) {
          // TODO
        }

      } else if (strcmp(c_node->Value(), "goalState") == 0) {
        // TODO
      }
    } else if (strcmp(node->Value(), "trafficSign") == 0) {
      // TODO
    } else if (strcmp(node->Value(), "trafficLight") == 0) {
      // TODO
    } else if (strcmp(node->Value(), "intersection") == 0) {
      // TODO
    } else if (strcmp(node->Value(), "staticObstacle") == 0) {
      // TODO
    }
    node = node->NextSiblingElement();
  }

  return true;
}

}  // namespace simulator
}  // namespace xju