
/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <utility>

#include <OgreVector3.h>

#include <QCursor> // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "visibility_control.hpp"

namespace rviz_rendering {
class Arrow;
} // namespace rviz_rendering

namespace xju {
namespace simulator {
class RVIZ_XJU_PLUGINS_PUBLIC PoseTool : public rviz_common::Tool {
public:
  PoseTool();

  ~PoseTool() override;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;

protected:
  virtual void onPoseSet(double x, double y, double theta) = 0;

  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

  void logPose(std::string designation, geometry_msgs::msg::Point position,
               geometry_msgs::msg::Quaternion orientation, double angle,
               std::string frame);

  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  enum State { Position, Orientation };
  State state_;
  double angle_;

  Ogre::Vector3 arrow_position_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  int processMouseLeftButtonPressed(
      std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseLeftButtonReleased();
  void makeArrowVisibleAndSetOrientation(double angle);
  double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);
};

} // namespace simulator
} // namespace xju
