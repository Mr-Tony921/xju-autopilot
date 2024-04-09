/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QString>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace xju {
namespace simulator {

// Object Command Msg
// geometry_msgs::msg::PoseWithCovarianceStamped
// psd: header:
//          frame_id: //command: add_object delete_object start_simulation
//          stop_simulation
//      pose:
//            pose:
//              pose:
//                position:
//                orientation:
//            covariance:
//                [0]: id
//                [1]: v
//                [2]: life_time
//                [3]: start_time
//                [4]: length
//                [5]: width
//                [6]: height
//                [7]: path_type   line lane curve
//                [8]: object_type   car truck people bicycle static

#define DEFALUT_CAR_LABEL "car"
#define DEFALUT_CAR_LENGTH "4.2"
#define DEFALUT_CAR_WIDTH "1.7"
#define DEFALUT_CAR_HEIGHT "1.5"
#define DEFALUT_CAR_SPEED "18.0"

#define DEFALUT_TRUCK_LABEL "truck"
#define DEFALUT_TRUCK_LENGTH "6.0"
#define DEFALUT_TRUCK_WIDTH "2.0"
#define DEFALUT_TRUCK_HEIGHT "3.0"
#define DEFALUT_TRUCK_SPEED "11.0"

#define DEFALUT_PEOPLE_LABEL "people"
#define DEFALUT_PEOPLE_LENGTH "0.3"
#define DEFALUT_PEOPLE_WIDTH "0.4"
#define DEFALUT_PEOPLE_HEIGHT "1.7"
#define DEFALUT_PEOPLE_SPEED "0.5"

#define DEFALUT_BICYCLE_LABEL "bicycle"
#define DEFALUT_BICYCLE_LENGTH "1.5"
#define DEFALUT_BICYCLE_WIDTH "0.6"
#define DEFALUT_BICYCLE_HEIGHT "1.8"
#define DEFALUT_BICYCLE_SPEED "2.0"

#define DEFALUT_STATIC_LABEL "static"
#define DEFALUT_STATIC_LENGTH "1.2"
#define DEFALUT_STATIC_WIDTH "1.2"
#define DEFALUT_STATIC_HEIGHT "1.2"
#define DEFALUT_STATIC_SPEED "0.0"

class SimulationPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit SimulationPanel(QWidget *parent = nullptr);
  void onInitialize() override;

 protected Q_SLOTS:
  void SetDefalutParam(const QString &label);
  void ClickStartSimulationButton();
  void ClickResetInitStateButton();
  void ClickCheckPlanningChart();
  void ClickCheckControlChart();
  void SetSimRate(const QString &label);

 private:
  void PoseGrabberCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  //   void PublishDestinationMarker();
  void Timer();

 private:
  QComboBox *box_label_;
  QComboBox *box_path_label_;
  QLineEdit *text_length_;
  QLineEdit *text_width_;
  QLineEdit *text_height_;
  QLineEdit *text_v_;
  QComboBox *box_v_;
  QLineEdit *text_life_time_;
  QComboBox *box_life_time_;
  QLineEdit *text_destination_x_;
  QLineEdit *text_destination_y_;
  QPushButton *start_simulation_;
  QLineEdit *text_init_x_;
  QLineEdit *text_init_y_;
  QLineEdit *text_init_heading_;
  QLineEdit *text_init_v_;
  QLineEdit *text_init_a_;
  QPushButton *reset_init_;
  QCheckBox *checkbox_planning_chart_;
  QCheckBox *checkbox_control_chart_;
  QComboBox *box_sim_rate_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_grabber_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      destination_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr des_marker_pub_;
  int object_id_ = 1;
};

}  // namespace simulator
}  // namespace xju
