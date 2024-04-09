/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "simulation_panel.h"

#include <QButtonGroup>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QVBoxLayout>
#include <thread>

#include "common/logger/logger.h"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "tf2/utils.h"

namespace xju {
namespace simulator {

SimulationPanel::SimulationPanel(QWidget *parent) : rviz_common::Panel(parent) {
  QGridLayout *grid = new QGridLayout();

  // 0
  grid->addWidget(new QLabel("v:"), 0, 0);
  text_v_ = new QLineEdit();
  text_v_->setText("0.0");
  // text_v_->setClearButtonEnabled(true);
  // text_v_->setPlaceholderText("m/s");
  box_v_ = new QComboBox();
  box_v_->setLineEdit(text_v_);
  // grid->addWidget(text_v_, 0, 1);
  box_v_->addItem("0");
  box_v_->addItem("3.0");
  box_v_->addItem("10.0");
  box_v_->addItem("20.0");
  grid->addWidget(box_v_, 0, 1);
  grid->addWidget(new QLabel("type:"), 0, 2);
  box_label_ = new QComboBox();
  connect(box_label_, SIGNAL(currentTextChanged(QString)), this,
          SLOT(SetDefalutParam(QString)));
  grid->addWidget(box_label_, 0, 3);
  grid->addWidget(new QLabel("path_type:"), 0, 4);
  box_path_label_ = new QComboBox();
  grid->addWidget(box_path_label_, 0, 5);

  // 1
  grid->addWidget(new QLabel("length:"), 1, 0);
  text_length_ = new QLineEdit();
  grid->addWidget(text_length_, 1, 1);
  grid->addWidget(new QLabel("width:"), 1, 2);
  text_width_ = new QLineEdit();
  grid->addWidget(text_width_, 1, 3);
  grid->addWidget(new QLabel("height:"), 1, 4);
  text_height_ = new QLineEdit();
  grid->addWidget(text_height_, 1, 5);

  // 2
  grid->addWidget(new QLabel("life_time:"), 2, 0);
  text_life_time_ = new QLineEdit();
  // text_life_time_->setText("60.0");
  box_life_time_ = new QComboBox();
  box_life_time_->setLineEdit(text_life_time_);
  box_life_time_->addItem("30.0");
  box_life_time_->addItem("60.0");
  box_life_time_->addItem("120.0");
  box_life_time_->addItem("240.0");
  text_life_time_->setText("60.0");
  // grid->addWidget(text_life_time_, 2, 1);
  grid->addWidget(box_life_time_, 2, 1);
  grid->addWidget(new QLabel("des_x:"), 2, 2);
  text_destination_x_ = new QLineEdit();
  text_destination_x_->setText("0.0");
  grid->addWidget(text_destination_x_, 2, 3);
  grid->addWidget(new QLabel("des_y:"), 2, 4);
  text_destination_y_ = new QLineEdit();
  text_destination_y_->setText("0.0");
  grid->addWidget(text_destination_y_, 2, 5);

  // 3
  grid->addWidget(new QLabel("init_x:"), 3, 0);
  text_init_x_ = new QLineEdit();
  text_init_x_->setText("0.0");
  grid->addWidget(text_init_x_, 3, 1);
  grid->addWidget(new QLabel("init_y:"), 3, 2);
  text_init_y_ = new QLineEdit();
  text_init_y_->setText("0.0");
  grid->addWidget(text_init_y_, 3, 3);
  grid->addWidget(new QLabel("init_heading:"), 3, 4);
  text_init_heading_ = new QLineEdit();
  text_init_heading_->setText("0.0");
  grid->addWidget(text_init_heading_, 3, 5);

  // 4
  grid->addWidget(new QLabel("init_v:"), 4, 0);
  text_init_v_ = new QLineEdit();
  text_init_v_->setText("0.0");
  grid->addWidget(text_init_v_, 4, 1);
  grid->addWidget(new QLabel("init_a:"), 4, 2);
  text_init_a_ = new QLineEdit();
  text_init_a_->setText("0.0");
  grid->addWidget(text_init_a_, 4, 3);
  reset_init_ = new QPushButton();
  reset_init_->setText("RESET_INIT");
  connect(reset_init_, SIGNAL(clicked()), this,
          SLOT(ClickResetInitStateButton()));
  grid->addWidget(reset_init_, 4, 4);

  // 5
  checkbox_planning_chart_ = new QCheckBox;
  checkbox_planning_chart_->setText("planning");
  connect(checkbox_planning_chart_, SIGNAL(clicked()), this,
          SLOT(ClickCheckPlanningChart()));
  grid->addWidget(checkbox_planning_chart_, 5, 0);

  checkbox_control_chart_ = new QCheckBox;
  checkbox_control_chart_->setText("control");
  connect(checkbox_control_chart_, SIGNAL(clicked()), this,
          SLOT(ClickCheckControlChart()));
  grid->addWidget(checkbox_control_chart_, 5, 1);

  start_simulation_ = new QPushButton();
  start_simulation_->setText("START");
  connect(start_simulation_, SIGNAL(clicked()), this,
          SLOT(ClickStartSimulationButton()));
  grid->addWidget(start_simulation_, 5, 2);

  grid->addWidget(new QLabel("sim_rate:"), 5, 3);
  box_sim_rate_ = new QComboBox();
  connect(box_sim_rate_, SIGNAL(currentTextChanged(QString)), this,
          SLOT(SetSimRate(QString)));
  grid->addWidget(box_sim_rate_, 5, 4);

  setLayout(grid);

  box_label_->addItem(QString(DEFALUT_CAR_LABEL));
  box_label_->addItem(QString(DEFALUT_TRUCK_LABEL));
  box_label_->addItem(QString(DEFALUT_PEOPLE_LABEL));
  box_label_->addItem(QString(DEFALUT_BICYCLE_LABEL));
  box_label_->addItem(QString(DEFALUT_STATIC_LABEL));
  SetDefalutParam(QString(DEFALUT_CAR_LABEL));

  box_path_label_->addItem("lane");   // 0
  box_path_label_->addItem("line");   // 1
  box_path_label_->addItem("curve");  // 2

  box_sim_rate_->addItem("1");
  box_sim_rate_->addItem("0");
  box_sim_rate_->addItem("2");
  box_sim_rate_->addItem("0.5");
  box_sim_rate_->addItem("0.1");
  box_sim_rate_->addItem("0.05");
}

void SimulationPanel::onInitialize() {
  //  tree_widget_->setModel(getDisplayContext()->getSelectionManager()->getPropertyModel());
  node_ = rviz_common::Panel::getDisplayContext()
              ->getRosNodeAbstraction()
              .lock()
              ->get_raw_node();
  goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1,
      std::bind(&SimulationPanel::GoalPoseCallback, this,
                std::placeholders::_1));

  pose_grabber_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_grabber", 1,
          std::bind(&SimulationPanel::PoseGrabberCallback, this,
                    std::placeholders::_1));

  command_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/rviz_xju_panel/command", 1);

  des_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/rviz_xju_panel/viz/destination", 1);

  destination_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pnc/destination", 1);
}

void SimulationPanel::SetDefalutParam(const QString &label) {
  if (label == "car") {
    text_length_->setText(DEFALUT_CAR_LENGTH);
    text_width_->setText(DEFALUT_CAR_WIDTH);
    text_height_->setText(DEFALUT_CAR_HEIGHT);
    text_v_->setText(DEFALUT_CAR_SPEED);
  } else if (label == "truck") {
    text_length_->setText(DEFALUT_TRUCK_LENGTH);
    text_width_->setText(DEFALUT_TRUCK_WIDTH);
    text_height_->setText(DEFALUT_TRUCK_HEIGHT);
    text_v_->setText(DEFALUT_TRUCK_SPEED);
  } else if (label == "people") {
    text_length_->setText(DEFALUT_PEOPLE_LENGTH);
    text_width_->setText(DEFALUT_PEOPLE_WIDTH);
    text_height_->setText(DEFALUT_PEOPLE_HEIGHT);
    text_v_->setText(DEFALUT_PEOPLE_SPEED);
  } else if (label == "bicycle") {
    text_length_->setText(DEFALUT_BICYCLE_LENGTH);
    text_width_->setText(DEFALUT_BICYCLE_WIDTH);
    text_height_->setText(DEFALUT_BICYCLE_HEIGHT);
    text_v_->setText(DEFALUT_BICYCLE_SPEED);
  } else if (label == "static") {
    text_length_->setText(DEFALUT_STATIC_LENGTH);
    text_width_->setText(DEFALUT_STATIC_WIDTH);
    text_height_->setText(DEFALUT_STATIC_HEIGHT);
    text_v_->setText(DEFALUT_STATIC_SPEED);
  }
}

void SimulationPanel::SetSimRate(const QString &label) {
  if (!node_) {
    return;
  }
  double sim_rate = 1.0;
  if (label == "1") {
    sim_rate = 1;
  } else if (label == "0") {
    sim_rate = 0;
  } else if (label == "2") {
    sim_rate = 2;
  } else if (label == "0.5") {
    sim_rate = 0.5;
  } else if (label == "0.1") {
    sim_rate = 0.1;
  } else if (label == "0.05") {
    sim_rate = 0.05;
  }
  // publish change_sim_rate cmd
  geometry_msgs::msg::PoseWithCovarianceStamped psd_obj_cmd;
  psd_obj_cmd.header.frame_id = "change_sim_rate";
  psd_obj_cmd.pose.covariance[0] = sim_rate;
  command_pub_->publish(psd_obj_cmd);
}

void SimulationPanel::ClickCheckPlanningChart() {
  geometry_msgs::msg::PoseWithCovarianceStamped psd_obj_cmd;
  psd_obj_cmd.header.frame_id = "planning_chart";
  if (checkbox_planning_chart_->isChecked()) {
    psd_obj_cmd.pose.covariance[0] = 1;
  } else {
    psd_obj_cmd.pose.covariance[0] = -1;
  }
  command_pub_->publish(psd_obj_cmd);
}

void SimulationPanel::ClickCheckControlChart() {
  geometry_msgs::msg::PoseWithCovarianceStamped psd_obj_cmd;
  psd_obj_cmd.header.frame_id = "control_chart";
  if (checkbox_control_chart_->isChecked()) {
    psd_obj_cmd.pose.covariance[0] = 1;
  } else {
    psd_obj_cmd.pose.covariance[0] = -1;
  }
  command_pub_->publish(psd_obj_cmd);
}

void SimulationPanel::ClickStartSimulationButton() {
  // publish start command
  geometry_msgs::msg::PoseStamped des_point;
  des_point.pose.position.x = text_destination_x_->text().toDouble();
  des_point.pose.position.y = text_destination_y_->text().toDouble();
  destination_pub_->publish(des_point);
}

void SimulationPanel::ClickResetInitStateButton() {
  geometry_msgs::msg::PoseWithCovarianceStamped psd;
  psd.header.frame_id = "reset_init_state";
  psd.pose.pose.position.x = text_init_x_->text().toDouble();        // x
  psd.pose.pose.position.y = text_init_y_->text().toDouble();        // y
  psd.pose.pose.position.z = text_init_heading_->text().toDouble();  // heading
  psd.pose.pose.orientation.x = text_init_v_->text().toDouble();     // v
  psd.pose.pose.orientation.y = text_init_a_->text().toDouble();     // a
  command_pub_->publish(psd);
}

void SimulationPanel::GoalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  geometry_msgs::msg::PoseWithCovarianceStamped psd;
  psd.header.frame_id = "add_object";
  psd.pose.pose = msg->pose;

  psd.pose.covariance[0] = object_id_;                          // id
  psd.pose.covariance[1] = text_v_->text().toDouble();          // v
  psd.pose.covariance[2] = text_life_time_->text().toDouble();  // life_time
  psd.pose.covariance[4] = text_length_->text().toDouble();     // length
  psd.pose.covariance[5] = text_width_->text().toDouble();      // width
  psd.pose.covariance[6] = text_height_->text().toDouble();     // height
  psd.pose.covariance[7] = box_path_label_->currentIndex();     // path_type
  psd.pose.covariance[8] = box_label_->currentIndex();          // object_type

  // publish add command
  command_pub_->publish(psd);
  object_id_++;
}

void SimulationPanel::PoseGrabberCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // if (start_simulation_->text().toStdString() == "STOP") {
  //   return;
  // }
  text_destination_x_->setText(std::to_string(msg->pose.position.x).c_str());
  text_destination_y_->setText(std::to_string(msg->pose.position.y).c_str());

  // publish destination marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "local";
  marker.ns = "destination";
  marker.id = 0;
  marker.type = 2;
  marker.action = 0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.pose.position.x = msg->pose.position.x;
  marker.pose.position.y = msg->pose.position.y;
  marker.pose.position.z = 0;
  des_marker_pub_->publish(marker);
}

}  // namespace simulator
}  // namespace xju

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(xju::simulator::SimulationPanel, rviz_common::Panel)
