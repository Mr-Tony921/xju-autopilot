/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void DebugConversion(
    const Debug& proto,
    chart_msgs::msg::Debug* const msg) {
  msg->charts.clear();
  for (const auto& chart : proto.charts()) {
    chart_msgs::msg::Chart chart_msg;
    ChartConversion(chart, &chart_msg);
    msg->charts.push_back(chart_msg);
  }

  msg->double_variables.clear();
  for (const auto& value : proto.name_value1()) {
    chart_msgs::msg::DoubleVariablePari value_msg;
    value_msg.name = value.first;
    value_msg.value = value.second;
    msg->double_variables.push_back(value_msg);
  }

  msg->string_variables.clear();
  for (const auto& value : proto.name_value2()) {
    chart_msgs::msg::StringVariablePari value_msg;
    value_msg.name = value.first;
    value_msg.value = value.second;
    msg->string_variables.push_back(value_msg);
  }

  if (proto.has_msg()) {
    msg->msg = proto.msg();
  }
}

void ChartConversion(
    const Chart& proto,
    chart_msgs::msg::Chart* const msg) {
  if (proto.has_title()) {
    msg->title = proto.title();
  }
  if (proto.has_options()) {
    OptionsConversion(proto.options(), &(msg->options));
  }

  msg->lines.clear();
  for (const auto& line : proto.line()) {
    chart_msgs::msg::Line line_msg;
    LineConversion(line, &line_msg);
    msg->lines.push_back(line_msg);
  }

  msg->polygons.clear();
  for (const auto& polygon : proto.polygon()) {
    chart_msgs::msg::Polygon polygon_msg;
    PolygonConversion(polygon, &polygon_msg);
    msg->polygons.push_back(polygon_msg);
  }
}

void LineConversion(
    const Line& proto,
    chart_msgs::msg::Line* const msg) {
  if (proto.has_label()) {
    msg->label = proto.label();
  }

  if (proto.has_hide_label_in_legend()) {
    msg->hide_label_in_legend = proto.hide_label_in_legend();
  }

  msg->point.clear();
  common_msgs::msg::Point2D point;
  for (const auto& pt : proto.point()) {
    Point2DConversion(pt, &point);
    msg->point.push_back(point);
  }
  
  auto itr = proto.properties().find("color");
  if (itr == proto.properties().end()) {
    msg->color.r = 0.0;
    msg->color.g = 0.0;
    msg->color.b = 0.0;
    msg->color.a = 0.0;
  } else {
    RGBAConverison(itr->second, &(msg->color));
  }

  itr = proto.properties().find("linestyle");
  if (itr == proto.properties().end()) {
    msg->line_type = chart_msgs::msg::Line::SOLID;
  } else {
    LineStyleConversion(itr->second, &(msg->line_type));
  }
}

void PolygonConversion(
    const Polygon& proto, 
    chart_msgs::msg::Polygon* const msg) {
  if (proto.has_label()) {
    msg->label = proto.label();
  }

  if (proto.has_hide_label_in_legend()) {
    msg->hide_label_in_legend = proto.hide_label_in_legend();
  }

  msg->point.clear();
  common_msgs::msg::Point2D point;
  for (const auto& pt : proto.point()) {
    Point2DConversion(pt, &point);
    msg->point.push_back(point);
  }

  auto itr = proto.properties().find("color");
  if (itr == proto.properties().end()) {
    msg->color.r = 0.0;
    msg->color.g = 0.0;
    msg->color.b = 0.0;
    msg->color.a = 0.0;
  } else {
    RGBAConverison(itr->second, &(msg->color));
  }
}

void OptionsConversion(
    const Options& proto,
    chart_msgs::msg::Options* const msg) {
  if (proto.has_legend_display()) {
    msg->legend_display = proto.legend_display();
  }
  if (proto.has_x()) {
    AxisConversion(proto.x(), &(msg->x));
  }
  if (proto.has_y()) {
    AxisConversion(proto.y(), &(msg->y));
  }
  if (proto.has_aspect_ratio()) {
    msg->aspect_ratio = proto.aspect_ratio();
  }
  if (proto.has_sync_xy_window_size()) {
    msg->sync_xy_window_size = proto.sync_xy_window_size();
  }
}

void AxisConversion(
    const Axis& proto,
    chart_msgs::msg::Axis* const msg) {
  if (proto.has_min()) {
    msg->min = proto.min();
  }
  if (proto.has_max()) {
    msg->max = proto.max();
  }
  if (proto.has_label_string()) {
    msg->label_string = proto.label_string();
  }
  if (proto.has_window_size()) {
    msg->window_size = proto.window_size();
  }
  if (proto.has_step_size()) {
    msg->step_size = proto.step_size();
  }
  if (proto.has_mid_value()) {
    msg->mid_value = proto.mid_value();
  }
}

void RGBAConverison(
    const std::string& color,
    chart_msgs::msg::ColorRGBA* const msg) {
  msg->r = 0.0;
  msg->g = 0.0;
  msg->b = 0.0;
  msg->a = 1.0;

  if (color == "r") {
    msg->r = 1.0;
  } else if (color == "g") {
    msg->g = 1.0;
  } else if (color == "b") {
    msg->b = 1.0;
  } else {
    // default;
  }
}

void LineStyleConversion(
    const std::string& style,
    uint8_t* msg) {
  *msg = chart_msgs::msg::Line::SOLID;
  if (style == "solid") {
    *msg = chart_msgs::msg::Line::SOLID;
  } else if (style == "dashed") {
    *msg = chart_msgs::msg::Line::DASHED;
  } else if (style == "dotted") {
    *msg = chart_msgs::msg::Line::DOTTED;
  }
}

} // namespace pnc
} // namespace xju