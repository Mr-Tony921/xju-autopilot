// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef JSK_RVIZ_PLUGINS_PIE_CHART_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_PIE_CHART_DISPLAY_H_
#include <std_msgs/msg/float32.hpp>
#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
#include "overlay_utils.hpp"
#include <OgreColourValue.h>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#endif

namespace rviz_2d_overlay_plugins
{
  class PieChartDisplay
    : public rviz_common::RosTopicDisplay<std_msgs::msg::Float32>
  {
    Q_OBJECT
  public:
    PieChartDisplay();
    virtual ~PieChartDisplay();
    
    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; };
    virtual int getY() { return top_; };

  protected:
    virtual void onEnable();
    virtual void onDisable();
    virtual void onInitialize();
    virtual void processMessage(std_msgs::msg::Float32::ConstSharedPtr msg);
    virtual void drawPlot(double val);
    virtual void update(float wall_dt, float ros_dt);
    // properties
    rviz_common::properties::IntProperty* size_property_;
    rviz_common::properties::IntProperty* left_property_;
    rviz_common::properties::IntProperty* top_property_;
    rviz_common::properties::ColorProperty* fg_color_property_;
    rviz_common::properties::ColorProperty* bg_color_property_;
    rviz_common::properties::FloatProperty* fg_alpha_property_;
    rviz_common::properties::FloatProperty* fg_alpha2_property_;
    rviz_common::properties::FloatProperty* bg_alpha_property_;
    rviz_common::properties::IntProperty* text_size_property_;
    rviz_common::properties::FloatProperty* max_value_property_;
    rviz_common::properties::FloatProperty* min_value_property_;
    rviz_common::properties::BoolProperty* show_caption_property_;
    rviz_common::properties::BoolProperty* auto_color_change_property_;
    rviz_common::properties::ColorProperty* max_color_property_;
    rviz_common::properties::ColorProperty* med_color_property_;
    rviz_common::properties::FloatProperty* max_color_threshold_property_;
    rviz_common::properties::FloatProperty* med_color_threshold_property_;
    rviz_common::properties::BoolProperty* clockwise_rotate_property_;

    int left_;
    int top_;
    uint16_t texture_size_;
    QColor fg_color_;
    QColor bg_color_;
    QColor max_color_;
    QColor med_color_;
    int text_size_;
    bool show_caption_;
    bool auto_color_change_;
    int caption_offset_;
    double fg_alpha_;
    double fg_alpha2_;
    double bg_alpha_;
    double max_value_;
    double min_value_;
    double max_color_threshold_;
    double med_color_threshold_;
    float data_;
    bool update_required_;
    bool first_time_;
    rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
    bool clockwise_rotate_;
    
    std::mutex mutex_;
                       
  protected Q_SLOTS:
    void updateSize();
    void updateTop();
    void updateLeft();
    void updateBGColor();
    void updateTextSize();
    void updateFGColor();
    void updateFGAlpha();
    void updateFGAlpha2();
    void updateBGAlpha();
    void updateMinValue();
    void updateMaxValue();
    void updateShowCaption();
    void updateAutoColorChange();
    void updateMaxColor();
    void updateMedColor();
    void updateMaxColorThreshold();
    void updateMedColorThreshold();
    void updateClockwiseRotate();

  private:
  };

}

#endif
