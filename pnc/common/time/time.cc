/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/time/time.h"

#include <ctime>
#include <iomanip>
#include <sstream>

#include "common/logger/logger.h"
#include "rclcpp/rclcpp.hpp"

namespace xju {
namespace pnc {

using std::chrono::high_resolution_clock;
using std::chrono::steady_clock;
using std::chrono::system_clock;

double Time::NowInSeconds() {
  return static_cast<double>(Now()) / 1000000000UL;
}

double Time::NowInMilliSeconds() {
  return static_cast<double>(Now()) / 1000000.0;
}

double Time::NowInNanoSeconds() { return static_cast<double>(Now()); }

std::string Time::NowInDate() {
  double now_seconds = NowInSeconds();
  return NowInDateWithSec(now_seconds);
}

std::string Time::NowInDateWithSec(const double sec) {
  auto sec_int = int64_t(sec);
  auto ms_rec = static_cast<int>((sec - static_cast<double>(sec_int)) * 1e3);

  struct tm stm;
  auto ret = localtime_r(&sec_int, &stm);
  if (ret == nullptr) {
    return std::to_string(sec);
  }

  char date[60] = {0};
  sprintf(date, "%d-%02d-%02d_%02d:%02d:%02d.%02d", stm.tm_year + 1900,
          stm.tm_mon + 1, stm.tm_mday, stm.tm_hour, stm.tm_min, stm.tm_sec,
          ms_rec);
  return std::string(date);
}

uint64_t Time::Now() {
  static std::shared_ptr<rclcpp::Node> time_node = nullptr;
  static bool use_sim_time = false;
  if (!time_node) {
    time_node = std::make_shared<rclcpp::Node>("time");
    time_node->get_parameter("use_sim_time", use_sim_time);
  }
  if (use_sim_time && time_node) {
    return time_node->get_clock()->now().nanoseconds();
  }
  auto now = high_resolution_clock::now();
  auto nano_time_point =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = nano_time_point.time_since_epoch();
  uint64_t now_nano =
      std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  return now_nano;
}

}  // namespace pnc
}  // namespace xju
