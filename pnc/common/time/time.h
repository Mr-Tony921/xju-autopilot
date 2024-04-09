/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <chrono>
#include <string>

namespace xju {
namespace pnc {

class Time {
 public:
  static double NowInSeconds();
  static double NowInMilliSeconds();
  static double NowInNanoSeconds();
  static std::string NowInDate();
  static std::string NowInDateWithSec(const double sec);

 private:
  static uint64_t Now();
};

} // namespace pnc
} // namespace xju
