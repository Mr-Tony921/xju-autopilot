/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <list>

#include "common/design_pattern/singleton.h"
#include "pnc_map/reference_line.h"

namespace xju {
namespace pnc_map {
class DataPool {
 public:
  void set_reference_lines(
      const std::list<xju::pnc_map::ReferenceLine>& ref_lines) {
    reference_lines_ = ref_lines;
  }

  std::list<xju::pnc_map::ReferenceLine>& reference_lines() {
    return reference_lines_;
  }

  void clear() { reference_lines_.clear(); }

 private:
  std::list<xju::pnc_map::ReferenceLine> reference_lines_;

 private:
  DataPool() {}
  DECLARE_SINGLETON(DataPool)
};

} // namespace pnc_map
} // namespace xju
