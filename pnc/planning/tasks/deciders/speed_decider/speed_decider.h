/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/math/polynomial_x_order.h"
#include "planning/tasks/task.h"
#include "planning_task_config.pb.h"

namespace xju {
namespace planning {

class SpeedDecider : public Task {
 public:
  SpeedDecider(const pnc::TaskConfig& config,
               const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
               Frame* const frame) override;

 private:
  enum STLocation {
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
  };
  STLocation GetSTLocation(
      const STBoundary& st_boundary,
      const std::shared_ptr<pnc::PolynomialXOrder>& final_poly_ptr);

  bool MakeObjectsDecision(
      const std::shared_ptr<pnc::PolynomialXOrder>& final_poly_ptr);

  bool GenerateLonPolynomial(
      std::shared_ptr<pnc::PolynomialXOrder> final_poly_ptr);

  bool UpdateSpeedData(
      const std::shared_ptr<pnc::PolynomialXOrder>& final_poly_ptr);
  
  void DrawDebugInfo();

 private:
  SpeedDeciderConfig speed_decider_config_;
};

}  // namespace planning
}  // namespace xju
