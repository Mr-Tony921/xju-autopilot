/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/speed_optimizer/piecewise_jerk_speed_optimizer.h"

#include <fstream>
#include <memory>
#include <vector>

#include "common/logger/logger.h"
#include "gtest/gtest.h"
#include "planning_task_config.pb.h"

namespace xju {
namespace planning {
class PiecewiseJerkSpeedOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string dir = getenv("HOME");
    dir += "/speed_optimizer_log/ref_speed_data.csv";
    std::ifstream csv_data(dir.c_str(), std::ios::in);
    std::vector<std::string> words;
    std::string line;

    std::getline(csv_data, line);

    std::getline(csv_data, line);
    std::stringstream ss(line);
    std::string str;
    std::getline(ss, str, ',');
    num_of_knots_ = std::stoi(str);
    std::getline(ss, str, ',');
    s_init_[0] = std::stod(str);
    std::getline(ss, str, ',');
    s_init_[1] = std::stod(str);
    std::getline(ss, str, ',');
    s_init_[2] = std::stod(str);

    std::getline(csv_data, line);

    while (std::getline(csv_data, line)) {
      std::stringstream data_ss(line);
      // ref_s
      std::getline(data_ss, str, ',');
      ref_s_.push_back(std::stod(str));
      // ref_v
      std::getline(data_ss, str, ',');
      ref_ds_.push_back(std::stod(str));
      // ref_a
      std::getline(data_ss, str, ',');
      ref_dds_.push_back(std::stod(str));
      // ref_k
      std::getline(data_ss, str, ',');
      ref_kappa_.push_back(std::stod(str));
      // s_lb
      std::getline(data_ss, str, ',');
      double s_lb = std::stod(str);
      // s_ub
      std::getline(data_ss, str, ',');
      double s_ub = std::stod(str);
      s_bounds_.emplace_back(s_lb, s_ub);
      // v_lb
      std::getline(data_ss, str, ',');
      double v_lb = std::stod(str);
      // v_ub
      std::getline(data_ss, str, ',');
      double v_ub = std::stod(str);
      ds_bounds_.emplace_back(v_lb, v_ub);
    }

    piecewise_jerk_speed_optimizer_ =
        std::make_shared<PiecewiseJerkSpeedOptimizer>(num_of_knots_, delta_t_,
                                                      s_init_);
    SetStartCondition();
  }

  void SetStartCondition() {
    // weight
    SpeedOptimizerWeights config;
    config.set_dds(1.0);
    config.set_ddds(400.0);
    config.set_a_c(1000.0);
    config.set_s_ref(5.0);
    config.set_ds_ref(1.0);
    config.set_dds_slack(100.0);
    config.set_ddds_slack(1000.0);
    piecewise_jerk_speed_optimizer_->set_weight(config);

    // s_bounds
    piecewise_jerk_speed_optimizer_->set_s_bounds(s_bounds_);

    // ds_bounds
    piecewise_jerk_speed_optimizer_->set_ds_bounds(ds_bounds_);

    // dds_bounds
    piecewise_jerk_speed_optimizer_->set_dds_bounds(-3.0, 2.0);

    // ddds_bounds
    piecewise_jerk_speed_optimizer_->set_ddds_bounds(-2.0, 1.0);

    // slack_bounds
    piecewise_jerk_speed_optimizer_->set_dds_slack_bounds(-3.0, 2.0);
    piecewise_jerk_speed_optimizer_->set_ddds_slack_bounds(-2.0, 1.0);

    // s_ref   // ds_ref  // kappa_ref  // start_s
    piecewise_jerk_speed_optimizer_->set_s_ref(ref_s_);
    piecewise_jerk_speed_optimizer_->set_ds_ref(ref_ds_);
    piecewise_jerk_speed_optimizer_->set_kappa_ref(ref_kappa_);
    std::vector<double> start_s(5 * num_of_knots_ - 1, 0.0);
    for (int i = 0; i < num_of_knots_; i++) {
      start_s[i] = ref_s_[i];
      start_s[num_of_knots_ + i] = ref_ds_[i];
      start_s[2 * num_of_knots_ + i] = ref_dds_[i];
    }

    piecewise_jerk_speed_optimizer_->set_warm_start_val(start_s);
  }

 protected:
  int num_of_knots_;
  double delta_t_ = 0.1;
  std::array<double, 3> s_init_;
  std::vector<double> ref_s_, ref_ds_, ref_dds_, ref_kappa_;
  std::vector<std::pair<double, double>> s_bounds_, ds_bounds_;
  std::shared_ptr<PiecewiseJerkSpeedOptimizer> piecewise_jerk_speed_optimizer_;
};

TEST_F(PiecewiseJerkSpeedOptimizerTest, Optimize) {
  std::vector<double> opt_s;
  std::vector<double> opt_ds;
  std::vector<double> opt_dds;
  bool res =
      piecewise_jerk_speed_optimizer_->Optimize(&opt_s, &opt_ds, &opt_dds);
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace xju