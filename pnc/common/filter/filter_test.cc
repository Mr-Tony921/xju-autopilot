/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/filter/digital_filter.h"
#include "common/filter/first_order_lowpass_filter.h"
#include "common/filter/second_order_lowpass_filter.h"

#include "gtest/gtest.h"

namespace xju {
namespace pnc {

class DigitalFilterTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(DigitalFilterTest, SetGet) {
  DigitalFilter digital_filter;
  std::vector<double> numerators = {1.0, 2.0, 3.0};
  std::vector<double> denominators = {4.0, 5.0, 6.0};
  digital_filter.set_denominators(denominators);
  digital_filter.set_numerators(numerators);
  std::vector<double> denominators_got = digital_filter.denominators();
  std::vector<double> numerators_got = digital_filter.numerators();
  EXPECT_EQ(numerators_got.size(), numerators.size());
  EXPECT_EQ(denominators_got.size(), denominators.size());
  for (size_t i = 0; i < numerators.size(); ++i) {
    EXPECT_DOUBLE_EQ(numerators_got[i], numerators[i]);
  }
  for (size_t i = 0; i < denominators.size(); ++i) {
    EXPECT_DOUBLE_EQ(denominators_got[i], denominators[i]);
  }
  digital_filter.set_coefficients(denominators, numerators);
  denominators_got.clear();
  denominators_got = digital_filter.denominators();
  numerators_got.clear();
  numerators_got = digital_filter.numerators();
  EXPECT_EQ(numerators_got.size(), numerators.size());
  EXPECT_EQ(denominators_got.size(), denominators.size());
  for (size_t i = 0; i < numerators.size(); ++i) {
    EXPECT_DOUBLE_EQ(numerators_got[i], numerators[i]);
  }
  for (size_t i = 0; i < denominators.size(); ++i) {
    EXPECT_DOUBLE_EQ(denominators_got[i], denominators[i]);
  }

  double dead_zone = 1.0;
  digital_filter.set_dead_zone(dead_zone);
  EXPECT_DOUBLE_EQ(digital_filter.dead_zone(), dead_zone);
}

TEST_F(DigitalFilterTest, FilterOff) {
  std::vector<double> numerators = {0.0, 0.0, 0.0};
  std::vector<double> denominators = {1.0, 0.0, 0.0};
  DigitalFilter digital_filter(denominators, numerators);

  const std::vector<double> step_input(100, 1.0);
  std::vector<double> rand_input(100, 1.0);

  unsigned int seed;
  for (size_t i = 0; i < rand_input.size(); ++i) {
    rand_input[i] = rand_r(&seed);
  }
  // Check setp input
  for (size_t i = 0; i < step_input.size(); ++i) {
    EXPECT_DOUBLE_EQ(digital_filter.Filter(step_input[i]), 0.0);
  }
  // Check random input
  for (size_t i = 0; i < rand_input.size(); ++i) {
    EXPECT_DOUBLE_EQ(digital_filter.Filter(rand_input[i]), 0.0);
  }
}

TEST_F(DigitalFilterTest, MovingAverage) {
  std::vector<double> numerators = {0.25, 0.25, 0.25, 0.25};
  std::vector<double> denominators = {1.0, 0.0, 0.0};
  DigitalFilter digital_filter;
  digital_filter.set_numerators(numerators);
  digital_filter.set_denominators(denominators);

  const std::vector<double> step_input(100, 1.0);
  // Check step input, steady state
  double value = 0.0;
  for (size_t i = 0; i < step_input.size(); ++i) {
    value += 0.25;
    value = std::fmin(value, 1.0);
    EXPECT_DOUBLE_EQ(digital_filter.Filter(step_input[i]), value);
  }
}

class FirstOrderLowpassFilterTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(FirstOrderLowpassFilterTest, Init) {
  double ts = 0.01;
  double cutoff_freq = 20;
  FirstOrderLowPassFilter filter(ts, cutoff_freq);
  std::vector<double> den = filter.denominators();
  std::vector<double> num = filter.numerators();
  double a_term = exp(-1 * 2 * M_PI * cutoff_freq * ts);
  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 2);
  EXPECT_NEAR(num[0], 0.0, 0.01);
  EXPECT_NEAR(num[1], 1 - a_term, 0.01);
  EXPECT_NEAR(den[0], 1.0, 0.01);
  EXPECT_NEAR(den[1], -a_term, 0.01);
}

TEST_F(FirstOrderLowpassFilterTest, InitTimeDomain) {
  double ts = 0.01;
  double settling_time = 0.005;
  double dead_time = 0.04;
  FirstOrderLowPassFilter filter;
  filter.InitTimeDomain(ts, settling_time, dead_time);
  std::vector<double> den = filter.denominators();
  std::vector<double> num = filter.numerators();

  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 5);
  EXPECT_NEAR(den[1], -0.13533, 0.01);
  EXPECT_DOUBLE_EQ(num[0], 0.0);
  EXPECT_DOUBLE_EQ(num[1], 0.0);
  EXPECT_NEAR(num[4], 1 - 0.13533, 0.01);

  dead_time = 0.0;
  filter.InitTimeDomain(ts, settling_time, dead_time);
  den = filter.denominators();
  num = filter.numerators();
  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 1);
  EXPECT_NEAR(den[1], -0.13533, 0.01);
  EXPECT_NEAR(num[0], 1 - 0.13533, 0.01);
}

TEST(SecondOrderLowPassFilter, Init) {
  double ts = 0.01;
  double cutoff_freq = 20;
  SecondOrderLowPassFilter filter = SecondOrderLowPassFilter(ts, cutoff_freq);
  std::vector<double> den = filter.denominators();
  std::vector<double> num = filter.numerators();
  EXPECT_EQ(den.size(), 3);
  EXPECT_EQ(num.size(), 3);
  EXPECT_NEAR(num[0], 0.1729, 0.01);
  EXPECT_NEAR(num[1], 0.3458, 0.01);
  EXPECT_NEAR(num[2], 0.1729, 0.01);
  EXPECT_NEAR(den[0], 1.0, 0.01);
  EXPECT_NEAR(den[2], 0.2217, 0.01);
}

} // namespace pnc
} // namespace xju