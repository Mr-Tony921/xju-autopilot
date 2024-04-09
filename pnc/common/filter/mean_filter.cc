/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/filter/mean_filter.h"

namespace xju {
namespace pnc {

MeanFilter::MeanFilter(const std::uint_fast8_t window_size)
    : window_size_(window_size) {
  CHECK_GT(window_size_, 0);
  CHECK_LE(window_size_, std::numeric_limits<std::uint_fast8_t>::max() / 2);
  initialized_ = true;
}

double MeanFilter::Update(const double measurement) {
  ACHECK(initialized_);
  CHECK_LE(values_.size(), window_size_);
  CHECK_LE(min_candidates_.size(), window_size_);
  CHECK_LE(max_candidates_.size(), window_size_);
  ++time_;
  time_ %= static_cast<std::uint_fast8_t>(2 * window_size_);
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(measurement);
  if (values_.size() > 2) {
    return (sum_ - GetMin() - GetMax()) /
           static_cast<double>(values_.size() - 2);
  } else {
    return sum_ / static_cast<double>(values_.size());
  }
}

void MeanFilter::Clear() {
  sum_ = 0.0;
  values_.clear();
  min_candidates_.clear();
  max_candidates_.clear();
}

void MeanFilter::RemoveEarliest() {
  CHECK_EQ(values_.size(), window_size_);
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MeanFilter::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

double MeanFilter::GetMin() const {
  if (min_candidates_.empty()) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MeanFilter::GetMax() const {
  if (max_candidates_.empty()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

bool MeanFilter::ShouldPopOldestCandidate(const std::uint_fast8_t old_time) const {
  if (old_time < window_size_) {
    CHECK_LE(time_, old_time + window_size_);
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    CHECK_GE(old_time, time_ + window_size_);
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

}  // namespace pnc
}  // namespace xju
