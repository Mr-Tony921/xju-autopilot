/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <mutex>
#include <cmath>

#include "common/logger/logger.h"

namespace xju {
namespace pnc {

template <class DataType>
class DataRingRepo {
 public:
  explicit DataRingRepo(const size_t max_size);
  ~DataRingRepo();
  
  void Clear();
  void InsertData(const double timestamp, const DataType& data);
  bool GetLastestData(DataType* const data);
  bool GetClosestData(
      const double timestamp, DataType* const data);

 private:
  void BubbleSort();
  size_t BinarySearch(
      const double timestamp, const size_t low, const size_t high);
 private:
  typedef std::shared_ptr<DataType> DataTypePtr;
  typedef std::pair<double, DataTypePtr> DataPair;

 private:
  size_t max_size_ = 0;
  std::mutex data_mutex_;
  DataPair* data_ = nullptr;

  size_t size_ = 0;
  int head_ = -1;
  bool is_full_ = false;
};

template <class DataType>
DataRingRepo<DataType>::DataRingRepo(const size_t max_size) {
  CHECK_GE(max_size, 0) << "DataRingRepo Initialization Error! "
                        << "max_size must be great than 0.";

  std::lock_guard<std::mutex> lock(data_mutex_);
  data_ = new DataPair[max_size];
  max_size_ = max_size;
}

template <class DataType>
DataRingRepo<DataType>::~DataRingRepo() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  delete[] data_;
  max_size_ = 0;
}

template <class DataType>
void DataRingRepo<DataType>::Clear() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  size_ = 0;
  head_ = -1;
  is_full_ = false;
}

template <class DataType>
void DataRingRepo<DataType>::InsertData(
    const double timestamp, const DataType& data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  ++head_;
  ++size_;
  size_ = std::min(size_, max_size_);

  if (!is_full_ && head_ >= static_cast<int>(max_size_ - 1)) {
    is_full_ = true;
  }

  head_ = head_ % max_size_;
  data_[head_].first = timestamp;
  data_[head_].second.reset(new DataType(data));

  // BubbleSort();
}

template <class DataType>
bool DataRingRepo<DataType>::GetLastestData(DataType* const data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (head_ < 0) {
    return false;
  }
  *data = *data_[head_].second;
  return true;
}

template <class DataType>
bool DataRingRepo<DataType>::GetClosestData(
    const double timestamp, DataType* const data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (head_ < 0) {
    return false;
  }

  size_t low = is_full_ ? (head_ + 1) % max_size_ : 0;
  size_t high = head_;

  if (data_[low].first > timestamp) {
    *data = *data_[low].second;
    return true;
  }
  if (data_[high].first < timestamp) {
    *data = *data_[high].second;
    return true;
  }

  int data_idx = BinarySearch(timestamp, low, high);
  *data = *data_[data_idx].second;
  return true;
}

template <class DataType>
void DataRingRepo<DataType>::BubbleSort() {
  bool is_bubble_move = true;
  size_t bubble_at = head_;
  size_t top_at = is_full_ ? (head_ + 1) % max_size_ : 0;

  while (is_bubble_move) {
    if (bubble_at == top_at) {
      break;
    }

    size_t comp_at = (bubble_at - 1) % max_size_;
    if (data_[bubble_at].first < data_[comp_at].first) {
      DataPair tmp = data_[bubble_at];
      data_[bubble_at] = data_[comp_at];
      data_[comp_at] = tmp;
      bubble_at = comp_at;
      is_bubble_move = true;
    } else {
      is_bubble_move = false;
    }
  }
}

template <class DataType>
size_t DataRingRepo<DataType>::BinarySearch(
    const double timestamp, const size_t low, const size_t high) {
  high = high < low ? high + max_size_ : high;
  if (low == high) {
    return low;
  }

  if (low == high - 1) {
    high = high % max_size_;
    return std::fabs(data_[low].first - timestamp) <= 
           std::fabs(data_[high].first - timestamp) ?
           low : high;
  }

  size_t mid = ((low + high) / 2) % max_size_;
  if (data_[mid].first > timestamp) {
    return BinarySearch(timestamp, low, mid);
  } else {
    return BinarySearch(timestamp, mid, high % max_size_);
  }
}

} // namespace pnc
} // namespace xju
