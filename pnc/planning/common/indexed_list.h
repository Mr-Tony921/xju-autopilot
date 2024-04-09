/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <unordered_map>
#include <vector>
#include <boost/thread/shared_mutex.hpp>

#include "common/logger/logger.h"

namespace xju {
namespace planning {

template <typename I, typename T>
class IndexedList {
 public:
  T* Add(const I id, const T& object) {
    auto obs = Find(id);
    if (obs) {
      AWARN << "object " << id << " is already in container";
      *obs = object;
      return obs;
    } else {
      object_dict_.insert({id, object});
      auto* ptr = &object_dict_.at(id);
      object_list_.push_back(ptr);
      return ptr;
    }
  }

  T* Find(const I id) {
    auto it = object_dict_.find(id);
    if (it == object_dict_.end()) {
        return 0;
    }
    return &it->second;
  }

  const T* Find(const I id) const {
    auto it = object_dict_.find(id);
    if (it == object_dict_.end()) {
        return 0;
    }
    return &it->second;
  }

  const std::vector<const T*>& Items() const { return object_list_; }

  const std::unordered_map<I, T>& Dict() const { return object_dict_; }

  IndexedList& operator=(const IndexedList& other) {
    this->object_list_.clear();
    this->object_dict_.clear();
    for (const auto& item : other.Dict()) {
      Add(item.first, item.second);
    }
    return *this;
  }
  
  IndexedList() = default;
  
  IndexedList(const IndexedList& other) {
    this->object_list_.clear();
    this->object_dict_.clear();
    for (const auto& item : other.Dict()) {
      Add(item.first, item.second);
    }
  }

 private:
  std::vector<const T*> object_list_;
  std::unordered_map<I, T> object_dict_;
};

template <typename I, typename T>
class ThreadSafeIndexedList : public IndexedList<I, T> {
 public:
  T* Add(const I id, const T& object) {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    return IndexedList<I, T>::Add(id, object);
  }

  T* Find(const I id) {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    return IndexedList<I, T>::Find(id);
  }

  std::vector<const T*> Items() const {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    return IndexedList<I, T>::Items();
  }

 private:
  mutable boost::shared_mutex mutex_;
};

}  // namespace planning
}  // namespace xju
