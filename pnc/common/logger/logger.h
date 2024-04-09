/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "common/gflags/global_gflags.h"

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// LOG_IF
#define ACHECK(cond) CHECK(cond)
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AWARN_IF(cond) LOG_IF(WARNING, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)
#define AFATAL_IF(cond) LOG_IF(FATAL, cond)

// LOG_EVERY_N
#define AINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define AWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define AERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }

#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }

#define RETURN_IF(con)               \
  if (con) {                         \
    AWARN << #con << " is met.";     \
    return;                          \
  }

#define RETURN_VAL_IF(con, val)      \
  if (con) {                         \
    AWARN << #con << " is met.";     \
    return val;                      \
  }

#define AWARN_TOPIC_TIMEOUT(topic, duration)           \
  (LOG(WARNING) << "Topic: " << topic << ", timeout: " \
  << druration)
