/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_wrapper.h"

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

DOcpQPWrapper::DOcpQPWrapper(const std::shared_ptr<DOcpQpDimWrapper>& dim) 
  : DOcpQPWrapper() {
  resize(dim);
}

DOcpQPWrapper::DOcpQPWrapper() 
  : dim_(),
    ocp_qp_hpipm_(),
    memory_(nullptr),
    memsize_(0) {}

DOcpQPWrapper::~DOcpQPWrapper() {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
}

DOcpQPWrapper::DOcpQPWrapper(const DOcpQPWrapper& other) 
  : DOcpQPWrapper() {
  copy(other);
}

DOcpQPWrapper& DOcpQPWrapper::operator=(const DOcpQPWrapper& other) {
  if (this != &other) {
    copy(other);
  }
  return *this;
}

DOcpQPWrapper::DOcpQPWrapper(DOcpQPWrapper&& other) noexcept
  : dim_(std::move(other.dim_)),
    ocp_qp_hpipm_(other.ocp_qp_hpipm_),
    memory_(other.memory_),
    memsize_(other.memsize_) {
  other.ocp_qp_hpipm_.dim = nullptr;
  other.ocp_qp_hpipm_.BAbt = nullptr;
  other.ocp_qp_hpipm_.RSQrq = nullptr;
  other.ocp_qp_hpipm_.DCt = nullptr;
  other.ocp_qp_hpipm_.b = nullptr;
  other.ocp_qp_hpipm_.rqz = nullptr;
  other.ocp_qp_hpipm_.d = nullptr;
  other.ocp_qp_hpipm_.d_mask = nullptr;
  other.ocp_qp_hpipm_.m = nullptr;
  other.ocp_qp_hpipm_.Z = nullptr;
  other.ocp_qp_hpipm_.idxb = nullptr;
  other.ocp_qp_hpipm_.idxs_rev = nullptr;
  other.ocp_qp_hpipm_.idxe = nullptr;
  other.ocp_qp_hpipm_.diag_H_flag = nullptr;
  other.ocp_qp_hpipm_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
}

DOcpQPWrapper& DOcpQPWrapper::operator=(DOcpQPWrapper&& other) noexcept {
  if (this == &other) return *this;

  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
  dim_ = std::move(other.dim_);
  ocp_qp_hpipm_ = other.ocp_qp_hpipm_;
  memory_ = other.memory_;
  memsize_ = other.memsize_;

  other.ocp_qp_hpipm_.dim = nullptr;
  other.ocp_qp_hpipm_.BAbt = nullptr;
  other.ocp_qp_hpipm_.RSQrq = nullptr;
  other.ocp_qp_hpipm_.DCt = nullptr;
  other.ocp_qp_hpipm_.b = nullptr;
  other.ocp_qp_hpipm_.rqz = nullptr;
  other.ocp_qp_hpipm_.d = nullptr;
  other.ocp_qp_hpipm_.d_mask = nullptr;
  other.ocp_qp_hpipm_.m = nullptr;
  other.ocp_qp_hpipm_.Z = nullptr;
  other.ocp_qp_hpipm_.idxb = nullptr;
  other.ocp_qp_hpipm_.idxs_rev = nullptr;
  other.ocp_qp_hpipm_.idxe = nullptr;
  other.ocp_qp_hpipm_.diag_H_flag = nullptr;
  other.ocp_qp_hpipm_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
  return *this;
}

d_ocp_qp* DOcpQPWrapper::get() { 
  ACHECK(memory_ != nullptr) << "[DOcpQPWrapper] hpipm memory is not created. "
                             << "Call resize() first.";
  return &ocp_qp_hpipm_; 
}

const d_ocp_qp* DOcpQPWrapper::get() const { 
  ACHECK(memory_ != nullptr) << "[DOcpQPWrapper] hpipm memory is not created. "
                             << "Call resize() first.";
  return &ocp_qp_hpipm_; 
}

void DOcpQPWrapper::resize(const std::shared_ptr<DOcpQpDimWrapper>& dim) {
  dim_ = dim;
  const hpipm_size_t new_memsize = d_ocp_qp_memsize(dim_->get());
  if (memory_ != nullptr && new_memsize > memsize_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = std::max(memsize_, new_memsize);
  if (memory_ == nullptr) {
    memory_ = malloc(memsize_);
  }
  d_ocp_qp_create(dim_->get(), &ocp_qp_hpipm_, memory_);
}

void DOcpQPWrapper::copy(const DOcpQPWrapper& other) {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = 0;
  resize(other.dim_);

  d_ocp_qp_copy_all(const_cast<d_ocp_qp*>(other.get()), &ocp_qp_hpipm_); 
}

} // namespace hpipm
} // namespace pnc
} // namespace xju