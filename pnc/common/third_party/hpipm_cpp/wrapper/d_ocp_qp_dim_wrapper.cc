/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_dim_wrapper.h"

#include <string>

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

DOcpQpDimWrapper::DOcpQpDimWrapper(const unsigned int N)
    : DOcpQpDimWrapper() {
  if (N > 0) {
    resize(N);
  }
}

DOcpQpDimWrapper::DOcpQpDimWrapper()
    : d_ocp_qp_dim_()
    , memory_(nullptr)
    , memsize_(0) {}

DOcpQpDimWrapper::~DOcpQpDimWrapper() {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
}

DOcpQpDimWrapper::DOcpQpDimWrapper(const DOcpQpDimWrapper& other)
    : d_ocp_qp_dim_() {
  copy(other);
}

DOcpQpDimWrapper& DOcpQpDimWrapper::operator=(const DOcpQpDimWrapper& other) {
  if (this != &other) {
    copy(other);
  }
  return *this;
}

DOcpQpDimWrapper::DOcpQpDimWrapper(DOcpQpDimWrapper&& other) noexcept 
    : d_ocp_qp_dim_(other.d_ocp_qp_dim_)
    , memory_(other.memory_)
    , memsize_(other.memsize_) {
  other.d_ocp_qp_dim_.nx = nullptr;
  other.d_ocp_qp_dim_.nu = nullptr;
  other.d_ocp_qp_dim_.nb = nullptr;
  other.d_ocp_qp_dim_.nbx = nullptr;
  other.d_ocp_qp_dim_.nbu = nullptr;
  other.d_ocp_qp_dim_.ng = nullptr;
  other.d_ocp_qp_dim_.ns = nullptr;
  other.d_ocp_qp_dim_.nsbx = nullptr;
  other.d_ocp_qp_dim_.nsbu = nullptr;
  other.d_ocp_qp_dim_.nsg = nullptr;
  other.d_ocp_qp_dim_.nbxe = nullptr;
  other.d_ocp_qp_dim_.nbue = nullptr;
  other.d_ocp_qp_dim_.nge = nullptr;
  other.d_ocp_qp_dim_.N = 0;
  other.d_ocp_qp_dim_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
}

DOcpQpDimWrapper& DOcpQpDimWrapper::operator=(DOcpQpDimWrapper&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
  d_ocp_qp_dim_ = other.d_ocp_qp_dim_;
  memory_ = other.memory_;
  memsize_ = other.memsize_;

  other.d_ocp_qp_dim_.nx = nullptr;
  other.d_ocp_qp_dim_.nu = nullptr;
  other.d_ocp_qp_dim_.nb = nullptr;
  other.d_ocp_qp_dim_.nbx = nullptr;
  other.d_ocp_qp_dim_.nbu = nullptr;
  other.d_ocp_qp_dim_.ng = nullptr;
  other.d_ocp_qp_dim_.ns = nullptr;
  other.d_ocp_qp_dim_.nsbx = nullptr;
  other.d_ocp_qp_dim_.nsbu = nullptr;
  other.d_ocp_qp_dim_.nsg = nullptr;
  other.d_ocp_qp_dim_.nbxe = nullptr;
  other.d_ocp_qp_dim_.nbue = nullptr;
  other.d_ocp_qp_dim_.nge = nullptr;
  other.d_ocp_qp_dim_.N = 0;
  other.d_ocp_qp_dim_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
  return *this;
}

d_ocp_qp_dim* DOcpQpDimWrapper::get() {
  ACHECK(memory_) << "[DOcpQpDimWrapper] hpipm memory is not created. "
                  << "Call resize() first.";
  return &d_ocp_qp_dim_;
}

const d_ocp_qp_dim* DOcpQpDimWrapper::get() const {
  ACHECK(memory_) << "[DOcpQpDimWrapper] hpipm memory is not created. "
                  << "Call resize() first.";
  return &d_ocp_qp_dim_;
}

void DOcpQpDimWrapper::resize(const unsigned int N) {
  const hpipm_size_t new_memsize = d_ocp_qp_dim_memsize(N);
  if (memory_ != nullptr && new_memsize > memsize_) {
    free(memory_);
    memory_ = nullptr;
  }

  memsize_ = std::max(memsize_, new_memsize);
  if (memory_ == nullptr) {
    memory_ = malloc(memsize_);
  }
  if (d_ocp_qp_dim_.N != N) {
    d_ocp_qp_dim_create(N, &d_ocp_qp_dim_, memory_);
  }
}

void DOcpQpDimWrapper::copy(const DOcpQpDimWrapper& other) {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = 0;
  resize(static_cast<unsigned int>(other.get()->N));

  d_ocp_qp_dim_copy_all(const_cast<d_ocp_qp_dim*>(other.get()), &d_ocp_qp_dim_); 
}

} // namespace hpipm
} // namespace pnc
} // namespace xju