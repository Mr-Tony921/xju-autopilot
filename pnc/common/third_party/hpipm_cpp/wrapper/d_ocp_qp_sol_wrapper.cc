/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_sol_wrapper.h"

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

DOcpQpSolWrapper::DOcpQpSolWrapper(
    const std::shared_ptr<DOcpQpDimWrapper>& dim) 
  : DOcpQpSolWrapper() {
  resize(dim);
}

DOcpQpSolWrapper::DOcpQpSolWrapper() 
  : dim_(),
    ocp_qp_sol_hpipm_(),
    memory_(nullptr),
    memsize_(0) {
}

DOcpQpSolWrapper::~DOcpQpSolWrapper() {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
}

DOcpQpSolWrapper::DOcpQpSolWrapper(const DOcpQpSolWrapper& other) 
  : DOcpQpSolWrapper() {
  copy(other);
}

DOcpQpSolWrapper& DOcpQpSolWrapper::operator=(const DOcpQpSolWrapper& other) {
  if (this != &other) {
    copy(other);
  }
  return *this;
}

DOcpQpSolWrapper::DOcpQpSolWrapper(DOcpQpSolWrapper&& other) noexcept 
  : dim_(std::move(other.dim_)),
    ocp_qp_sol_hpipm_(other.ocp_qp_sol_hpipm_),
    memory_(other.memory_),
    memsize_(other.memsize_) {
  other.ocp_qp_sol_hpipm_.dim  = nullptr;
  other.ocp_qp_sol_hpipm_.ux   = nullptr;
  other.ocp_qp_sol_hpipm_.pi   = nullptr;
  other.ocp_qp_sol_hpipm_.lam  = nullptr;
  other.ocp_qp_sol_hpipm_.t    = nullptr;
  other.ocp_qp_sol_hpipm_.misc = nullptr;
  other.ocp_qp_sol_hpipm_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
}

DOcpQpSolWrapper& DOcpQpSolWrapper::operator=(DOcpQpSolWrapper&& other) noexcept {
  if (this == &other) return *this;

  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
  dim_ = std::move(other.dim_);
  ocp_qp_sol_hpipm_ = other.ocp_qp_sol_hpipm_;
  memory_ = other.memory_;
  memsize_ = other.memsize_;

  other.ocp_qp_sol_hpipm_.dim  = nullptr;
  other.ocp_qp_sol_hpipm_.ux   = nullptr;
  other.ocp_qp_sol_hpipm_.pi   = nullptr;
  other.ocp_qp_sol_hpipm_.lam  = nullptr;
  other.ocp_qp_sol_hpipm_.t    = nullptr;
  other.ocp_qp_sol_hpipm_.misc = nullptr;
  other.ocp_qp_sol_hpipm_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
  return *this;
}

d_ocp_qp_sol* DOcpQpSolWrapper::get() { 
  ACHECK(memory_ != nullptr) << "[DOcpQpSolWrapper] hpipm memory is not created. "
                             << "Call resize() first.";
  return &ocp_qp_sol_hpipm_; 
}

const d_ocp_qp_sol* DOcpQpSolWrapper::get() const { 
  ACHECK(memory_ != nullptr) << "[DOcpQpSolWrapper] hpipm memory is not created. "
                             << "Call resize() first.";
  return &ocp_qp_sol_hpipm_; 
}

void DOcpQpSolWrapper::resize(const std::shared_ptr<DOcpQpDimWrapper>& dim) {
  dim_ = dim;
  const hpipm_size_t new_memsize = d_ocp_qp_sol_memsize(dim_->get());
  if (memory_ != nullptr && new_memsize > memsize_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = std::max(memsize_, new_memsize);
  if (memory_ == nullptr) {
    memory_ = malloc(memsize_);
  }
  d_ocp_qp_sol_create(dim_->get(), &ocp_qp_sol_hpipm_, memory_);
}

void DOcpQpSolWrapper::copy(const DOcpQpSolWrapper& other) {
  resize(other.dim_);
  d_ocp_qp_sol_copy_all(const_cast<d_ocp_qp_sol*>(other.get()), &ocp_qp_sol_hpipm_); 
}

} // namespace hpipm
} // namespace pnc
} // namespace xju