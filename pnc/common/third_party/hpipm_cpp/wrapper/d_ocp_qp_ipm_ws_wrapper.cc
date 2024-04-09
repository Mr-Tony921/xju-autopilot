/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_ipm_ws_wrapper.h"

#include <string>

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

DOcpQpIpmWsWrapper::DOcpQpIpmWsWrapper(
    const std::shared_ptr<DOcpQpDimWrapper>& dim, 
    const std::shared_ptr<DOcpQpIpmArgWrapper>& ipm_arg) 
  : DOcpQpIpmWsWrapper() {
  resize(dim, ipm_arg);
}

DOcpQpIpmWsWrapper::DOcpQpIpmWsWrapper() 
  : dim_(),
    ipm_arg_(),
    ocp_qp_ipm_ws_hpipm_(),
    memory_(nullptr),
    memsize_(0) {}

DOcpQpIpmWsWrapper::~DOcpQpIpmWsWrapper() {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
}

DOcpQpIpmWsWrapper::DOcpQpIpmWsWrapper(DOcpQpIpmWsWrapper&& other) noexcept 
  : dim_(std::move(other.dim_)),
    ipm_arg_(std::move(other.ipm_arg_)),
    ocp_qp_ipm_ws_hpipm_(other.ocp_qp_ipm_ws_hpipm_),
    memory_(other.memory_),
    memsize_(other.memsize_) {
  other.ocp_qp_ipm_ws_hpipm_.core_workspace = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.dim = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.res_workspace = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.sol_step = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.sol_itref = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.qp_step = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.qp_itref = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.res_itref = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.res = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Gamma = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.gamma = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nuxM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nbgM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nsM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Pb = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Zs_inv = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_m = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.l = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.L = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Ls = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.P = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Lh = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.AL = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.lq0 = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nxM_nxM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.stat = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.use_hess_fact = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.lq_work0 = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
}

DOcpQpIpmWsWrapper& DOcpQpIpmWsWrapper::operator=(DOcpQpIpmWsWrapper&& other) noexcept {
  if (this == &other) return *this;

  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
  dim_ = std::move(other.dim_);
  ipm_arg_ = std::move(other.ipm_arg_);
  ocp_qp_ipm_ws_hpipm_ = other.ocp_qp_ipm_ws_hpipm_;
  memory_ = other.memory_;
  memsize_ = other.memsize_;

  other.ocp_qp_ipm_ws_hpipm_.core_workspace = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.dim = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.res_workspace = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.sol_step = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.sol_itref = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.qp_step = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.qp_itref = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.res_itref = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.res = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Gamma = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.gamma = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nuxM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nbgM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nsM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Pb = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Zs_inv = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_m = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.l = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.L = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Ls = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.P = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.Lh = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.AL = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.lq0 = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.tmp_nxM_nxM = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.stat = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.use_hess_fact = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.lq_work0 = nullptr;
  other.ocp_qp_ipm_ws_hpipm_.memsize = 0;
  other.memory_ = nullptr;
  other.memsize_ = 0;
  return *this;
}

d_ocp_qp_ipm_ws* DOcpQpIpmWsWrapper::get() { 
  ACHECK(memory_ != nullptr) << "[DOcpQpIpmWsWrapper] hpipm memory is not created. " 
                             << "Call resize() first.";
  return &ocp_qp_ipm_ws_hpipm_; 
}

const d_ocp_qp_ipm_ws* DOcpQpIpmWsWrapper::get() const { 
  ACHECK(memory_ != nullptr) << "[DOcpQpIpmWsWrapper] hpipm memory is not created. " 
                             << "Call resize() first.";
  return &ocp_qp_ipm_ws_hpipm_; 
}

void DOcpQpIpmWsWrapper::resize(
    const std::shared_ptr<DOcpQpDimWrapper>& dim,
    const std::shared_ptr<DOcpQpIpmArgWrapper>& ipm_arg) {
  dim_ = dim;
  ipm_arg_ = ipm_arg;
  const hpipm_size_t new_memsize = d_ocp_qp_ipm_ws_memsize(dim_->get(), ipm_arg_->get());
  if (memory_ != nullptr && new_memsize > memsize_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = std::max(memsize_, new_memsize);
  if (memory_ == nullptr) {
    memory_ = malloc(memsize_);
  }
  d_ocp_qp_ipm_ws_create(dim_->get(), ipm_arg_->get(), &ocp_qp_ipm_ws_hpipm_, memory_);
}

} // namespace hpipm
} // namespace pnc
} // namespace xju