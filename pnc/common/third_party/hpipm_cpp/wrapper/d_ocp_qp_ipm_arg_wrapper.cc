/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_ipm_arg_wrapper.h"

namespace xju {
namespace pnc {
namespace hpipm {

DOcpQpIpmArgWrapper::DOcpQpIpmArgWrapper() 
  : ocp_qp_ipm_arg_hpipm_(),
    memory_(nullptr),
    memsize_(0) {
  resize();
}

DOcpQpIpmArgWrapper::~DOcpQpIpmArgWrapper() {
  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
}

DOcpQpIpmArgWrapper::DOcpQpIpmArgWrapper(const DOcpQpIpmArgWrapper& other)
  : DOcpQpIpmArgWrapper() {
  copy(other);
}

DOcpQpIpmArgWrapper& DOcpQpIpmArgWrapper::operator=(const DOcpQpIpmArgWrapper& other) {
  if (this != &other) {
    copy(other);
  }
  return *this;
}

DOcpQpIpmArgWrapper::DOcpQpIpmArgWrapper(DOcpQpIpmArgWrapper&& other) noexcept 
  : ocp_qp_ipm_arg_hpipm_(other.ocp_qp_ipm_arg_hpipm_),
    memory_(other.memory_),
    memsize_(other.memsize_) {
  memory_ = nullptr;
  memsize_ = 0;
}

DOcpQpIpmArgWrapper& DOcpQpIpmArgWrapper::operator=(DOcpQpIpmArgWrapper&& other) noexcept {
  if (this == &other) return *this;

  if (memory_) {
    free(memory_);
    memory_ = nullptr;
    memsize_ = 0;
  }
  ocp_qp_ipm_arg_hpipm_ = other.ocp_qp_ipm_arg_hpipm_;
  memory_ = other.memory_;
  memsize_ = other.memsize_;

  other.memory_ = nullptr;
  other.memsize_ = 0;
  return *this;
}

d_ocp_qp_ipm_arg* DOcpQpIpmArgWrapper::get() { 
  return &ocp_qp_ipm_arg_hpipm_; 
}

const d_ocp_qp_ipm_arg* DOcpQpIpmArgWrapper::get() const { 
  return &ocp_qp_ipm_arg_hpipm_; 
}

void DOcpQpIpmArgWrapper::resize(const std::shared_ptr<DOcpQpDimWrapper>& dim) {
  const hpipm_size_t new_memsize = d_ocp_qp_ipm_arg_memsize(dim->get());
  if (memory_ && new_memsize > memsize_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = std::max(memsize_, new_memsize);
  if (!memory_) {
    memory_ = malloc(memsize_);
    d_ocp_qp_ipm_arg_create(dim->get(), &ocp_qp_ipm_arg_hpipm_, memory_);
  }
}

void DOcpQpIpmArgWrapper::resize() {
  d_ocp_qp_dim ocp_qp_dim_hpipm; // this does nothing in the below
  const hpipm_size_t new_memsize = d_ocp_qp_ipm_arg_memsize(&ocp_qp_dim_hpipm);
  if (memory_ && new_memsize > memsize_) {
    free(memory_);
    memory_ = nullptr;
  }
  memsize_ = std::max(memsize_, new_memsize);
  if (!memory_) {
    memory_ = malloc(memsize_);
    d_ocp_qp_ipm_arg_create(&ocp_qp_dim_hpipm, &ocp_qp_ipm_arg_hpipm_, memory_);
  }
}

void DOcpQpIpmArgWrapper::copy(const DOcpQpIpmArgWrapper& other) {
  const d_ocp_qp_ipm_arg* other_ptr = other.get();
  ocp_qp_ipm_arg_hpipm_.mu0 = other_ptr->mu0;
  ocp_qp_ipm_arg_hpipm_.alpha_min = other_ptr->alpha_min;
  ocp_qp_ipm_arg_hpipm_.res_g_max = other_ptr->res_g_max;
  ocp_qp_ipm_arg_hpipm_.res_b_max = other_ptr->res_b_max;
  ocp_qp_ipm_arg_hpipm_.res_d_max = other_ptr->res_d_max;
  ocp_qp_ipm_arg_hpipm_.res_m_max = other_ptr->res_m_max;
  ocp_qp_ipm_arg_hpipm_.reg_prim = other_ptr->reg_prim;
  ocp_qp_ipm_arg_hpipm_.lam_min = other_ptr->lam_min;
  ocp_qp_ipm_arg_hpipm_.t_min = other_ptr->t_min;
  ocp_qp_ipm_arg_hpipm_.tau_min = other_ptr->tau_min;
  ocp_qp_ipm_arg_hpipm_.iter_max = other_ptr->iter_max;
  ocp_qp_ipm_arg_hpipm_.stat_max = other_ptr->stat_max;
  ocp_qp_ipm_arg_hpipm_.pred_corr = other_ptr->pred_corr;
  ocp_qp_ipm_arg_hpipm_.cond_pred_corr = other_ptr->cond_pred_corr;
  ocp_qp_ipm_arg_hpipm_.itref_pred_max = other_ptr->itref_pred_max;
  ocp_qp_ipm_arg_hpipm_.itref_corr_max = other_ptr->itref_corr_max;
  ocp_qp_ipm_arg_hpipm_.warm_start = other_ptr->warm_start;
  ocp_qp_ipm_arg_hpipm_.square_root_alg = other_ptr->square_root_alg;
  ocp_qp_ipm_arg_hpipm_.lq_fact = other_ptr->lq_fact;
  ocp_qp_ipm_arg_hpipm_.abs_form = other_ptr->abs_form;
  ocp_qp_ipm_arg_hpipm_.comp_dual_sol_eq = other_ptr->comp_dual_sol_eq;
  ocp_qp_ipm_arg_hpipm_.comp_res_exit = other_ptr->comp_res_exit;
  ocp_qp_ipm_arg_hpipm_.comp_res_pred = other_ptr->comp_res_pred;
  ocp_qp_ipm_arg_hpipm_.split_step = other_ptr->split_step;
  ocp_qp_ipm_arg_hpipm_.var_init_scheme = other_ptr->var_init_scheme;
  ocp_qp_ipm_arg_hpipm_.t_lam_min = other_ptr->t_lam_min;
}

} // namespace hpipm
} // namespace pnc
} // namespace xju