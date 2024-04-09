/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_ARG_WRAPPWE_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_ARG_WRAPPWE_H_

#include <memory>

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_dim_wrapper.h"
#include "Eigen/Core"

extern "C" {
#include "hpipm_d_ocp_qp_ipm.h"
}

namespace xju {
namespace pnc {
namespace hpipm {

class DOcpQpIpmArgWrapper {
 public:
  /// @brief Default constructor. Allocates the hpipm resource.
  DOcpQpIpmArgWrapper();
  
  /// @brief Destructor.
  ~DOcpQpIpmArgWrapper();
  
  /// @brief Custom copy constructor.
  DOcpQpIpmArgWrapper(const DOcpQpIpmArgWrapper&);
  
  /// @brief Custom copy assign operator.
  DOcpQpIpmArgWrapper& operator=(const DOcpQpIpmArgWrapper&);
  
  /// @brief Custom move constructor.
  DOcpQpIpmArgWrapper(DOcpQpIpmArgWrapper&&) noexcept;

  /// @brief Custom move assign operator.
  DOcpQpIpmArgWrapper& operator=(DOcpQpIpmArgWrapper&&) noexcept;

  /// @brief Gets the pointer to the hpipm resource. 
  /// @return Pointer to the hpipm resource.
  d_ocp_qp_ipm_arg* get();

  /// @brief Gets the const pointer to the hpipm instance.
  /// @return const pointer to the hpipm resource.
  const d_ocp_qp_ipm_arg* get() const;

  void resize(const std::shared_ptr<DOcpQpDimWrapper>& dim);
 
 private:
  void copy(const DOcpQpIpmArgWrapper& other);

  void resize();

 private:
  d_ocp_qp_ipm_arg ocp_qp_ipm_arg_hpipm_;
  void* memory_ = nullptr;
  hpipm_size_t memsize_ = 0;

};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_ARG_WRAPPWE_H_