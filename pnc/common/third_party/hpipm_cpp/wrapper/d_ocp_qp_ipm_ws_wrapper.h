/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_WS_WRAPPWE_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_WS_WRAPPWE_H_

#include <memory>

extern "C" {
#include "hpipm_d_ocp_qp_ipm.h"
}

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_dim_wrapper.h"
#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_ipm_arg_wrapper.h"

namespace xju {
namespace pnc {
namespace hpipm {

class DOcpQpIpmWsWrapper {
 public:
  /// @brief Constructor. Allocates the hpipm resource.
  /// @param[in] dim Dimension.
  /// @param[in] ipm_arg Ipm solver argument.
  DOcpQpIpmWsWrapper(const std::shared_ptr<DOcpQpDimWrapper>& dim, 
                     const std::shared_ptr<DOcpQpIpmArgWrapper>& ipm_arg);

  /// @brief Default constructor. Does not allocate the hpipm resource.
  DOcpQpIpmWsWrapper();

  /// @brief Destructor.
  ~DOcpQpIpmWsWrapper();

  /// @brief Prohibit copy constructor.
  DOcpQpIpmWsWrapper(const DOcpQpIpmWsWrapper&) = delete;

  /// @brief Prohibit copy assign operator.
  DOcpQpIpmWsWrapper& operator=(const DOcpQpIpmWsWrapper&) = delete;

  /// @brief Custom move constructor.
  DOcpQpIpmWsWrapper(DOcpQpIpmWsWrapper&&) noexcept;

  /// @brief Custom move assign operator.
  DOcpQpIpmWsWrapper& operator=(DOcpQpIpmWsWrapper&&) noexcept;

  /// @brief Gets the pointer to the hpipm resource. Throw an exception if the 
  /// memory for the instance is not allocated.
  /// @return Pointer to the hpipm resource.
  d_ocp_qp_ipm_ws* get();

  /// @brief Gets the const pointer to the hpipm instance.
  /// @return const pointer to the hpipm resource.
  const d_ocp_qp_ipm_ws* get() const;

  /// @brief Resizes the hpipm resource.
  /// @param[in] dim Dimension.
  /// @param[in] ipm_arg Ipm solver argument.
  void resize(const std::shared_ptr<DOcpQpDimWrapper>& dim,
              const std::shared_ptr<DOcpQpIpmArgWrapper>& ipm_arg);

 private:
  std::shared_ptr<DOcpQpDimWrapper> dim_;
  std::shared_ptr<DOcpQpIpmArgWrapper> ipm_arg_;
  d_ocp_qp_ipm_ws ocp_qp_ipm_ws_hpipm_;
  void* memory_ = nullptr;
  hpipm_size_t memsize_ = 0;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_WS_WRAPPWE_H_