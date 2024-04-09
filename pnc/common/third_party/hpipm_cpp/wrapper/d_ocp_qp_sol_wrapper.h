/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_SOL_WRAPPWE_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_SOL_WRAPPWE_H_

#include <memory>

extern "C" {
#include "hpipm_d_ocp_qp_sol.h"
}

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_dim_wrapper.h"

namespace xju {
namespace pnc {
namespace hpipm {

/// @class DOcpQpSolWrapper
/// @brief A wrapper of d_ocp_qp_sol with a memory management.
class DOcpQpSolWrapper {
 public:
  /// @brief Constructor. Allocates the hpipm resource.
  /// @param[in] dim Dimension.
  DOcpQpSolWrapper(const std::shared_ptr<DOcpQpDimWrapper>& dim);

  /// @brief Default constructor. Does not allocate the hpipm resource.
  DOcpQpSolWrapper();

  /// @brief Destructor.
  ~DOcpQpSolWrapper();

  /// @brief Custom copy constructor.
  DOcpQpSolWrapper(const DOcpQpSolWrapper&);

  /// @brief Custom copy assign operator.
  DOcpQpSolWrapper& operator=(const DOcpQpSolWrapper&);
  
  /// @brief Custom move constructor.
  DOcpQpSolWrapper(DOcpQpSolWrapper&&) noexcept;

  /// @brief Custom move assign operator.
  DOcpQpSolWrapper& operator=(DOcpQpSolWrapper&&) noexcept;

  /// @brief Gets the pointer to the hpipm resource. Throw an exception if the 
  /// memory for the instance is not allocated.
  /// @return Pointer to the hpipm resource.
  d_ocp_qp_sol* get();

  /// @brief Gets the const pointer to the hpipm instance.
  /// @return const pointer to the hpipm resource.
  const d_ocp_qp_sol* get() const;

  /// @brief Resizes the hpipm resource.
  /// @param[in] dim Dimension.
  void resize(const std::shared_ptr<DOcpQpDimWrapper>& dim);

 private: 
  void copy(const DOcpQpSolWrapper& other);

 private:
  std::shared_ptr<DOcpQpDimWrapper> dim_;
  d_ocp_qp_sol ocp_qp_sol_hpipm_;
  void* memory_ = nullptr;
  hpipm_size_t memsize_ = 0;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_SOL_WRAPPWE_H_