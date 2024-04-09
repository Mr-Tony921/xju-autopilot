/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_WRAPPWE_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_WRAPPWE_H_

#include <memory>

extern "C" {
#include "hpipm_d_ocp_qp.h"
}

#include "common/third_party/hpipm_cpp/wrapper/d_ocp_qp_dim_wrapper.h"

namespace xju {
namespace pnc {
namespace hpipm {

/// @class DOcpQPWrapper
/// @brief A wrapper of d_ocp_qp with a memory management.
class DOcpQPWrapper {
 public:
  /// @brief Constructor. Allocates the hpipm resource.
  /// @param[in] dim Dimension.
  DOcpQPWrapper(const std::shared_ptr<DOcpQpDimWrapper>& dim);

  /// @brief Default constructor. Does not allocate the hpipm resource.
  DOcpQPWrapper();

  /// @brief Destructor.
  ~DOcpQPWrapper();

  /// @brief Custom copy constructor.
  DOcpQPWrapper(const DOcpQPWrapper&);

  /// @brief Custom copy assign operator.
  DOcpQPWrapper& operator=(const DOcpQPWrapper&);

  /// @brief Custom move constructor.
  DOcpQPWrapper(DOcpQPWrapper&&) noexcept;

  /// @brief Custom move assign operator.
  DOcpQPWrapper& operator=(DOcpQPWrapper&&) noexcept;

  /// @brief Gets the pointer to the hpipm resource. Throw an exception if the 
  /// memory for the instance is not allocated.
  /// @return Pointer to the hpipm resource.
  d_ocp_qp* get();

  /// @brief Gets the const pointer to the hpipm instance.
  /// @return const pointer to the hpipm resource.
  const d_ocp_qp* get() const;

  /// @brief Resizes the hpipm resource.
  /// @param[in] dim Dimension.
  void resize(const std::shared_ptr<DOcpQpDimWrapper>& dim);

 private:
  void copy(const DOcpQPWrapper& dim);

 private:
  std::shared_ptr<DOcpQpDimWrapper> dim_;
  d_ocp_qp ocp_qp_hpipm_;
  void* memory_ = nullptr;
  hpipm_size_t memsize_ = 0;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_WRAPPWE_H_