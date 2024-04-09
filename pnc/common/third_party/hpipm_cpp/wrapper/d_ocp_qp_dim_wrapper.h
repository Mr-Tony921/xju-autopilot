/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_WRAPPWE_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_WRAPPWE_H_

extern "C" {
#include "hpipm_d_ocp_qp_dim.h"
}

namespace xju {
namespace pnc {
namespace hpipm {

/// @class DOcpQpDimWrapper
/// @brief A Wrapper of d_ocp_qp_dim with a memory management
class DOcpQpDimWrapper {
 public:
  /// @brief Constructor. Allocates the hpipm resource.
  /// @param[in] N length of the horizon. 
  DOcpQpDimWrapper(const unsigned int N);

  /// @brief Default constructor. Does not allocate the hpipm resource.
  DOcpQpDimWrapper();

  /// @brief Destructor.
  ~DOcpQpDimWrapper();

  /// @brief Custom copy constructor.
  DOcpQpDimWrapper(const DOcpQpDimWrapper&);

  /// @brief Custom copy assign operator.
  DOcpQpDimWrapper& operator=(const DOcpQpDimWrapper&);

  /// @brief Custom move constructor.
  DOcpQpDimWrapper(DOcpQpDimWrapper&&) noexcept;

  /// @brief Custom move assign operator.
  DOcpQpDimWrapper& operator=(DOcpQpDimWrapper&&) noexcept;
  
  /// @brief Gets the pointer to the hpipm resource. Throw an exception if the 
  /// memory for the instance is not allocated.
  /// @return Pointer to the hpipm resource.
  d_ocp_qp_dim* get();

  /// @brief Gets the const pointer to the hpipm resource.
  /// @return const pointer to the hpipm resource.
  const d_ocp_qp_dim* get() const;
  
  /// @brief Resizes the hpipm resource.
  /// @param[in] N length of the horizon. 
  void resize(const unsigned int N);

 private:
  void copy(const DOcpQpDimWrapper& other);

 public:
  d_ocp_qp_dim d_ocp_qp_dim_;
  void* memory_ = nullptr;
  hpipm_size_t memsize_ = 0;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_WRAPPER_D_OCP_QP_DIM_WRAPPWE_H_