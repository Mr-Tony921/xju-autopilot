/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <limits>

#include "Eigen/Core"
#include "osqp/osqp.h"
#include "osqp/constants.h"

namespace xju {
namespace pnc {

class QPSolver {
 public:
  QPSolver() = delete;
  explicit QPSolver(const int max_iter);
  double ObjectiveValue() {
    return obj_val_;
  }
  int IterNum(){
    return iter_num_;
  }

 protected:
  virtual bool Optimize();
  
  std::vector<double> solution();
  
  virtual void CalculateKernelAndOffset(
      std::vector<c_float>* P_data,
      std::vector<c_int>* P_indices,
      std::vector<c_int>* P_indptr,
      std::vector<c_float>* q) = 0;

  virtual void CalculateAffineConstraint(
      std::vector<c_float>* A_data,
      std::vector<c_int>* A_indices,
      std::vector<c_int>* A_indptr,
      std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds) = 0;

  virtual OSQPSettings* SolverDefaultSetting();
  virtual void SetWarmStartX() = 0;

  OSQPData* FormulateProblem();
  void FreeData(OSQPData* const data);
  void WarmStart(OSQPWorkspace* const work);

  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

  void LogOsqpSolverStatus(const int status);

  std::string DebugString(const OSQPWorkspace *work);

 protected:
  double obj_val_ = std::numeric_limits<double>::infinity();
  int max_iter_;
  size_t kernel_dim_;
  std::vector<double> solution_;
  std::vector<c_float> start_x_;
  std::vector<c_float> start_y_;

  int iter_num_ = 0;
};

} // namespace pnc
} // namespace xju
