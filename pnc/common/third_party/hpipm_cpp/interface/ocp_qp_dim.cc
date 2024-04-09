/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/interface/ocp_qp_dim.h"

#include <string>

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

OcpQpDim::OcpQpDim(const unsigned int N) {
  resize(N);
}

OcpQpDim::OcpQpDim(const std::vector<OcpQp>& ocp_qp) {
  resize(ocp_qp);
}

void OcpQpDim::resize(const unsigned int _N) {
  N = _N;
  nx.resize(N + 1);
  nu.resize(N + 1);
  nbx.resize(N + 1);
  nbu.resize(N + 1);
  ng.resize(N + 1);
  nsbx.resize(N + 1);
  nsbu.resize(N + 1);
  nsg.resize(N + 1);
}

void OcpQpDim::resize(const std::vector<OcpQp>& ocp_qp) {
  ACHECK(!ocp_qp.empty()) << "Error! ocp_qp is empty.";

  resize(ocp_qp.size() - 1);
  for (unsigned int i = 0; i < N; ++i) {
    nx[i] = ocp_qp[i].q.size();
    nu[i] = ocp_qp[i].r.size();
    nbx[i] = ocp_qp[i].idxbx.size();
    nbu[i] = ocp_qp[i].idxbu.size();
    ng[i] = ocp_qp[i].lg.size();
    nsbx[i] = ocp_qp[i].idxs.size();
    nsbu[i] = 0;
    nsg[i] = ocp_qp[i].idxs.size();
  }
  nx[N] = ocp_qp[N].q.size();
  nu[N] = 0;
  nbx[N] = ocp_qp[N].idxbx.size();
  nbu[N] = 0;
  ng[N] = ocp_qp[N].lg.size();
  nsbx[N] = ocp_qp[N].idxs.size();
  nsbu[N] = 0;
  nsg[N] = ocp_qp[N].idxs.size();
  CheckSize(ocp_qp);
}

void OcpQpDim::CheckSize(const std::vector<OcpQp>& ocp_qp) const {
  // check dynamics
  for (unsigned int i = 0; i < N; ++i) {
    ACHECK(ocp_qp[i].A.rows() == nx[i + 1]) 
        << "ocp_qp[" << i << "].A.rows() must to be " << nx[i + 1];
    ACHECK(ocp_qp[i].A.cols() == nx[i]) 
        << "ocp_qp[" << i << "].A.cols() must to be " << nx[i];
    ACHECK(ocp_qp[i].B.rows() == nx[i + 1]) 
        << "ocp_qp[" << i << "].B.rows() must to be " << nx[i + 1];
    ACHECK(ocp_qp[i].B.cols() == nu[i]) 
        << "ocp_qp[" << i << "].B.cols() must to be " << nu[i];
    ACHECK(ocp_qp[i].b.size() == nx[i]) 
        << "ocp_qp[" << i << "].b.size() must to be " << nx[i];
  }
  // check cost
  for (unsigned int i = 0; i < N; ++i) {
    ACHECK(ocp_qp[i].Q.rows() == nx[i])
        << "ocp_qp[" << i << "].Q.rows() must to be " << nx[i];
    ACHECK(ocp_qp[i].Q.cols() == nx[i])
        << "ocp_qp[" << i << "].Q.cols() must to be " << nx[i];
    ACHECK(ocp_qp[i].S.rows() == nu[i])
        << "ocp_qp[" << i << "].S.rows() must to be " << nu[i];
    ACHECK(ocp_qp[i].S.cols() == nx[i])
        << "ocp_qp[" << i << "].S.cols() must to be " << nx[i];
    ACHECK(ocp_qp[i].R.rows() == nu[i])
        << "ocp_qp[" << i << "].R.rows() must to be " << nu[i];
    ACHECK(ocp_qp[i].R.cols() == nu[i])
        << "ocp_qp[" << i << "].R.cols() must to be " << nu[i];
    ACHECK(ocp_qp[i].q.size() == nx[i])
        << "ocp_qp[" << i << "].q.size() must to be " << nx[i];
    ACHECK(ocp_qp[i].r.size() == nu[i])
        << "ocp_qp[" << i << "].r.size() must to be " << nu[i];
  }
  ACHECK(ocp_qp[N].Q.rows() == nx[N])
      << "ocp_qp[" << N << "].Q.rows() must to be " << nx[N];
  ACHECK(ocp_qp[N].Q.cols() == nx[N])
      << "ocp_qp[" << N << "].Q.cols() must to be " << nx[N];
  ACHECK(ocp_qp[N].q.size() == nx[N])
      << "ocp_qp[" << N << "].q.size() must to be " << nx[N];
  // check box constraints on x
  for (unsigned int i = 0; i <= N; ++i) {
    ACHECK(ocp_qp[i].idxbx.size() == nbx[i])
        << "ocp_qp[" << i << "].idxbx.size() must to be " << nbx[i];
    ACHECK(ocp_qp[i].lbx.size() == nbx[i])
        << "ocp_qp[" << i << "].lbx.size() must to be " << nbx[i];
    ACHECK(ocp_qp[i].ubx.size() == nbx[i])
        << "ocp_qp[" << i << "].ubx.size() must to be " << nbx[i];
    if (ocp_qp[i].lbx_mask.size() != 0) {
      ACHECK(ocp_qp[i].lbx_mask.size() == nbx[i])
          << "ocp_qp[" << i << "].lbx_mask.size() must to be " << nbx[i];
    }
    if (ocp_qp[i].ubx_mask.size() != 0) {
      ACHECK(ocp_qp[i].ubx_mask.size() == nbx[i])
          << "ocp_qp[" << i << "].ubx_mask.size() must to be " << nbx[i];
    }
  }
  // check box constraints on u
  for (unsigned int i = 0; i < N; ++i) {
    ACHECK(ocp_qp[i].idxbu.size() == nbu[i])
        << "ocp_qp[" << i << "].idxbu.size() must to be " << nbu[i];
    ACHECK(ocp_qp[i].lbu.size() == nbu[i])
        << "ocp_qp[" << i << "].lbu.size() must to be " << nbu[i];
    ACHECK(ocp_qp[i].ubu.size() == nbu[i])
        << "ocp_qp[" << i << "].ubu.size() must to be " << nbu[i];
    if (ocp_qp[i].lbu_mask.size() != 0) {
      ACHECK(ocp_qp[i].lbu_mask.size() == nbu[i])
          << "ocp_qp[" << i << "].lbu.size() must to be " << nbu[i];
    }
    if (ocp_qp[i].ubu_mask.size() != 0) {
      ACHECK(ocp_qp[i].ubu_mask.size() == nbu[i])
          << "ocp_qp[" << i << "].ubu.size() must to be " << nbu[i];
    }
  }
  // check general constraints
  for (unsigned int i = 0; i < N; ++i) {
    ACHECK(ocp_qp[i].C.rows() == ng[i]) 
        << "ocp_qp[" << i << "].C.rows() must to be " << ng[i];
    if (ng[i] > 0) {
      ACHECK(ocp_qp[i].C.cols() == nx[i])
          << "ocp_qp[" << i << "].C.cols() must to be " << nx[i];
    }
    ACHECK(ocp_qp[i].D.rows() == ng[i]) 
        << "ocp_qp[" << i << "].D.rows() must to be " << ng[i];
    if (ng[i] > 0) {
      ACHECK(ocp_qp[i].D.cols() == nu[i])
          << "ocp_qp[" << i << "].D.cols() must to be " << nu[i];
    }
    ACHECK(ocp_qp[i].lg.size() == ng[i])
        << "ocp_qp[" << i << "].lg.size() must to be " << ng[i];
    ACHECK(ocp_qp[i].ug.size() == ng[i])
        << "ocp_qp[" << i << "].ug.size() must to be " << ng[i];
    if (ocp_qp[i].lg_mask.size() != 0) {
      ACHECK(ocp_qp[i].lg_mask.size() == ng[i])
          << "ocp_qp[" << i << "].lg_mask.size() must to be " << ng[i];
    }
    if (ocp_qp[i].ug_mask.size() != 0) {
      ACHECK(ocp_qp[i].ug_mask.size() == ng[i])
          << "ocp_qp[" << i << "].ug_mask.size() must to be " << ng[i];
    }
  }
  ACHECK(ocp_qp[N].C.rows() == ng[N]) 
      << "ocp_qp[" << N << "].C.rows() must to be " << ng[N];
  if (ng[N] > 0) {
    ACHECK(ocp_qp[N].C.cols() == nx[N])
        << "ocp_qp[" << N << "].C.cols() must to be " << nx[N];
  }
  ACHECK(ocp_qp[N].lg.size() == ng[N])
      << "ocp_qp[" << N << "].lg.size() must to be " << ng[N];
  ACHECK(ocp_qp[N].ug.size() == ng[N])
      << "ocp_qp[" << N << "].ug.size() must to be " << ng[N];
  if (ocp_qp[N].lg_mask.size() != 0) {
    ACHECK(ocp_qp[N].lg_mask.size() == ng[N])
        << "ocp_qp[" << N << "].lg_mask.size() must to be " << ng[N];
  }
  if (ocp_qp[N].ug_mask.size() != 0) {
    ACHECK(ocp_qp[N].ug_mask.size() == ng[N])
        << "ocp_qp[" << N << "].ug_mask.size() must to be " << ng[N];
  }
  // check soft constraints
  for (unsigned int i = 0; i < N; ++i) {
    ACHECK(ocp_qp[i].Zl.rows() == nsg[i])
        << "ocp_qp[" << i << "].Zl.rows() must to be " << nsg[i];
    ACHECK(ocp_qp[i].Zl.cols() == nsg[i])
        << "ocp_qp[" << i << "].Zl.cols() must to be " << nsg[i];
    ACHECK(ocp_qp[i].Zu.rows() == nsg[i])
        << "ocp_qp[" << i << "].Zu.rows() must to be " << nsg[i];
    ACHECK(ocp_qp[i].Zu.cols() == nsg[i])
        << "ocp_qp[" << i << "].Zu.cols() must to be " << nsg[i];
    ACHECK(ocp_qp[i].zl.size() == nsg[i])
        << "ocp_qp[" << i << "].zl.size() must to be " << nsg[i];
    ACHECK(ocp_qp[i].zu.size() == nsg[i])
        << "ocp_qp[" << i << "].zu.size() must to be " << nsg[i];
    ACHECK(ocp_qp[i].idxs.size() == nsg[i])
        << "ocp_qp[" << i << "].idxs.size() must to be " << nsg[i];
    ACHECK(ocp_qp[i].lls.size() == nsg[i])
        << "ocp_qp[" << i << "].lls.size() must to be " << nsg[i];
    ACHECK(ocp_qp[i].lus.size() == nsg[i])
        << "ocp_qp[" << i << "].lus.size() must to be " << nsg[i];
  }
}

} // namespace hpipm
} // namespace pnc
} // namespace xju