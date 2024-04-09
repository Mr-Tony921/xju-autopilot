/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/collision_lib/gjk.h"

#include "stdint.h"
#include <iostream>
#include <cmath>
#include <limits>

// macro that implements the CompareSign function
#define SAMESIGN(m, n) ((m > 0.) == (n > 0.)) 

// macro that implements squared vector norm 
#define NORM2(m) (m[0] * m[0] + m[1] * m[1])

// macro that implements dot product 
#define DOTPRODUCT(m, n) (m[0] * n[0] + m[1] * n[1])

// macro that implements cross product
#define CROSSPRODUCT(m, n) (m[0] * n[1] - m[1] * n[0])

// macro that assign m == n
#define ASSIGN(m, n) m[0] = n[0]; \
                     m[1] = n[1]

// Tolerance on relative distance
constexpr double kEpsRel = 1e-5;

// Tolerance on absolute distance
constexpr double kEpsAbs = 1e-5;

// Maximum number of iterations of the GJK algorithm
constexpr int kMaxInter = 50;


namespace xju {
namespace pnc {

bool Gjk::GjkLevel1(const Polygon& polygon_p, const Polygon& polygon_q) {
  int i = 0;                       // counter, avoid constructe and destructe
  double tesnorm = 0.;             // norm2 of simplex's vertices, avoid constructe and destructe
  double norm2Wmax = 0.;
  int iteration = 0;               // iteration counter
  double direction[2];             // search direction
  double negetive_direction[2];    // search direction * -1
  double s_p[2], s_q[2];           // sopport mapping computed last
  double w[2];                     // vertex on CSO boundary by the difference of support functions on both polygon
  
  double eps_rel2 = kEpsRel * kEpsRel;
  double eps_abs2 = kEpsAbs * kEpsAbs;

  // Initialise search direction and simplex
  Simplex simplex;
  simplex.num_vertex = 1;
  for (i = 0; i < 2; ++i) {
    direction[i] = polygon_p.vertices[0][i] - polygon_q.vertices[0][i];
    s_p[i] = polygon_p.vertices[0][i];
    s_q[i] = polygon_q.vertices[0][i];

    simplex.vertices[0][i] = direction[i];
    simplex.pts_p[0][i] = s_p[i];
    simplex.pts_q[0][i] = s_q[i];
  }

  if (NORM2(direction) <= eps_abs2) {
    return true;
  }

  // begin GJK interation
  do {
    iteration++;

    // update negetive search direction
    negetive_direction[0] = -direction[0];
    negetive_direction[1] = -direction[1];

    // support function
    Support(polygon_p, negetive_direction, s_p);
    Support(polygon_q, direction, s_q);
    // CSO point
    w[0] = s_p[0] - s_q[0];
    w[1] = s_p[1] - s_q[1];

    // 1st exit condition
    if (DOTPRODUCT(direction, w) > 0.0) {
      return false;
    } else {
      if (simplex.num_vertex == 2) {
        if (CROSSPRODUCT(simplex.vertices[0], w) * CROSSPRODUCT(simplex.vertices[1], w) <= 0.0) {
          return true;
        }
      }
    }

    // 2nd exit condition
    if (NORM2(direction) < eps_rel2) {
      break;
    }

    // add new vertex to simplex
    i = simplex.num_vertex;
    ASSIGN(simplex.vertices[i], w);
    simplex.num_vertex++;

    // record point of CSO from polygon P and Q
    for (i = 0; i < 2; ++i) {
      simplex.pts_p[simplex.num_vertex - 1][i] = s_p[i];
      simplex.pts_q[simplex.num_vertex - 1][i] = s_q[i];
    }

    // invoke distance sub-algorithm
    SubAlgorithm(simplex, direction);

    // 3rd exit condition
    for (i = 0; i < simplex.num_vertex; ++i) {
      tesnorm = NORM2(simplex.vertices[i]);
      if (tesnorm > norm2Wmax) {
        norm2Wmax = tesnorm;
      }
    }
    if (NORM2(direction) <= eps_abs2 * norm2Wmax) {
      break;
    }

    // 4th and 5th exit condition
  } while (simplex.num_vertex != 3 && iteration != kMaxInter);

  // compute and return distance
  return NORM2(direction) < eps_abs2 ? true : false;
}

double Gjk::GjkLevel2(const Polygon& polygon_p, const Polygon& polygon_q) {

  int i = 0;                       // counter, avoid constructe and destructe
  double tesnorm = 0.;             // norm2 of simplex's vertices, avoid constructe and destructe
  double norm2Wmax = 0.;
  int iteration = 0;               // iteration counter
  double direction[2];             // search direction
  double negetive_direction[2];    // search direction * -1
  double s_p[2], s_q[2];           // sopport mapping computed last
  double w[2];                     // vertex on CSO boundary by the difference of support functions on both polygon
  
  double eps_rel2 = kEpsRel * kEpsRel;
  double eps_abs2 = kEpsAbs * kEpsAbs;

  // Initialise search direction and simplex
  Simplex simplex;
  simplex.num_vertex = 1;
  for (i = 0; i < 2; ++i) {
    direction[i] = polygon_p.vertices[0][i] - polygon_q.vertices[0][i];
    s_p[i] = polygon_p.vertices[0][i];
    s_q[i] = polygon_q.vertices[0][i];

    simplex.vertices[0][i] = direction[i];
    simplex.pts_p[0][i] = s_p[i];
    simplex.pts_q[0][i] = s_q[i];
  }

  if (NORM2(direction) <= eps_abs2) {
    return 0.0;
  }

  // begin GJK interation
  do {
    iteration++;

    // update negetive search direction
    negetive_direction[0] = -direction[0];
    negetive_direction[1] = -direction[1];

    // support function
    Support(polygon_p, negetive_direction, s_p);
    Support(polygon_q, direction, s_q);
    // CSO point
    w[0] = s_p[0] - s_q[0];
    w[1] = s_p[1] - s_q[1];

    // 1st exit condition
    if (NORM2(direction) - DOTPRODUCT(direction, w) <= eps_rel2 * NORM2(direction)) {
      break;
    }

    // 2nd exit condition
    if (NORM2(direction) < eps_rel2) {
      break;
    }

    // add new vertex to simplex
    i = simplex.num_vertex;
    ASSIGN(simplex.vertices[i], w);
    // simplex.vertices[i][0] = w[0];
    // simplex.vertices[i][1] = w[1];
    simplex.num_vertex++;

    // record point of CSO from polygon P and Q
    for (i = 0; i < 2; ++i) {
      simplex.pts_p[simplex.num_vertex - 1][i] = s_p[i];
      simplex.pts_q[simplex.num_vertex - 1][i] = s_q[i];
    }

    // invoke distance sub-algorithm
    SubAlgorithm(simplex, direction);

    // 3rd exit condition
    for (i = 0; i < simplex.num_vertex; ++i) {
      tesnorm = NORM2(simplex.vertices[i]);
      if (tesnorm > norm2Wmax) {
        norm2Wmax = tesnorm;
      }
    }
    if (NORM2(direction) <= eps_abs2 * norm2Wmax) {
      break;
    }

    // 4th and 5th exit condition
  } while (simplex.num_vertex != 3 && iteration != kMaxInter);

  // compute and return distance
  return std::sqrt(NORM2(direction));
}

// maybe use hill-climbing algorithm
void Gjk::Support(const Polygon& polygon, const double* direction, double* s) {
  int index = -1;
  double max_projection = std::numeric_limits<double>::lowest();
  for (int i = 0; i < polygon.num_vertex; ++i) {
    double projection = DOTPRODUCT(polygon.vertices[i], direction);
    if (projection > max_projection) {
      max_projection = projection;
      index = i;
    }
  }
  if (index != -1) {
    ASSIGN(s, polygon.vertices[index]);
  }
}

void Gjk::RemovePointFrom3P(Simplex& simplex, int index) {
  switch (index) {
    case 0:
      ASSIGN(simplex.vertices[0], simplex.vertices[1]);
      ASSIGN(simplex.vertices[1], simplex.vertices[2]);
      ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
      ASSIGN(simplex.pts_p[1], simplex.pts_p[2]);
      ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
      ASSIGN(simplex.pts_q[1], simplex.pts_q[2]);
      simplex.num_vertex--;
      break;
    case 1:
      ASSIGN(simplex.vertices[1], simplex.vertices[2]);
      ASSIGN(simplex.pts_p[1], simplex.pts_p[2]);
      ASSIGN(simplex.pts_q[1], simplex.pts_q[2]);
      simplex.num_vertex--;
      break;
    case 2:
      simplex.num_vertex--;
      break;
    default:
      break;
  }
}

void Gjk::RemovePointFrom2P(Simplex& simplex, int index) {
  switch (index) {
    case 0:
      ASSIGN(simplex.vertices[0], simplex.vertices[1]);
      ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
      ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
      simplex.num_vertex--;
      break;
    case 1:
      simplex.num_vertex--;
      break;
    default:
      break;
  }
}

void Gjk::ClaculateLambda(Simplex& simplex, double u, double v, double sum) {
  simplex.lambdas[0] = u / sum;
  simplex.lambdas[1] = v / sum;
  simplex.lambdas[2] = 1 - simplex.lambdas[0] - simplex.lambdas[1];
}

void Gjk::ClaculateDirection(Simplex& simplex, double* direction) {
  direction[0] = simplex.lambdas[0] * simplex.vertices[0][0] + 
                 simplex.lambdas[1] * simplex.vertices[1][0] +
                 simplex.lambdas[2] * simplex.vertices[2][0];
  direction[1] = simplex.lambdas[0] * simplex.vertices[0][1] + 
                 simplex.lambdas[1] * simplex.vertices[1][1] +
                 simplex.lambdas[2] * simplex.vertices[2][1];
}

void Gjk::S1D(Simplex& simplex, double* direction) {
  // assume 1st point is A
  //        2nd point is B
  double ab[2]{simplex.vertices[1][0] - simplex.vertices[0][0],
               simplex.vertices[1][1] - simplex.vertices[0][1]};
  double con1 = DOTPRODUCT(simplex.vertices[0], ab);
  double con2 = DOTPRODUCT(simplex.vertices[1], ab);
  if (con1 > 0. || con1 == 0.) {
    simplex.num_vertex = 1;
    simplex.lambdas[0] = 1.0;
    ASSIGN(direction, simplex.vertices[0]);
    return;
  }
  if (con2 < 0. || con2 == 0.) {
    simplex.num_vertex = 1;
    simplex.lambdas[0] = 1.0;
    ASSIGN(simplex.vertices[0], simplex.vertices[1]);
    ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
    ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
    ASSIGN(direction, simplex.vertices[1]);
    return;
  }
  simplex.lambdas[0] = - con2 / (con1 - con2);
  simplex.lambdas[1] = 1- simplex.lambdas[0];
  direction[0] = simplex.lambdas[0] * simplex.vertices[0][0] +
                 simplex.lambdas[1] * simplex.vertices[1][0];
  direction[1] = simplex.lambdas[0] * simplex.vertices[0][1] +
                 simplex.lambdas[1] * simplex.vertices[1][1];
}

void Gjk::ConeRegion0(Simplex& simplex, double* direction, int index) {
  double mv[2], nv[2];
  for (int i = 0; i < 2; ++i) {
    mv[i] = simplex.vertices[0][i] - simplex.vertices[1][i];
    nv[i] = simplex.vertices[0][i] - simplex.vertices[2][i];
  }
  if (DOTPRODUCT(mv, nv) < 0.) {
    if (DOTPRODUCT(simplex.vertices[0], mv) > 0.) {
      RemovePointFrom3P(simplex, 2);
      S1D(simplex, direction);
      return;
    }
    if (DOTPRODUCT(simplex.vertices[0], nv) > 0.) {
      RemovePointFrom3P(simplex, 1);
      S1D(simplex, direction);
      return;
    }
  } else {
    simplex.num_vertex = 1;
    simplex.lambdas[0] = 1.0;
    direction = simplex.vertices[0];
  }
}

void Gjk::ConeRegion1(Simplex& simplex, double* direction, int index) {
  double mv[2], nv[2];
  for (int i = 0; i < 2; ++i) {
    mv[i] = simplex.vertices[1][i] - simplex.vertices[0][i];
    nv[i] = simplex.vertices[1][i] - simplex.vertices[2][i];
  }
  if (DOTPRODUCT(mv, nv) < 0.) {
    if (DOTPRODUCT(simplex.vertices[1], mv) > 0.) {
      RemovePointFrom3P(simplex, 2);
      S1D(simplex, direction);
      return;
    }
    if (DOTPRODUCT(simplex.vertices[1], nv) > 0.) {
      RemovePointFrom3P(simplex, 0);
      S1D(simplex, direction);
      return;
    }
  } else {
    simplex.num_vertex = 1;
    simplex.lambdas[0] = 1.0;
    ASSIGN(simplex.vertices[0], simplex.vertices[1]);
    ASSIGN(simplex.pts_p[0], simplex.pts_p[1]);
    ASSIGN(simplex.pts_q[0], simplex.pts_q[1]);
    ASSIGN(direction, simplex.vertices[0]);
  }
}

void Gjk::ConeRegion2(Simplex& simplex, double* direction, int index) {
  double mv[2], nv[2];
  for (int i = 0; i < 2; ++i) {
    mv[i] = simplex.vertices[2][i] - simplex.vertices[0][i];
    nv[i] = simplex.vertices[2][i] - simplex.vertices[1][i];
  }
  if (DOTPRODUCT(mv, nv) < 0.) {
    if (DOTPRODUCT(simplex.vertices[2], mv) > 0.) {
      RemovePointFrom3P(simplex, 1);
      S1D(simplex, direction);
      return;
    }
    if (DOTPRODUCT(simplex.vertices[2], nv) > 0.) {
      RemovePointFrom3P(simplex, 0);
      S1D(simplex, direction);
      return;
    }
  } else {
    simplex.num_vertex = 1;
    simplex.lambdas[0] = 1.0;
    ASSIGN(simplex.vertices[0], simplex.vertices[2]);
    ASSIGN(simplex.pts_p[0], simplex.pts_p[2]);
    ASSIGN(simplex.pts_q[0], simplex.pts_q[2]);
    ASSIGN(direction, simplex.vertices[0]);
  }
}

void Gjk::S2D(Simplex& simplex, double* direction) {
  // assume 1st point is A
  //        2nd point is B
  //        3rd point is C
  double u = CROSSPRODUCT(simplex.vertices[1], simplex.vertices[2]);
  double v = CROSSPRODUCT(simplex.vertices[2], simplex.vertices[0]);
  double w = CROSSPRODUCT(simplex.vertices[0], simplex.vertices[1]);
  double sum = u + v + w;
  int barycode = 
      (SAMESIGN(sum, w)) ^ (SAMESIGN(sum, v) << 1) ^ (SAMESIGN(sum, u) << 2);
  switch (barycode) {
    case 1: 
      ConeRegion2(simplex, direction, 2);
      break;
    case 2:
      ConeRegion1(simplex, direction, 1);
      break;
    case 3:
      RemovePointFrom3P(simplex, 0);
      S1D(simplex, direction);
      break;
    case 4:
      ConeRegion0(simplex, direction, 0);
      break;
    case 5:
      RemovePointFrom3P(simplex, 1);
      S1D(simplex, direction);
      break;
    case 6:
      RemovePointFrom3P(simplex, 2);
      S1D(simplex, direction);
      break;
    case 7:
      ClaculateLambda(simplex, u, v, sum);
      ClaculateDirection(simplex, direction);
      break;
    default:
      break;
  }  
}

// barycode-based distance sub-algorithm
void Gjk::SubAlgorithm(Simplex& s, double* dir) {
  switch (s.num_vertex) {
    case 3:
      S2D(s, dir);
      break;
    case 2:
      S1D(s, dir);
      break;
    case 1:
      s.lambdas[0] = 1.0;
      s.lambdas[1] = 0.0;
      s.lambdas[2] = 0.0;
      break;
    default:
      break;
  }
}

} // namespace pnc
} // namespace xju