/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_COLLISION_LIB__GJK_H_
#define COMMON_THIRD_PARTY_COLLISION_LIB__GJK_H_

namespace xju {
namespace pnc {

class Gjk {
 public:
  struct Polygon {
    int num_vertex;                   // number of polygon's vertices
    double **vertices;                // pointer to pointer to the vertices's coordinate
  };

  struct Simplex {
    int num_vertex;                   // number of simplex's vertices
    double vertices[3][2];            // simplex's vertices
    double lambdas[3];                // barycentric coordinates for each vertex
    double pts_p[3][2];               // points of polygon P that form the simplex
    double pts_q[3][2];               // points of polygon Q that form the simplex
  };

 public:
  // Level 1: binary result, collide or not
  static bool GjkLevel1(const Polygon& polygon_p, const Polygon& polygon_q);
  // Level 2: distance scalar
  static double GjkLevel2(const Polygon& polygon_p, const Polygon& polygon_q);

 private:
  // maybe use hill-climbing algorithm
  inline static void Support(const Polygon& polygon, const double* direction, double* s);

  inline static void RemovePointFrom3P(Simplex& simplex, int index);

  inline static void RemovePointFrom2P(Simplex& simplex, int index);

  inline static void ClaculateLambda(Simplex& simplex, double u, double v, double sum);

  inline static void ClaculateDirection(Simplex& simplex, double* direction);

  inline static void S1D(Simplex& simplex, double* direction);

  inline static void ConeRegion0(Simplex& simplex, double* direction, int index);

  inline static void ConeRegion1(Simplex& simplex, double* direction, int index);

  inline static void ConeRegion2(Simplex& simplex, double* direction, int index);

  inline static void S2D(Simplex& simplex, double* direction);

  // barycode-based distance sub-algorithm
  inline static void SubAlgorithm(Simplex& s, double* dir);

};

} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_COLLISION_LIB__GJK_H_