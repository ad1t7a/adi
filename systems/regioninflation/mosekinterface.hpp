#pragma once
#include "mathematics/ellipsoid.hpp"
#include "mathematics/polyhedron.hpp"
#include <exception>
#include <mosek.h>

namespace adi {
namespace systems {
class MosekInterface {
public:
  MosekInterface() {}
  ~MosekInterface() {}

  static void extract_solution(double *xx, double *barx, int n,
                               std::vector<int> ndx_d,
                               adi::mathematics::Ellipsoid *ellipsoid);

  static double inner_ellipsoid(const adi::mathematics::Polyhedron &polyhedron,
                                adi::mathematics::Ellipsoid *ellipsoid,
                                MSKenv_t *existing_env = NULL);

  static void chooseClosestPointSolver(const Eigen::MatrixXd &Points,
                                       Eigen::VectorXd &result, MSKenv_t &env);

  static void closest_point_in_convex_hull(const MatrixXd &Points,
                                           VectorXd &result,
                                           MSKenv_t *existing_env);
};
} // namespace systems
} // namespace adi