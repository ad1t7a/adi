#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"

// CDD libraries
#include "setoper.h"
#include "cdd.h"

namespace adi {
namespace interfaces {
class CDDInterface {
public:
  static void getGenerators(MatrixXd &A, VectorXd &b,
                            std::vector<Eigen::VectorXd> &generatorPoints,
                            std::vector<Eigen::VectorXd> &generatorRays) {
    ADI_ASSERT(A.rows() == b.rows());
    size_t dim = A.cols();

    dd_MatrixPtr hrep = dd_CreateMatrix(A.rows(), 1 + dim);
    for (size_t i = 0; i < A.rows(); i++) {
      dd_set_d(hrep->matrix[i][0], b(i));
      for (size_t j = 0; j < dim; j++) {
        dd_set_d(hrep->matrix[i][j + 1], -A(i, j));
      }
    }
    hrep->representation = dd_Inequality;
    dd_ErrorType err;
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(hrep, &err);
    ADI_ASSERT(err == dd_NoError);

    dd_MatrixPtr generators = dd_CopyGenerators(poly);
    ADI_ASSERT(dim + 1 == generators->colsize);

    for (size_t i = 0; i < generators->rowsize; i++) {
      VectorXd pointOrRay(dim);
      for (size_t j = 0; j < dim; j++) {
        pointOrRay(j) = dd_get_d(generators->matrix[i][j + 1]);
      }
      if (dd_get_d(generators->matrix[i][0]) == 0) {
        generatorRays.push_back(pointOrRay);
      } else {
        generatorPoints.push_back(pointOrRay);
      }
    }
    dd_FreeMatrix(hrep);
    dd_FreeMatrix(generators);
    dd_FreePolyhedra(poly);
  }

private:
  // Disallow creating an instance of this object
  CDDInterface() {}
};
} // namespace interfaces
} // namespace adi