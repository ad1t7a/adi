#pragma once

#include <Eigen/Dense>
#include <vector>

namespace adi {

template <typename DerivedA, typename DerivedB>
bool is_approx_equal_abstol(const Eigen::MatrixBase<DerivedA> &m1,
                            const Eigen::MatrixBase<DerivedB> &m2,
                            double tolerance) {
  return ((m1.rows() == m2.rows()) && (m1.cols() == m2.cols()) &&
          ((m1 - m2).template lpNorm<Eigen::Infinity>() <= tolerance));
}

template <typename DerivedA, typename DerivedB>
bool isApproxEqualAbstolPermutedColumns(const Eigen::MatrixBase<DerivedA> &m1,
                                        const Eigen::MatrixBase<DerivedB> &m2,
                                        double tolerance) {
  if (((m1.rows() != m2.rows()) && (m1.cols() != m2.cols()))) {
    return false;
  }
  std::vector<bool> available(m2.cols(), true);

  for (std::size_t i = 0; i < m1.cols(); i++) {
    bool foundMatch = false;
    for (std::size_t j = 0; j < m2.cols(); j++) {
      if (available[j] && is_approx_equal_abstol(m1, m2, tolerance)) {
        foundMatch = true;
        available[j] = false;
        break;
      }
    }
    if (!foundMatch) {
      return false;
    }
  }
  return true;
}
} // namespace adi
