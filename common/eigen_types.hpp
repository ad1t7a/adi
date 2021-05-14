#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <unsupported/Eigen/Polynomials>
#include <vector>

namespace adi {
template <typename Scalar> using Vector1 = Eigen::Matrix<Scalar, 1, 1>;

using Vector1d = Eigen::Matrix<double, 1, 1>;

template <typename Scalar> using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;

using Vector3d = Eigen::Matrix<double, 3, 1>;

template <typename Scalar> using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar> using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;

template <typename Scalar> using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

template <typename Scalar> using Vector7 = Eigen::Matrix<Scalar, 7, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

template <typename Scalar, int Rows>
using Vector = Eigen::Matrix<Scalar, Rows, 1>;

template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;

template <typename Scalar> using RowVector2 = Eigen::Matrix<Scalar, 1, 2>;

template <typename Scalar> using RowVector3 = Eigen::Matrix<Scalar, 1, 3>;

template <typename Scalar> using RowVector4 = Eigen::Matrix<Scalar, 1, 4>;

template <typename Scalar> using RowVector6 = Eigen::Matrix<Scalar, 1, 6>;

template <typename Scalar, int Cols>
using RowVector = Eigen::Matrix<Scalar, 1, Cols>;

template <typename Scalar>
using RowVectorX = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>;

using RowVectorXd = Eigen::Matrix<double, 1, Eigen::Dynamic>;

template <typename Scalar> using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;

template <typename Scalar> using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar> using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

template <typename Scalar> using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

template <typename Scalar>
using Matrix2X = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;

template <typename Scalar>
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

using Matrix3Xd = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

template <typename Scalar>
using Matrix4X = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>;

template <typename Scalar>
using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

template <typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar> using Quaternion = Eigen::Quaternion<Scalar>;
using Quaterniond = Eigen::Quaternion<double>;

template <typename Scalar> using AngleAxis = Eigen::AngleAxis<Scalar>;

template <typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using Isometry3d = Eigen::Transform<double, 3, Eigen::Isometry>;
template <typename Scalar> using Translation3 = Eigen::Translation<Scalar, 3>;

template <typename Scalar> using WrenchVector = Eigen::Matrix<Scalar, 6, 1>;

// rotation matrix
typedef Eigen::AngleAxisd AngleAxisd;

// translation
typedef Eigen::Translation<double, 3> Translation;
typedef Eigen::Affine3d Affine3d;

// svd matrix
typedef Eigen::JacobiSVD<Eigen::MatrixXd> SVDXd;

// hyperplane
typedef std::pair<Eigen::VectorXd, double> Hyperplane;

// std vector to eigen vector
template <class T>
Eigen::Matrix<T, 1, Eigen::Dynamic> StdVecToEigenVec(std::vector<double> vec) {
  Eigen::Matrix<T, 1, Eigen::Dynamic> eigenVector =
      Eigen::Matrix<T, 1, Eigen::Dynamic>::Zero(vec.size());
  for (size_t i = 0; i < vec.size(); i++) {
    eigenVector[i] = vec[i];
  }
  return eigenVector;
}

} // namespace adi
