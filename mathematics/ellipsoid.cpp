#include "mathematics/ellipsoid.hpp"
#include "common/factorial.hpp"

namespace adi {
namespace mathematics {
//! constructor
Ellipsoid::Ellipsoid(size_t dim) : mC(MatrixXd(dim, dim)), md(VectorXd(dim)) {}

//! constructor
Ellipsoid::Ellipsoid(MatrixXd C, VectorXd d) : mC(C), md(d) {}

//! destructor
Ellipsoid::~Ellipsoid() {}

//! C Matrix
const MatrixXd &Ellipsoid::C() const { return mC; }

//! C Matrix
void Ellipsoid::C(const MatrixXd &c) { mC = c; }

//! set C matrix entry
void Ellipsoid::setCEntry(Eigen::DenseIndex row, Eigen::DenseIndex col,
                          double value) {
  mC(row, col) = value;
}

//! get D vector
const VectorXd &Ellipsoid::d() const { return md; }

//! set D vector
void Ellipsoid::d(const VectorXd &d) { md = d; }

//! set D vector entry
void Ellipsoid::setDEntry(Eigen::DenseIndex idx, double value) {
  md(idx) = value;
}

//! get dimension
int Ellipsoid::dim() const { return mC.cols(); }

//! sphere volume
double Ellipsoid::nSphereVolume(int dim, double radius) const {
  double v;
  int k = std::floor(dim / 2);
  if (dim % 2 == 0) {
    v = std::pow(M_PI, k) / static_cast<double>(adi::factorial(k));
  } else {
    v = (2.0 * adi::factorial(k) * std::pow(4 * M_PI, k)) /
        static_cast<double>(adi::factorial(2 * k + 1));
  }
  return v * std::pow(radius, dim);
}

//! get volume
double Ellipsoid::getVolume() const {
  return mC.determinant() * nSphereVolume(this->dim(), 1.0);
}

//! ellipsoid from N sphere
Ellipsoid Ellipsoid::fromNSphere(VectorXd &center, double radius) {
  const int dim = center.size();
  MatrixXd C = MatrixXd::Zero(dim, dim);
  C.diagonal().setConstant(radius);
  Ellipsoid ellipsoid(C, center);
  return ellipsoid;
}

//! tangent plane through point
Hyperplane Ellipsoid::tangentPlaneThroughPoint(const VectorXd &x) const {
  MatrixXd Cinv = C().inverse();
  MatrixXd Cinv2 = Cinv * Cinv.transpose();
  VectorXd nhat = (2 * Cinv2 * (x - d())).normalized();
  Hyperplane plane(nhat, nhat.transpose() * x);
  return plane;
}
} // namespace mathematics
} // namespace adi