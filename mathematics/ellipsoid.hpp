#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"

namespace adi {
namespace mathematics {

class Ellipsoid {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Ellipsoid);

  //! constructor
  Ellipsoid(size_t dim = 0);

  //! constructor
  Ellipsoid(MatrixXd C, VectorXd d);

  //! destructor
  ~Ellipsoid();

  //! get C Matrix
  const MatrixXd &C() const;

  //! set C Matrix
  void C(const MatrixXd &c);

  //! set C matrix entry
  void setCEntry(Eigen::DenseIndex row, Eigen::DenseIndex col, double value);

  //! get D vector
  const VectorXd &d() const;

  //! set D vector
  void d(const VectorXd &d);

  //! set D vector entry
  void setDEntry(Eigen::DenseIndex idx, double value);

  //! get dimension
  int dim() const;

  //! create elliposid from N sphere
  static Ellipsoid fromNSphere(VectorXd &center, double radius = 1e-4);

  //! get volume
  double getVolume() const;

  //! tangent plane through elliposid
  Hyperplane tangentPlaneThroughPoint(const VectorXd &x) const;

private:
  //! sphere volume
  double nSphereVolume(int dim, double radius) const;

  //! elliposid description
  MatrixXd mC;

  //! elliposid description
  VectorXd md;
};
} // namespace mathematics
} // namespace adi