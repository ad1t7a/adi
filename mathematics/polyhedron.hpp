#pragma once

#include "common/adi_assert.hpp"
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"

namespace adi {
namespace mathematics {

class Polyhedron {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Polyhedron);

  //! constructor
  Polyhedron(size_t dim = 0);

  //! constructor
  Polyhedron(MatrixXd A, VectorXd b);

  //! destructor
  ~Polyhedron();

  //! get A Matrix
  const MatrixXd &A() const;

  //! set A Matrix
  void A(const MatrixXd &A);

  //! get B Matrix
  const VectorXd &b() const;

  //! set B Matrix
  void b(const VectorXd &b);

  //! get dimension
  int dim() const;

  //! get number of constraints
  int getNumberOfConstraints() const;

  //! append constraint
  void appendConstraints(const Polyhedron &other);

  //! generator points
  std::vector<VectorXd> generatorPoints();

  //! get generator rays
  std::vector<VectorXd> generatorRays();

  //! check if polyhedron contains point
  bool contains(VectorXd point, double tolerance);

private:
  //! update representation
  void updateRepresentation();

  //! A matrix
  MatrixXd mA;

  //! b vector
  VectorXd mb;

  //! representation dirty
  bool mRepresentationDirty;

  //! generator points
  std::vector<Eigen::VectorXd> mGeneratorPoints;

  //! generator rays
  std::vector<Eigen::VectorXd> mGeneratorRays;
};
} // namespace mathematics
} // namespace adi