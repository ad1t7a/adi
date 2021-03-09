#include "mathematics/polyhedron.hpp"
#include "interfaces/cddinterface.hpp"

namespace adi {
namespace mathematics {
//! Constructor
Polyhedron::Polyhedron(size_t dim)
    : mA(0, dim), mb(0, 1), mRepresentationDirty(true) {}

//! Constructor
Polyhedron::Polyhedron(MatrixXd A, VectorXd b)
    : mA(A), mb(b), mRepresentationDirty(true) {}

//! destructor
Polyhedron::~Polyhedron() {}

//! get A matrix
const MatrixXd &Polyhedron::A() const { return mA; }

//! set A Matrix
void Polyhedron::A(const MatrixXd &A) {
  mA = A;
  mRepresentationDirty = true;
}

//! get B Matrix
const VectorXd &Polyhedron::b() const { return mb; }

//! set B Matrix
void Polyhedron::b(const VectorXd &b) {
  mb = b;
  mRepresentationDirty = true;
}

//! get dimension
int Polyhedron::dim() const { return mA.cols(); }

//! get number of constraints
int Polyhedron::getNumberOfConstraints() const { return mA.rows(); }

//! append constraint
void Polyhedron::appendConstraints(const Polyhedron &other) {
  mA.conservativeResize(mA.rows() + other.A().rows(), mA.cols());
  mA.bottomRows(other.A().rows()) = other.A();
  mb.conservativeResize(mb.rows() + other.b().rows());
  mb.tail(other.b().rows()) = other.b();
  mRepresentationDirty = true;
}

//! contains point
bool Polyhedron::contains(VectorXd point, double tolerance) {
  return (mA * point - mb).maxCoeff() < tolerance;
}

//! generator points
std::vector<VectorXd> Polyhedron::generatorPoints() {
  if (mRepresentationDirty) {
    updateRepresentation();
  }
  return mGeneratorPoints;
}

//! generator rays
std::vector<VectorXd> Polyhedron::generatorRays() {
  if (mRepresentationDirty) {
    updateRepresentation();
  }
  return mGeneratorRays;
}

//! update representation
void Polyhedron::updateRepresentation() {
  mGeneratorPoints.clear();
  mGeneratorRays.clear();
  interfaces::CDDInterface::getGenerators(mA, mb, mGeneratorPoints,
                                          mGeneratorRays);
  mRepresentationDirty = false;
}
} // namespace mathematics
} // namespace adi