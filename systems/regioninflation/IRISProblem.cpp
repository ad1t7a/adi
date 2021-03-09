#include "systems/regioninflation/IRISProblem.hpp"
#include "mathematics/ellipsoid.hpp"

namespace adi {
namespace systems {

//! constructor
IRISProblem::IRISProblem(int dim) : mBounds(dim), mDim(dim), mSeed(dim) {}

//! destructor
IRISProblem::~IRISProblem() {}

//! add obstacle
void IRISProblem::addObstacle(MatrixXd obstacleVertices) {
  ADI_ASSERT(obstacleVertices.rows() == this->dim());
  this->mObstaclePts.push_back(obstacleVertices);
}

const std::vector<MatrixXd> &IRISProblem::getObstacles() const {
  return this->mObstaclePts;
}

//! dimension
int IRISProblem::dim() const { return this->mDim; }

//! set bounds
void IRISProblem::setBounds(mathematics::Polyhedron newBounds) {
  ADI_ASSERT(newBounds.dim() == this->dim());
  this->mBounds = newBounds;
}

mathematics::Polyhedron IRISProblem::getBounds() const { return mBounds; }

//! set seed point
void IRISProblem::setSeedPoint(VectorXd point) {
  ADI_ASSERT(point.size() == this->dim());
  this->mSeed = mathematics::Ellipsoid::fromNSphere(point);
}

//! set seed ellipsoid
void IRISProblem::setSeedEllipsoid(mathematics::Ellipsoid ellipsoid) {
  ADI_ASSERT(ellipsoid.dim() == this->dim());
  mSeed = ellipsoid;
}

//! get seed
mathematics::Ellipsoid IRISProblem::getSeed() const { return mSeed; }
} // namespace systems
} // namespace adi