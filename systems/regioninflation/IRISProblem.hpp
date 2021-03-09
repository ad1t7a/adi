#pragma once

#include "common/copyable.hpp"
#include "common/eigen_types.hpp"
#include "mathematics/ellipsoid.hpp"
#include "mathematics/polyhedron.hpp"
#include <vector>

namespace adi {
namespace systems {
class IRISProblem {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IRISProblem);

  //! constructor
  IRISProblem(int dim);

  //! destructor
  ~IRISProblem();

  //! add obstacle
  void addObstacle(MatrixXd obstacleVertices);

  //! get obstacles
  const std::vector<MatrixXd> &getObstacles() const;

  //! get dimension
  int dim() const;

  //! set bounds
  void setBounds(mathematics::Polyhedron newBounds);

  //! get bounds
  mathematics::Polyhedron getBounds() const;

  //! set seed point
  void setSeedPoint(VectorXd point);

  void setSeedEllipsoid(mathematics::Ellipsoid ellipsoid);

  mathematics::Ellipsoid getSeed() const;

private:
  //! obstacle points
  std::vector<MatrixXd> mObstaclePts;

  //! bounds
  adi::mathematics::Polyhedron mBounds;

  //! dimensions
  int mDim;

  //! region to inflate
  adi::mathematics::Ellipsoid mSeed;
};
} // namespace systems
} // namespace adi