#include "systems/regioninflation/IRIS.hpp"
#include "common/arg_sort.hpp"
#include "spdlog/spdlog.h"
#include "systems/regioninflation/mosekinterface.hpp"
#include <numeric>
namespace adi {
namespace systems {

//! separating hyperplane using QP formulation of a modified support vector
//! machine
void IRIS::separatingHyperplanes(const std::vector<MatrixXd> obstaclePts,
                                 const adi::mathematics::Ellipsoid &ellipsoid,
                                 adi::mathematics::Polyhedron &polyhedron,
                                 bool &infeasibleStart) {
  int dim = ellipsoid.dim();
  infeasibleStart = false;
  int numObstacles = obstaclePts.size();
  if (numObstacles == 0) {
    polyhedron.A(MatrixXd::Zero(0, dim));
    polyhedron.b(VectorXd::Zero(0));
    return;
  }
  MatrixXd Cinv = ellipsoid.C().inverse();

  // convert ellipse and the obstacle to ball space
  std::vector<Eigen::MatrixXd> imagePts(numObstacles);
  for (int i = 0; i < numObstacles; i++) {
    imagePts[i] = Cinv * (obstaclePts[i].colwise() - ellipsoid.d());
  }

  // squared distances (of obstacles)
  std::vector<VectorXd> imageSquaredDist(numObstacles);
  for (int i = 0; i < numObstacles; i++) {
    imageSquaredDist[i] = imagePts[i].colwise().squaredNorm();
  }

  std::vector<double> imageMinDist(numObstacles);
  for (int i = 0; i < numObstacles; i++) {
    imageMinDist[i] = imageSquaredDist[i].minCoeff();
  }

  // uncovered obstacles
  Eigen::Matrix<bool, Eigen::Dynamic, 1> uncoveredObstacles =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Constant(numObstacles, true);

  // hyperplanes
  std::vector<Hyperplane> planes;
  MSKenv_t env = NULL;

  // sort indices to arrange obstacles
  std::vector<size_t> obstacleSortedIdx = adi::arg_sort(imageMinDist);
  for (auto it = obstacleSortedIdx.begin(); it != obstacleSortedIdx.end();
       ++it) {
    // get closest point on obstacles
    size_t i = *it;
    Eigen::DenseIndex idx;
    imageSquaredDist[i].minCoeff(&idx);
    if (!uncoveredObstacles(i)) {
      continue;
    }
    // tangent plane
    Hyperplane plane =
        ellipsoid.tangentPlaneThroughPoint(obstaclePts[i].col(idx));
    if ((((plane.first.transpose() * obstaclePts[i]).array() - plane.second) >=
         0)
            .all()) {
      planes.push_back(plane);
    } else {
      // solve QP to get the hyperplane
      VectorXd ystar(dim);
      adi::systems::MosekInterface::chooseClosestPointSolver(imagePts[i], ystar,
                                                             env);
      if (ystar.squaredNorm() < 1e-6) {
        infeasibleStart = true;
        planes.emplace_back(-plane.first,
                            -plane.first.transpose() * obstaclePts[i].col(idx));
      } else {
        VectorXd xstar = ellipsoid.C() * ystar + ellipsoid.d();
        planes.push_back(ellipsoid.tangentPlaneThroughPoint(xstar));
      }
    }
    for (size_t j = 0; j < numObstacles; j++) {
      if (((planes.back().first.transpose() * obstaclePts[j]).array() >=
           planes.back().second)
              .all()) {
        uncoveredObstacles(j) = false;
      }
    }
    uncoveredObstacles(i) = false;
    if (!uncoveredObstacles.any()) {
      break;
    }
  }
  MatrixXd A(planes.size(), dim);
  VectorXd b(planes.size());
  for (auto it = planes.begin(); it != planes.end(); ++it) {
    A.row(it - planes.begin()) = it->first.transpose();
    b(it - planes.begin()) = it->second;
  }
  polyhedron.A(A);
  polyhedron.b(b);
  return;
}

//! separating hyperplane
IRISRegion IRIS::inflateRegion(IRISProblem problem, IRISOptions options) {
  IRISRegion region(problem.dim());
  region.mEllipsoid.C(problem.getSeed().C());
  region.mEllipsoid.d(problem.getSeed().d());
  double bestVol = pow(1e-4, problem.dim());
  double volume = 0.0;
  long int iter = 0;
  bool infeasibleStart;
  adi::mathematics::Polyhedron newPoly(problem.dim());
  while (true) {
    separatingHyperplanes(problem.getObstacles(), region.mEllipsoid, newPoly,
                          infeasibleStart);
    ADI_ASSERT((options.errorOnInfeasibleStart && infeasibleStart) == false);
    newPoly.appendConstraints(problem.getBounds());

    if (options.requireContainment) {
      bool allPointsContained;
      if (options.requiredContainmentPoints.size()) {
        allPointsContained = true;
        for (auto pt = options.requiredContainmentPoints.begin();
             pt != options.requiredContainmentPoints.end(); ++pt) {
          if (!newPoly.contains(*pt, 0.0)) {
            allPointsContained = false;
            break;
          }
        }
      } else {
        allPointsContained = newPoly.contains(problem.getSeed().d(), 0.0);
      }
      if (allPointsContained || infeasibleStart) {
        region.mPolyhedron = newPoly;
      } else {
        spdlog::info("Breaking early because the start point is no longer "
                     "contained in the polyhedron");
        return region;
      }
    } else {
      region.mPolyhedron = newPoly;
    }
    volume = adi::systems::MosekInterface::inner_ellipsoid(region.mPolyhedron,
                                                           &region.mEllipsoid);
    const bool atIterLimit =
        (options.iterLimit > 0) && (iter + 1 >= options.iterLimit);
    const bool insufficientProgress =
        (std::abs(volume - bestVol) / bestVol) < options.terminationThreshold;
    if (atIterLimit || insufficientProgress) {
      break;
    }
    bestVol = volume;
    iter++;
  }
  return region;
}

} // namespace systems
} // namespace adi