#include "common/eigen_types.hpp"
#include "systems/regioninflation/IRIS.hpp"
#include "systems/regioninflation/IRISOptions.hpp"
#include "systems/regioninflation/IRISProblem.hpp"

int main() {

  adi::systems::IRISProblem problem(2);
  problem.setSeedPoint(adi::Vector2d(0.1, 0.1));

  // Inflate a region inside a 1x1 box
  adi::MatrixXd obs(2, 2);
  obs << 0, 1, 0, 0;
  problem.addObstacle(obs);
  obs << 1, 1, 0, 1;
  problem.addObstacle(obs);
  obs << 1, 0, 1, 1;
  problem.addObstacle(obs);
  obs << 0, 0, 1, 0;
  problem.addObstacle(obs);

  adi::systems::IRISOptions options;
  adi::systems::IRIS iris;
  adi::systems::IRISRegion region = iris.inflateRegion(problem, options);
  return 0;
}