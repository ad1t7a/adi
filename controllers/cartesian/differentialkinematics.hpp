#pragma once

#include "common/copyable.hpp"
#include "common/eigen_types.hpp"
#include "gurobi_c++.h"

#include "multibody/rbdlmultibody.hpp"

namespace adi {
namespace controllers {
class DifferentialKinematicsOptions {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentialKinematicsOptions);

  // constructor
  DifferentialKinematicsOptions() {}

  // destructor
  ~DifferentialKinematicsOptions() {}

  // log to console
  bool mLogToConsole{true};

  // obstacle avoidance
  bool mObstacleAvoidance{true};
};

class DifferentialKinematics {
public:
  //! constructor
  DifferentialKinematics(std::string path);

  //! destructor
  ~DifferentialKinematics();

  //! solve
  void step(unsigned int bodyID, Eigen::VectorXd &jntPos,
            Eigen::VectorXd &jntVel, Vector6d &cartVel,
            Eigen::VectorXd &cmdJntVel);

  //! get DoF
  size_t getDoF() { return m_dynamics->getDoFs(); }

  // diff IK options
  DifferentialKinematicsOptions options;

private:
  //!
  multibody::RBDLMultiBody *m_dynamics;

  // jacobian pos ori
  MatrixXd mJacobianOriPos;

};
} // namespace controllers
} // namespace adi