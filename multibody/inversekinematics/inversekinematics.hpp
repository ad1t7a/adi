#pragma once

#include "common/copyable.hpp"
#include "common/eigen_types.hpp"
#include "common/frame.hpp"
#include "gurobi_c++.h"
#include "multibody/rbdlmultibody.hpp"

namespace adi {
namespace multibody {
class InverseKinematicsOptions {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InverseKinematicsOptions);

  // constructor
  InverseKinematicsOptions() {}

  // destructor
  ~InverseKinematicsOptions() {}

  // log to console
  bool mLogToConsole{true};

  // obstacle avoidance
  bool mObstacleAvoidance{false};

  // cartesian position error
  double mCartesianPositionError{0.0005};

  // acceptable cartesian position error
  double mCartesianPosErrorRoundToZero{0.0001};

  // acceptable cartesian orientation error
  double mCartesianOriErrorRoundToZero{0.0001};
};

class InverseKinematics {
public:
  //! constructor
  InverseKinematics(std::string path);

  //! destructor
  virtual ~InverseKinematics();

  //! solve IK
  void solve(unsigned int &bodyID, Eigen::VectorXd &jntPos, Frame &cartPose);

  //! IK options
  InverseKinematicsOptions options;

private:
  //! multibody
  RBDLMultiBody *m_model;
};
} // namespace multibody
} // namespace adi