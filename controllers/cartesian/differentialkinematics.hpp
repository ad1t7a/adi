#pragma once

#include "common/copyable.hpp"
#include "common/eigen_types.hpp"
#include "gurobi_c++.h"
#include <rbdl/rbdl.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include <urdfreader/urdfreader.h>

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
};

class DifferentialKinematics {
public:
  //! constructor
  DifferentialKinematics(std::string path);

  //! destructor
  ~DifferentialKinematics();

  //! solve
  void step(unsigned int &bodyID, Eigen::VectorXd &jntPos,
            Eigen::VectorXd &jntVel, Vector6d &cartVel,
            Eigen::VectorXd &cmdJntVel);

  //! get DoF
  size_t getDoF() { return m_model->q_size; }

  // diff IK options
  DifferentialKinematicsOptions options;

private:
  //! robot model
  RigidBodyDynamics::Model *m_model;

  //! robot urdf model
  RigidBodyDynamics::ModelPtr m_urdfModel;

  // jacobian pos ori
  MatrixXd mJacobianOriPos;

};
} // namespace controllers
} // namespace adi