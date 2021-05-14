#pragma once

#include "common/eigen_types.hpp"
#include "common/frame.hpp"

namespace adi {
namespace io {
namespace arm {
class RobotState {
public:
  // constructor
  RobotState(int dof) {
    mJntPosition = VectorXd::Zero(dof);
    mJntVelocity = VectorXd::Zero(dof);
    mJntAcceleration = VectorXd::Zero(dof);
    mJntTorque = VectorXd::Zero(dof);
    mCartVelocity = VectorXd::Zero(6);
  }

  // destructor
  ~RobotState() {}

  // jnt pos
  VectorXd mJntPosition;

  // jnt vel
  VectorXd mJntVelocity;

  // jnt acc
  VectorXd mJntAcceleration;

  // jnt trq
  VectorXd mJntTorque;

  // robot position
  Frame mCurrentTCPPosition;

  // cartesian velocity
  Vector6d mCartVelocity;

  // robot base position
  Frame mCurrentBasePosition;
};

class RobotCommand {
public:
  // constructor
  RobotCommand(int dof) {
    mJntPosition = VectorXd::Zero(dof);
    mJntVelocity = VectorXd::Zero(dof);
    mJntTorque = VectorXd::Zero(dof);
    mCartVelocity = VectorXd::Zero(6);
  }

  // destructor
  ~RobotCommand() {}

  // jnt pos
  VectorXd mJntPosition;

  // jnt vel
  VectorXd mJntVelocity;

  // jnt trq
  VectorXd mJntTorque;

  // robot position
  Frame mCurrentTCPPosition;

  // cartesian velocity
  Vector6d mCartVelocity;
};

class RobotDatabase {
public:
  // constructor
  RobotDatabase(int dof) {
    mCmd = new RobotCommand(dof);
    mState = new RobotState(dof);
  }

  // destructor
  ~RobotDatabase() {
    delete (mCmd);
    delete (mState);
  }

  // robot command
  RobotCommand *mCmd;

  // robot state
  RobotState *mState;
};

} // namespace arm
} // namespace io
} // namespace adi