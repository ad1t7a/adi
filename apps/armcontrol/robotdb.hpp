#pragma once
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"

class RobotState {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RobotState);

  // constructor
  RobotState() {}

  // destructor
  ~RobotState() {}

  // joint pos
  Eigen::VectorXd mJntPos;

  // joint velocity
  Eigen::VectorXd mJntVel;

  // joint torque
  Eigen::VectorXd mJntTrq;
};

class RobotCommand {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RobotCommand);

  // constructor
  RobotCommand() {}

  // destructor
  ~RobotCommand() {}

  // joint pos
  Eigen::VectorXd mJntPos;

  // joint velocity
  Eigen::VectorXd mJntVel;

  // joint torque
  Eigen::VectorXd mJntTrq;
};

class RobotDatabase {
public:
  ADI_NO_COPY_NO_MOVE_NO_ASSIGN(RobotDatabase);

  static RobotDatabase *mInstance;

  // robot state
  RobotState mRobotState;

  // robot command
  RobotCommand mRobotCmd;

  // get instance
  static RobotDatabase *getInstance() {
    if (!mInstance) {
      mInstance = new RobotDatabase;
    }
    return mInstance;
  }

private:
  // constructor
  RobotDatabase() {}
};
