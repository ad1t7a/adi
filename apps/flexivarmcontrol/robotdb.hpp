#pragma once
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"

class RobotState {
public:
  ADI_NO_COPY_NO_MOVE_NO_ASSIGN(RobotState);

  // joint pos
  Eigen::VectorXd mJntPos;

  // joint velocity
  Eigen::VectorXd mJntVel;

  // joint torque
  Eigen::VectorXd mJntTrq;
};

class RobotCommand {
public:
  ADI_NO_COPY_NO_MOVE_NO_ASSIGN(RobotCommand);

  // joint pos
  Eigen::VectorXd mJntPos;

  // joint velocity
  Eigen::VectorXd mJntVel;

  // joint torque
  Eigen::VectorXd mJntTrq;
};
