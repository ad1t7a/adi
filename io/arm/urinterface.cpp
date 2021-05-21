#include "io/arm/urinterface.hpp"
#include "common/eigen_types.hpp"

namespace adi {
namespace io {
namespace arm {
void URInterface::startRobot() {
  if (false == mURRobot->start()) {
    spdlog::error("Unable to start UR.");
  }
  sleep(2);

  // check if the robot already reached ready state
  if (!mURRobot->sec_interface_->robot_state_->isReady()) {
    if (!mURRobot->sec_interface_->robot_state_->isPowerOnRobot()) {
      spdlog::error("Cannot remote control UR5: Robot arm is not powered on.");
      return;
    }

    if (!mURRobot->sec_interface_->robot_state_->isRealRobotEnabled()) {
      spdlog::error("Cannot remote control UR5: Robot is not enabled.");
      return;
    }
    spdlog::error("Cannot remote control UR5. (Debug: Robot mode is %d",
                  mURRobot->sec_interface_->robot_state_->getRobotMode());
    return;
  }

  // check emergency stop
  if (mURRobot->sec_interface_->robot_state_->isEmergencyStopped()) {
    spdlog::error("Cannot remote control UR5: Robot is emergency stopped");
    return;
  }

  // check protective stop
  if (mURRobot->sec_interface_->robot_state_->isProtectiveStopped()) {
    spdlog::error("Cannot remote control UR5: Robot is protective stopped.");
    return;
  }
  int query_count = 5;
  while (query_count--) {
    if (false == mURRobot->rt_interface_->connected_) {
      sleep(1);
    }
  }
  if (0 == query_count) {
    spdlog::error("rt_interface_ connect failed");
  }
  mURRobot->uploadProg(false);
  return;
}

void URInterface::updateState(RobotState &state) {
  // get joint sensor info from UR5
  state.mJntPosition = adi::StdVecToEigenVec<double>(
      mURRobot->rt_interface_->robot_state_->getQActual());
  state.mJntVelocity = adi::StdVecToEigenVec<double>(
      mURRobot->rt_interface_->robot_state_->getQdActual());
  state.mJntTorque = adi::StdVecToEigenVec<double>(
      mURRobot->rt_interface_->robot_state_->getMTarget());
}

void URInterface::setCommand(RobotCommand &cmd) {
  std::vector<double> cmdPosition(cmd.mJntPosition.data(),
                                  cmd.mJntPosition.data() +
                                      cmd.mJntPosition.size());
  mURRobot->servoj(cmdPosition, 1);
}

} // namespace arm
} // namespace io
} // namespace adi