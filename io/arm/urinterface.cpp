#include "io/arm/urinterface.hpp"

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
  return;
}
} // namespace arm
} // namespace io
} // namespace adi