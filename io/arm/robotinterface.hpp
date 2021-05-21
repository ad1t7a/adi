#pragma once
#include "common/adi_assert.hpp"
#include "io/arm/config.hpp"

namespace adi {
namespace io {
namespace arm {
class RobotInterface {
public:
  RobotInterface() {
    // constructor
  }

  ~RobotInterface() {
    //! destructor
  }

  virtual void startRobot() {
    spdlog::error("Cannot remote control robot: Robot is not enabled.");
  }

  virtual void updateState(RobotState &state) {
    spdlog::error("Cannot update state: Robot not described.");
  }

  virtual void setCommand(RobotCommand &cmd) {
    spdlog::error("Cannot set command: Robot not described.");
  }
};
} // namespace arm
} // namespace io
} // namespace adi