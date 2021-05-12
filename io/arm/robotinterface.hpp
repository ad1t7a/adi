#pragma once
#include "common/adi_assert.hpp"

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
    spdlog::error("Cannot remote control UR5: Robot is not enabled.");
  }
};
} // namespace arm
} // namespace io
} // namespace adi