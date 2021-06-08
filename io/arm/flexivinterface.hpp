#pragma once
#include "FvrRobotClient.hpp"
#include "common/adi_assert.hpp"
#include "io/arm/robotinterface.hpp"

namespace adi {
namespace io {
namespace arm {
class FlexivInterface : public RobotInterface {
public:
  // constructor
  FlexivInterface() {
    // constructor
  }

  //! destructor
  ~FlexivInterface() {}

  // start robot
  virtual void startRobot() override;

  // update state
  virtual void updateState(RobotState &state) override;

  // set robot command
  virtual void setCommand(RobotCommand &cmd) override;
};
} // namespace arm
} // namespace io
} // namespace adi