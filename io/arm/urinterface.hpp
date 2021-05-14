#pragma once
#include "common/adi_assert.hpp"
#include "io/arm/robotinterface.hpp"
#include "ur_driver.h"

namespace adi {
namespace io {
namespace arm {
class URInterface : public RobotInterface {
public:
  // constructor
  URInterface(std::string host, int reverse_port = 50001,
              int g_controlLoopTimeIntervalms = 10) {
    // constructor
    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    mURRobot =
        new UrDriver(rt_msg_cond_, msg_cond_, host, reverse_port,
                     (double(g_controlLoopTimeIntervalms)) / 1000.0, 300);
  }

  //! destructor
  ~URInterface() { delete (mURRobot); }

  // start robot
  virtual void startRobot() override;

  // update state
  virtual void updateState(RobotState &state) override;

private:
  // ur driver
  UrDriver *mURRobot;
};
} // namespace arm
} // namespace io
} // namespace adi