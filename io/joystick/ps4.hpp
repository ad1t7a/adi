#pragma once
#include "io/joystick/ps4state.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include <GLFW/glfw3.h>

namespace adi {
namespace io {
namespace joystick {
class PS4 {
public:
  // constructor
  PS4();

  // destructor
  ~PS4();

  // initialize
  void init(int joystickIndex = GLFW_JOYSTICK_1);

  // step
  void step();

  // slider state
  JoystickSliders mSliderState;

  // joystick status
  bool mOnline;

private:
  // update system state
  void updateState(JoystickSliders &sliderState);

  // normalize
  void normalize();

  // slider offset
  JoystickSliders mSliderOffset;

  // joystick name
  std::string mName;

  // joystick index
  int mIndex;
};
} // namespace joystick
} // namespace io
} // namespace adi