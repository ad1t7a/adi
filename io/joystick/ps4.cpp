#include "io/joystick/ps4.hpp"
#include <iostream>
namespace adi {
namespace io {
namespace joystick {

// constructor
PS4::PS4() {
  if (!glfwInit()) {
    spdlog::error("Unable to initialize glfw.");
  }
}

// destructor
PS4::~PS4() { glfwTerminate(); }

// initialize
void PS4::init(int joystickIndex) {
  mIndex = joystickIndex;
  if (!glfwJoystickPresent(joystickIndex)) {
    spdlog::error("Unable to initialize joystick index : %d", mIndex);
  }
  const char *name = glfwGetJoystickName(mIndex);
  mName = std::string(name);
  mOnline = true;
  // get offset
  int axesCount;
  const float *axes = glfwGetJoystickAxes(mIndex, &axesCount);
  if (axes == NULL) {
    mOnline = false;
  }
  for (size_t i = 0; i < 100; i++) {
    updateState(mSliderOffset);
  }
  spdlog::info("Initialized joystick");
}

// step
void PS4::step() {
  updateState(mSliderState);
  normalize();
}

// update system state
void PS4::updateState(JoystickSliders &sliderState) {
  // get current slider state
  int axesCount;
  const float *axes = glfwGetJoystickAxes(mIndex, &axesCount);
  if (axes == NULL) {
    mOnline = false;
  }
  sliderState.mSliderLeftHorizontal = axes[static_cast<int>(SliderIndex::LH)];
  sliderState.mSliderLeftVertical = axes[static_cast<int>(SliderIndex::LV)];
  sliderState.mSliderRightHorizontal = axes[static_cast<int>(SliderIndex::RH)];
  sliderState.mSliderRightVertical = axes[static_cast<int>(SliderIndex::RV)];
  sliderState.mSliderLeftButton = axes[static_cast<int>(SliderIndex::LB)];
  sliderState.mSliderRightButton = axes[static_cast<int>(SliderIndex::RB)];
}

// normalize
void PS4::normalize() {
  mSliderState.mSliderLeftHorizontal -= mSliderOffset.mSliderLeftHorizontal;
  mSliderState.mSliderLeftVertical -= mSliderOffset.mSliderLeftVertical;
  mSliderState.mSliderRightHorizontal -= mSliderOffset.mSliderRightHorizontal;
  mSliderState.mSliderRightVertical -= mSliderOffset.mSliderRightVertical;
}

} // namespace joystick
} // namespace io
} // namespace adi