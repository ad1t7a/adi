#pragma once
namespace adi {
namespace io {
namespace joystick {

// slider index
enum class SliderIndex { LH = 0, LV = 1, RH = 2, LB = 3, RB = 4, RV = 5 };

// Joystick Slider
struct JoystickSliders {
  // left horizontal slider
  double mSliderLeftHorizontal;

  // left vertical slider
  double mSliderLeftVertical;

  // right horizontal slider
  double mSliderRightHorizontal;

  // right vertical slider
  double mSliderRightVertical;

  // left button slider
  double mSliderLeftButton;

  // right button slider
  double mSliderRightButton;
};

// Joystick Buttons
struct JoystickButtons {
  // up arrow key
  bool mArrowKeyUp;

  // down arrow key
  bool mArrowKeyDown;

  // left arrow key
  bool mArrowKeyLeft;

  // right arrow key
  bool mArrowKeyRight;

  // triangle button
  bool mTriangleBtn;

  // square button
  bool mSquareBtn;

  // circle button
  bool mCircleBtn;

  // X button
  bool mXBtn;
};
} // namespace joystick
} // namespace io
} // namespace adi
