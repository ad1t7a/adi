#include "common/constants.hpp"
#include "common/converters.hpp"
#include "io/joystick/ps4.hpp"
#include "visualization/urdf.hpp"

// Robot connection
enum class RobotConnection {
  NONE,
  SIM,
  UR5,
};

//! Controller Connection
enum class ControllerConnection {
  NONE,
  PS4,
  AUTONOMOUS,
};