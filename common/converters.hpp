#pragma once

#include "constants.hpp"

namespace adi {
// degree/radian conversion
double degToRad(double deg) { return deg / 180.0 * kPI; }

// radian/degree conversion
double radToDeg(double rad) { return rad / kPI * 180.0; }

} // namespace adi
