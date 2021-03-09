#pragma once

#include "eigen_types.hpp"
#include <string>

namespace adi {

//! Quaternion size
constexpr int kQuaternionSize = 4;

//! Space dimensions
constexpr int kSpaceDimension = 3;

//! Roll Pitch Yaw size
constexpr int kRPYSize = 3;

//! Pi
constexpr double kPI = 3.14159265359;

//! 2 * Pi
constexpr double ktPI = 2.0 * kPI;

//! 1/sqrt(3)
constexpr double koneDivSqrt3 = 0.57735026919;

//! sqrt(3)/2
constexpr double ksqrt3Div2 = 0.86602540378;

//! acceleration due to gravity
constexpr double kaccGravity = -9.8;

//! cartesian position degrees of freedom
constexpr double kCartPoseDofs = 6;

//! timestep [s]
constexpr double kTimestep = 0.001;

//! cartesian pos accuracy
constexpr double kCartPosAccuracy = 0.0001;

//! cartesian pos accuracy
constexpr double kCartOriAccuracy = 0.0001;

//! visualizer path
static std::string kVisualizerPath = "/adi/";

} // namespace adi
