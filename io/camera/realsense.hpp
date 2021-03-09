#pragma once
#include "librealsense2/rs.hpp"
#include <string>

namespace adi {
namespace io {
namespace camera {
class Realsense {
public:
  // constructor
  Realsense();

  // destructor
  ~Realsense();

  // initialize
  void init();

  void step();

private:
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;

  // We want the points object to be persistent so we can display the last cloud
  // when a frame drops
  rs2::points points;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
};
} // namespace camera
} // namespace io
} // namespace adi