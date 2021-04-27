
#include "realsense.hpp"

#include <fstream>
#include <streambuf>
#include <string>

namespace adi {
namespace io {
namespace camera {
// constructor
Realsense::Realsense() {}

// destructor
Realsense::~Realsense() {}

// initialize
void Realsense::init() { mPipe.start(); }

// step
void Realsense::step() {
  try {
    // Wait for the next set of frames from the camera
    auto frames = mPipe.wait_for_frames();
    auto color = frames.get_color_frame();

    // Tell pointcloud object to map to this color frame
    mPc.map_to(color);
    auto depth = frames.get_depth_frame();
    // Generate the pointcloud and texture mappings
    mPoints = mPc.calculate(depth);
  } catch (const rs2::error &e) {
    return;
  } catch (const std::exception &e) {
    return;
  }
}

} // namespace camera
} // namespace io
} // namespace adi