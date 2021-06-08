#include "common/converters.hpp"
#include "common/eigen_types.hpp"
#include "common/frame.hpp"
#include "example.hpp"
#include "io/camera/realsense.hpp"
#include <chrono>
#include <thread>

// Helper functions
void register_glfw_callbacks(window &app, glfw_state &app_state);
// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";

int main(int argc, char *argv[]) {
  // Create a simple OpenGL window for rendering:
  window app(1280, 720, "RealSense Pointcloud Example");
  // Construct an object to manage view state
  glfw_state app_state;
  // register callbacks to allow manipulation of the pointcloud
  register_glfw_callbacks(app, app_state);

  adi::io::camera::Realsense mmap;
  while (app) {
    // Upload the color frame to OpenGL
    auto color = mmap.loadPointCloud();
    app_state.tex.upload(color);
    auto pts = mmap.getPoints();
    adi::io::camera::pcl_ptr cloud = mmap.pointsToPcl(pts);

    // Draw the pointcloud
    draw_pointcloud(app.width(), app.height(), app_state, pts);
  }
  return 0;
}