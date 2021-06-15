#include "common/converters.hpp"
#include "common/eigen_types.hpp"
#include "common/frame.hpp"
#include "visualization/perception.hpp"

#include <chrono>
#include <thread>
// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";
int main() {
  std::string path = "/Users/ad1t7a/Developer/adi/robots/objects/meshes/"
                     "003_cracker_box_textured.png";
  adi::visualization::Perception perception(kVisualizerIPAddress);
  perception.loadTexture(path);
  int color_rgb = 0xffffff;
  double world_pos[3] = {0, 0, 0};
  perception.sendZMQ(perception.createTextureCmd(
      perception.m_textureData.c_str(), world_pos, color_rgb, "/adi/aaa"));
  return 0;
}