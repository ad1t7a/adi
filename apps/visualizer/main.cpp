#include "common/converters.hpp"
#include "common/eigen_types.hpp"
#include "common/frame.hpp"
#include "multibody/inversekinematics/inversekinematics.hpp"
#include "visualization/urdf.hpp"

#include <chrono>
#include <thread>
int main() {
  // robot folder path
  std::string robotFolderPath = "/Users/ad1t7a/Developer/adi/robots/flexiv/";
  std::string robotFilePath = "robot.urdf";
  // clear visualizer
  adi::visualization::URDF robot(std::string("tcp://127.0.0.1:6000"));
  robot.sendZMQ(robot.createDeleteCmd());

  // load robot
  robot.setPathPrefix(robotFolderPath);
  robot.loadURDF(robotFilePath, false, "ad.jpg");

  adi::multibody::InverseKinematics ik(robotFolderPath + robotFilePath);

  unsigned int linkId = 7;

  adi::VectorXd seedJntPos = adi::VectorXd::Zero(linkId);
  seedJntPos << 0, adi::degToRad(-40), 0, adi::degToRad(-90), 0,
      adi::degToRad(45), 0;

  adi::Frame ikFrame;
  ikFrame.m_pos << -0.68, 0.11, 0.0;
  ikFrame.setUserEulerAngleZYX(0, adi::degToRad(180), 0);

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (true) {
    ik.solve(linkId, seedJntPos, ikFrame);
    // update visualization
    robot.syncVisualTransforms(seedJntPos);
    ikFrame.m_pos[0] = -0.4 + (0.15 * sin(x));
    ikFrame.m_pos[1] = 0.15 * sin(y);
    ikFrame.m_pos[2] = std::max(0.3 * sin(z), -0.15);
    x += 0.01;
    y += 0.02;
    z += 0.03;
  }

  return 0;
}