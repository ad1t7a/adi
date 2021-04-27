#include "common/constants.hpp"
#include "common/converters.hpp"
#include "controllers/differentialkinematics.hpp"
#include "io/joystick/ps4.hpp"
#include "robotdb.hpp"
#include "visualization/urdf.hpp"
#include <chrono>

// robot configuration
std::string prefixPath = "/Users/ad1t7a/Developer/adi/robots/flexiv/";
std::string robotPath = "robot.urdf";

// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";

// link index
unsigned int kControlLinkIndex = 7;

int main() {
  // initialize robot start position
  Eigen::VectorXd robotPosOri = Eigen::VectorXd::Zero(adi::kCartPoseDofs);

  // initialize controller
  adi::controllers::DifferentialKinematics diffIK(prefixPath + robotPath);

  // initialize visualizer
  adi::visualization::URDF urdf(kVisualizerIPAddress);
  urdf.setPathPrefix(prefixPath);
  urdf.loadURDF(robotPath, true, "texture.jpg", "");

  // initialize robot start joint angles (TODO: Replace with middleware and
  // simulator) and velocity
  Eigen::VectorXd robotJntPos(diffIK.getDoF());
  robotJntPos << 0, adi::degToRad(-40), 0, adi::degToRad(-90), 0,
      adi::degToRad(40), 0;
  Eigen::VectorXd robotJntVel(diffIK.getDoF());

  // update visualization
  Eigen::VectorXd robotPosOriConfig(urdf.getDoF());
  robotPosOriConfig << robotPosOri, robotJntPos;

  // cartesian velocity (ori, pos)
  Eigen::VectorXd cmdCartVel(adi::kCartPoseDofs);

  // joint velocity
  Eigen::VectorXd jntVelocity(diffIK.getDoF());

  adi::io::joystick::PS4 js;
  js.init();
  while (true) {
    js.step();
    cmdCartVel << 0.0, 0.0, 0.0, js.mSliderState.mSliderLeftHorizontal,
        js.mSliderState.mSliderLeftVertical,
        js.mSliderState.mSliderRightVertical;
    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    diffIK.step(kControlLinkIndex, robotJntPos, robotJntVel, cmdCartVel,
                jntVelocity);
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                       begin)
                     .count()
              << "[µs]" << std::endl;
    robotJntPos += adi::kTimestep * jntVelocity;
    robotPosOriConfig << robotPosOri, robotJntPos;
    robotJntVel = jntVelocity;

    // update visualization
    urdf.syncVisualTransforms(robotPosOriConfig);
  }
  return 0;
}