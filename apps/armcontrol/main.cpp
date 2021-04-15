#include "common/constants.hpp"
#include "common/converters.hpp"
#include "controllers/differentialkinematics.hpp"
#include "robotdb.hpp"
#include "visualization/urdf.hpp"

// robot configuration
std::string prefixPath = "/Users/ad1t7a/Developer/adi/robots/flexiv/";
std::string robotPath = "robot.urdf";

// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";

// timestep for calculation
const double kcanonicalTimestep = 0.001;

// link index
unsigned int kControlLinkIndex = 7;

int main() {
  // initialize robot start position
  Eigen::VectorXd robotPosOri = Eigen::VectorXd::Zero(adi::kCartPoseDofs);

  // initialize controller
  adi::controllers::DifferentialKinematics diffIK(prefixPath + robotPath);
  // initialize time
  double t = 0.0;

  // initialize visualizer
  adi::visualization::URDF urdf(kVisualizerIPAddress);
  urdf.setPathPrefix(prefixPath);
  urdf.loadURDF(robotPath, true, "texture.jpg", "");

  // initialize robot start joint angles (TODO: Replace with middleware and
  // simulator) and velocity
  Eigen::VectorXd robotJntPos(diffIK.getDoF());
  robotJntPos << 0, adi::degToRad(0), 0, adi::degToRad(0), 0,
      adi::degToRad(0.001), 0;
  Eigen::VectorXd robotJntVel(diffIK.getDoF());

  // update visualization
  Eigen::VectorXd robotPosOriConfig(urdf.getDoF());
  robotPosOriConfig << robotPosOri, robotJntPos;

  // cartesian velocity (ori, pos)
  Eigen::VectorXd cmdCartVel(adi::kCartPoseDofs);
  cmdCartVel << 0.0, 0.0, 0.0, 0.0, 0.0, -1.0;

  // joint velocity
  Eigen::VectorXd jntVelocity(diffIK.getDoF());

  while (true) {

    diffIK.step(kControlLinkIndex, robotJntPos, robotJntVel, cmdCartVel,
                jntVelocity);
    robotJntPos += kcanonicalTimestep * jntVelocity;
    robotPosOriConfig << robotPosOri, robotJntPos;
    robotJntVel = jntVelocity;
    // update visualization
    urdf.syncVisualTransforms(robotPosOriConfig);

    // update timestep
    t += kcanonicalTimestep;
  }

  return 0;
}