#include "armcontrol.hpp"

#include "common/factorial.hpp"
#include "controllers/cartesian/differentialkinematics.hpp"
#include "io/arm/config.hpp"
#include "io/arm/urinterface.hpp"

// controller connection
const ControllerConnection kControllerConnection = ControllerConnection::PS4;
//! system
const RobotConnection kRobotConnection = RobotConnection::NONE;

// robot configuration
std::string prefixPath = "/Users/ad1t7a/Developer/adi/robots/flexiv/";
std::string robotPath = "robot.urdf";
// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";
// texture file name
std::string kTextureFile = "texture.jpg";
// robot IP Address
std::string kRobotIPAddress = "192.168.2.50";

// link index
unsigned int kControlLinkIndex = 7;

int main() {
  // initialize visualizer
  /*adi::visualization::URDF urdf(kVisualizerIPAddress);
  urdf.setPathPrefix(prefixPath);
  urdf.loadURDF(robotPath, true, kTextureFile, "");

  // initialize controller
  adi::controllers::DifferentialKinematics diffIK(prefixPath + robotPath);

  // create robot state database
  adi::io::arm::RobotDatabase robotDB(diffIK.getDoF());

  // robot interface
  adi::io::arm::RobotInterface* robotInterface;

  // initialize controllers
  adi::io::joystick::PS4 js;
  switch (kControllerConnection) {
    case ControllerConnection::NONE : {
      break;
    }
    case ControllerConnection::PS4 : {
      js.init();
      break;
    }
    case ControllerConnection::AUTONOMOUS : {
      break;
    }
    default: break;
  }

  // update cmd
  switch (kRobotConnection) {
    case RobotConnection::NONE : {
      robotDB.mState->mJntVelocity = robotDB.mCmd->mJntVelocity;
      robotDB.mState->mJntPosition = robotDB.mCmd->mJntPosition;
      break;
    }
    case RobotConnection::SIM : {
      break;
    }
    case RobotConnection::UR5 : {
      robotInterface = new adi::io::arm::URInterface(kRobotIPAddress);
      robotInterface->startRobot();
      break;
    }
    default: break;
  }

  while (true) {
    // update cmd
    switch (kRobotConnection) {
      case RobotConnection::NONE : {
        robotDB.mState->mJntVelocity = robotDB.mCmd->mJntVelocity;
        robotDB.mState->mJntPosition = robotDB.mCmd->mJntPosition;
        break;
      }
      case RobotConnection::SIM : {
        break;
      }
      case RobotConnection::UR5 : {
        robotInterface->updateState(*robotDB.mState);
        break;
      }
      default: break;
    }

    // update based on commanded input
    switch (kControllerConnection) {
      case ControllerConnection::PS4 : {
        js.step();
        robotDB.mCmd->mCartVelocity << 0.0, 0.0, 0.0,
                                    js.mSliderState.mSliderLeftHorizontal,
                                    js.mSliderState.mSliderLeftVertical,
                                    js.mSliderState.mSliderRightVertical;
        break;
      }
      default: break;
    }

    diffIK.step(kControlLinkIndex, robotDB.mState->mJntPosition,
                robotDB.mState->mJntVelocity, robotDB.mCmd->mCartVelocity,
                robotDB.mCmd->mJntVelocity);

    robotDB.mCmd->mJntPosition = robotDB.mState->mJntPosition +
                                 (adi::kTimestep * robotDB.mCmd->mJntVelocity);

    // update visualization
    Eigen::VectorXd robotPosOriConfig(urdf.getDoF());
    robotPosOriConfig << robotDB.mState->mCurrentBasePosition.m_pos,
        robotDB.mState->mCurrentBasePosition.getEulerAngleZYX(),
        robotDB.mState->mJntPosition;

    // update cmd
    switch (kRobotConnection) {
      case RobotConnection::NONE : {
        robotDB.mState->mJntVelocity = robotDB.mCmd->mJntVelocity;
        robotDB.mState->mJntPosition = robotDB.mCmd->mJntPosition;
        break;
      }
      case RobotConnection::SIM : {
        break;
      }
      case RobotConnection::UR5 : {
        break;
      }
      default: break;
    }

    // update visualization
    urdf.syncVisualTransforms(robotPosOriConfig);
  }*/
  return 0;
}