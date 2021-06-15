
#include "common/constants.hpp"
#include "common/converters.hpp"

#include "visualization/urdf.hpp"

#include "controllers/cartesian/differentialkinematics.hpp"

#include "io/arm/config.hpp"
#include "io/arm/urinterface.hpp"

#include "io/joystick/ps4.hpp"

#include "armcontrol.hpp"
// controller connection
const ControllerConnection kControllerConnection = ControllerConnection::NONE;
//! robot system
const RobotConnection kRobotConnection = RobotConnection::NONE;

// robot configuration
std::string prefixPath = "/Users/ad1t7a/Developer/adi/robots/flexiv/";
std::string robotPath = "robot.urdf";
// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";
// texture file name
std::string kTextureFile = "texture.jpg";
// robot IP Address
std::string kRobotIPAddress = "192.168.1.138";

int main() {
  // initialize visualizer
  adi::visualization::URDF urdf(kVisualizerIPAddress);
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
      robotDB.mCmd->mJntPosition << 0, adi::degToRad(-40), 0, adi::degToRad(90),
          0, adi::degToRad(45), 0;
      /*robotDB.mCmd->mJntPosition << 0, adi::degToRad(-90), adi::degToRad(-90),
          adi::degToRad(-90), adi::degToRad(90), adi::degToRad(0);*/
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
    case RobotConnection::FLEXIV: {
      // TODO
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
      case RobotConnection::FLEXIV: {
        // TODO
        break;
      }
      default: break;
    }

    // update based on commanded input
    switch (kControllerConnection) {
      case ControllerConnection::PS4 : {
        js.step();
        robotDB.mCmd->mCartVelocity << js.mSliderState.mSliderLeftHorizontal,
            js.mSliderState.mSliderLeftVertical,
            js.mSliderState.mSliderRightVertical, 0.0, 0.0, 0.0;
        break;
      }
      default: break;
    }

    diffIK.step(diffIK.getDoF(), robotDB.mState->mJntPosition,
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
        robotInterface->setCommand(*robotDB.mCmd);
        break;
      }
      case RobotConnection::FLEXIV: {
        // TODO
        break;
      }
      default: break;
    }

    // update visualization
    urdf.syncVisualTransforms(robotPosOriConfig);
  }
  return 0;
}