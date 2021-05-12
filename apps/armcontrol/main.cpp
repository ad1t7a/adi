#include "common/constants.hpp"
#include "common/converters.hpp"
#include "controllers/cartesian/differentialkinematics.hpp"
#include "io/arm/config.hpp"
#include "io/joystick/ps4.hpp"
#include "visualization/urdf.hpp"

// robot configuration
std::string prefixPath = "/Users/ad1t7a/Developer/adi/robots/flexiv/";
std::string robotPath = "robot.urdf";

// visualizer IP address
std::string kVisualizerIPAddress = "tcp://127.0.0.1:6000";

// texture file name
std::string kTextureFile = "texture.jpg";

// link index
unsigned int kControlLinkIndex = 7;

int main() {

  // initialize visualizer
  adi::visualization::URDF urdf(kVisualizerIPAddress);
  urdf.setPathPrefix(prefixPath);
  urdf.loadURDF(robotPath, true, kTextureFile, "");

  // initialize controller
  adi::controllers::DifferentialKinematics diffIK(prefixPath + robotPath);

  // create robot state database
  adi::io::arm::RobotDatabase robotDB(diffIK.getDoF());

  // initialize joystick
  adi::io::joystick::PS4 js;
  js.init();

  // initialize robot start joint angles (TODO: Replace with middleware and
  // simulator) and velocity
  robotDB.mState->mJntPosition << 0, adi::degToRad(-40), 0, adi::degToRad(-90),
      0, adi::degToRad(40), 0;

  while (true) {
    js.step();
    robotDB.mCmd->mCartVelocity << 0.0, 0.0, 0.0,
        js.mSliderState.mSliderLeftHorizontal,
        js.mSliderState.mSliderLeftVertical,
        js.mSliderState.mSliderRightVertical;

    diffIK.step(kControlLinkIndex, robotDB.mState->mJntPosition,
                robotDB.mState->mJntVelocity, robotDB.mCmd->mCartVelocity,
                robotDB.mCmd->mJntVelocity);
    robotDB.mCmd->mJntPosition = robotDB.mState->mJntPosition +
                                 (adi::kTimestep * robotDB.mCmd->mJntVelocity);

    // update visualization
    Eigen::VectorXd robotPosOriConfig(urdf.getDoF());
    robotPosOriConfig << robotDB.mCmd->mCurrentBasePosition.m_pos,
        robotDB.mCmd->mCurrentBasePosition.getEulerAngleZYX(),
        robotDB.mCmd->mJntPosition;

    // update sensor from cmd
    robotDB.mState->mJntVelocity = robotDB.mCmd->mJntVelocity;
    robotDB.mState->mJntPosition = robotDB.mCmd->mJntPosition;

    // update visualization
    urdf.syncVisualTransforms(robotPosOriConfig);
  }
  return 0;
}