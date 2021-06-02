#include "common/frame.hpp"
#include "visualization/urdf.hpp"
#include <chrono>
#include <thread>

RigidBodyDynamics::Math::SpatialTransform
getSpatialTransform(adi::Frame frame) {
  RigidBodyDynamics::Math::SpatialTransform transform;
  transform.r = frame.m_pos;
  transform.E = frame.m_rot;
  return transform;
}

int main() {
  // clear visualizer
  adi::visualization::URDF object(std::string("tcp://127.0.0.1:6000"));
  object.sendZMQ(object.createDeleteCmd());

  // object visualization
  object.setPathPrefix("/Users/ad1t7a/Developer/adi/robots/objects/");
  object.loadURDF("003_cracker_box_textured.urdf", false,
                  "meshes/003_cracker_box_textured.png");
  adi::Frame X_WorldPObject;
  object.syncVisualTransforms(0, getSpatialTransform(X_WorldPObject));

  //! gripper visualization
  adi::visualization::URDF endEffector(std::string("tcp://127.0.0.1:6000"));
  endEffector.setPathPrefix("/Users/ad1t7a/Developer/adi/robots/robotiq/");
  endEffector.loadURDF("gripper.urdf", false, "ad.jpg");
  adi::Frame X_WorldPGripper;

  //! gripper WRT object grasp pose
  adi::Frame X_ObjectPGripper;
  X_ObjectPGripper.m_pos = {-0.009, 0.0, 0.31};
  X_ObjectPGripper.m_rot *= -1.0;
  adi::Frame X_GripperRotation;
  X_GripperRotation.setRotVec(adi::Vector4d(0.7071068, 0, 0, 0.7071068));
  X_ObjectPGripper = X_ObjectPGripper * X_GripperRotation;

  //! grasp pose
  adi::Frame XPoseGripperGrasp = X_WorldPObject * X_ObjectPGripper;

  //! pregrasp pose
  adi::Frame XPregraspGrasp;
  XPregraspGrasp.m_pos = {0, 0, 0.1};
  adi::Frame XPoseGripperPreGrasp =
      XPoseGripperGrasp * XPregraspGrasp.inverse();

  //! place pose
  adi::Frame XObjectPlace;
  XObjectPlace.m_pos = {0.3, 0.3, 0.0};
  XObjectPlace.setRotVec(adi::Vector4d(0.7071068, 0, 0, 0.7071068));
  adi::Frame XGripperPlace = XObjectPlace * X_ObjectPGripper;

  //! preplace pose
  adi::Frame XPrePlaceGripperPlace;
  XPrePlaceGripperPlace.m_pos = {0, 0, 0.1};
  adi::Frame XGripperPrePlace = XGripperPlace * XPrePlaceGripperPlace.inverse();

  // add midpoint frame
  adi::Frame XPregraspPrePlace =
      XPoseGripperPreGrasp.inverse() * XGripperPrePlace;
  adi::AngleAxisd angle(XPregraspPrePlace.getAxisAngleRotation().angle() / 2,
                        XPregraspPrePlace.getAxisAngleRotation().axis());
  XPregraspPrePlace.m_rot = angle.toRotationMatrix();
  XPregraspPrePlace.m_pos /= 2;
  adi::Frame XPoseGraspMidpoint = XPoseGripperPreGrasp * XPregraspPrePlace;
  // add all frames
  std::vector<adi::Frame> keyframes;
  keyframes.push_back(XPoseGripperPreGrasp);
  keyframes.push_back(XPoseGripperGrasp);
  keyframes.push_back(XPoseGripperPreGrasp);
  keyframes.push_back(XPoseGraspMidpoint);
  keyframes.push_back(XGripperPrePlace);
  keyframes.push_back(XGripperPlace);

  for (size_t i = 0; i < keyframes.size(); i++) {
    endEffector.syncVisualTransforms(0, getSpatialTransform(keyframes[i]));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  return 0;
}