#include "common/frame.hpp"

namespace adi {
Frame::Frame() {
  m_rot = Matrix3d::Identity();
  m_pos = Vector3d::Zero();
  m_userEulerAngle = Vector3d::Zero();
}

Frame::Frame(const Vector3d &pos, const Matrix3d &rot) {
  m_pos = pos;
  m_rot = rot;
  m_userEulerAngle = getEulerAngleZYX();
}

Frame::~Frame() {}

Vector3d Frame::operator*(const Vector3d &v) const { return m_rot * v + m_pos; }

Frame Frame::operator*(const Frame &f) const {
  Frame newFrame;
  newFrame.m_rot = m_rot * f.m_rot;
  newFrame.m_pos = m_rot * f.m_pos + m_pos;
  return newFrame;
}

void Frame::operator=(const Frame &f) {
  m_rot = f.m_rot;
  m_pos = f.m_pos;
  m_userEulerAngle = f.m_userEulerAngle;
}

Frame Frame::inverse() const {
  Frame newFrame;
  newFrame.m_rot = m_rot.transpose();
  newFrame.m_pos = -newFrame.m_rot * m_pos;
  return newFrame;
}

Vector7d Frame::getPoseVec() const {
  Vector7d vec;
  vec.block(0, 0, 3, 1) = m_pos;
  vec.block(3, 0, 4, 1) = getQuatVec();
  return vec;
}

void Frame::setPoseVec(const Vector7d &vec) {
  m_pos = vec.block(0, 0, 3, 1);
  Vector4d quat = vec.block(3, 0, 4, 1);
  setRotVec(quat);
}

Quaterniond Frame::getQuat() const {
  Quaterniond q(m_rot);
  return q;
}

Vector4d Frame::getQuatVec() const {
  Quaterniond q(m_rot);
  Vector4d x;
  x << q.w(), q.x(), q.y(), q.z();
  return x;
}

void Frame::setRotVec(const Vector4d &rotVec) {
  Quaterniond q(rotVec(0), rotVec(1), rotVec(2), rotVec(3));
  m_rot = q.toRotationMatrix();
  m_userEulerAngle = getEulerAngleZYX();
}

Vector3d Frame::getEulerAngleZYX() const { return m_rot.eulerAngles(2, 1, 0); }

void Frame::setEulerAngleZYX(const double thetaZ, const double thetaY,
                             const double thetaX) {
  m_rot = AngleAxisd(thetaZ, Vector3d::UnitZ()) *
          AngleAxisd(thetaY, Vector3d::UnitY()) *
          AngleAxisd(thetaX, Vector3d::UnitX());
}

Vector3d Frame::getUserEulerAngleZYX() const { return m_userEulerAngle; }

void Frame::setUserEulerAngleZYX(const double thetaZ, const double thetaY,
                                 const double thetaX) {
  m_userEulerAngle(0) = thetaZ;
  m_userEulerAngle(1) = thetaY;
  m_userEulerAngle(2) = thetaX;
  setEulerAngleZYX(thetaZ, thetaY, thetaX);
}

void Frame::setAxisAngleRotation(const Vector3d &rotV) {
  Vector3d rotationAxis = rotV;
  rotationAxis.normalize();
  double angle = rotV.norm();
  AngleAxisd rot(angle, rotationAxis);
  m_rot = rot.matrix();
}

AngleAxisd Frame::getAxisAngleRotation() {
  AngleAxisd axisAngle(m_rot);
  return axisAngle;
}

void Frame::orthonormalizeRotationMatrix() {
  Matrix3d Q = m_rot.householderQr().householderQ();
  Matrix3d R = Q.transpose() * m_rot;

  // sign diagonal matrix
  Matrix3d S = Matrix3d::Identity();

  // correct with sign
  for (int k = 0; k < 3; k++) {
    if (R(k, k) < 0) {
      S(k, k) = -1.0;
    }
  }

  // assign to rotation matrix
  m_rot = Q * S;
}

void Frame::clear() {
  m_pos = Vector3d::Zero();
  m_rot = Matrix3d::Identity();
  m_userEulerAngle = Vector3d::Zero();
}

std::string Frame::str() const {
  std::stringstream s;
  s << "\ntranslation = " << m_pos.transpose();
  s << "\nrotation = \n" << m_rot;
  s << "\nEulerAngle = \n" << getEulerAngleZYX();

  return s.str();
}

//==============================================================================
/*!
   compute the eular angle difference between target pose and current pose

  \params[in] targetPose   : target pose [m]
  \return  Eular angle difference (target - current) [rad]
*/
//==============================================================================
Vector3d Frame::computeEularAngleDistance(const Frame &targetPose) const {
  Vector3d delta = targetPose.getEulerAngleZYX() - getEulerAngleZYX();

  for (unsigned int i = 0; i < delta.size(); i++) {
    if (delta[i] > kPI) {
      delta[i] = ktPI - delta[i];
    } else if (delta[i] < -kPI) {
      delta[i] += ktPI;
    }
  }

  return delta;
}

/*!
 * \brief Frame::computePosDistance
 * \param[in] targetPos : target pose [m]
 * \return distance (target - current)
 */
double Frame::computePosDistance(const Frame &targetPos) const {
  return sqrt(
      (targetPos.m_pos[0] - m_pos[0]) * (targetPos.m_pos[0] - m_pos[0]) +
      (targetPos.m_pos[1] - m_pos[1]) * (targetPos.m_pos[1] - m_pos[1]) +
      (targetPos.m_pos[2] - m_pos[2]) * (targetPos.m_pos[2] - m_pos[2]));
}

//==============================================================================
/*!
   check if current and target poses are the same (similar)

  \params[in] pose     : target pose [rad][m]
  \params[in] posError : target position tolerance [m]
  \params[in] oriError : target orientation tolerance (quaternion distance)
  \return  true if the pose are within the tolerance
*/
//==============================================================================
bool Frame::isEqual(const Frame &pose, const double posError,
                    const double oriError) const {
  // check position difference
  double posDiff = (m_pos - pose.m_pos).norm();
  if (posDiff > posError) {
    return false;
  }

  // compute quaternion distance
  Vector4d q1 = getQuatVec();
  Vector4d q2 = pose.getQuatVec();
  if (computeQuaternionDistance(q1, q2) > oriError) {
    return false;
  }

  return true;
}

//==============================================================================
/*!
  compute the velocity from current frame

  \params[in] pose     : target pose [rad][m]
  \params[in] t        : time  [sec]
  \return  vector with velocities from current frame
*/
//==============================================================================
Vector6d Frame::computeRelativeVelocity(const Frame &f, double t) const {
  Vector6d velocity;
  Vector3d linearVelocity = m_rot.inverse() * (m_pos - f.m_pos) / t;

  AngleAxisd newAngleAxis(m_rot.inverse() * f.m_rot);
  Vector3d angularVelocity =
      m_rot.inverse() * m_rot * newAngleAxis.angle() * newAngleAxis.axis() / t;
  velocity << linearVelocity, angularVelocity;
  return velocity;
}

double Frame::computeQuaternionDistance(const Vector4d &v1,
                                        const Vector4d &v2) const {
  double distA = (v1 + v2).norm();
  double distB = (v1 - v2).norm();

  if (distA > distB) {
    return 2 * distB;
  }
  return 2 * distA;
}

} // namespace adi