#pragma once
#include "common/constants.hpp"
#include "common/eigen_types.hpp"
#include <sstream>

namespace adi {

class Frame {
public:
  Frame();

  Frame(const Vector3d &pos, const Matrix3d &rot);

  virtual ~Frame();

  // OPERATORS
  //--------------------------------------------------------------------------

  //! multiply vector
  Vector3d operator*(const Vector3d &v) const;

  //! multiply frame
  Frame operator*(const Frame &f) const;

  //! equal
  void operator=(const Frame &f);

  // Basic functions
  //--------------------------------------------------------------------------

  //! check if current and target poses are the same (within tolerance)
  bool isEqual(const Frame &pose, const double posError = kCartPosAccuracy,
               const double oriError = kCartOriAccuracy) const;

  //! return inverse transformation
  Frame inverse() const;

  //! Get pose vector [posX, posY, posZ, quatW, quatX, quatY, quatZ]
  Vector7d getPoseVec() const;

  //! set frame by given pose vector
  //! [posX, posY, posZ, quatW, quatX, quatY, quatZ]
  void setPoseVec(const Vector7d &vec);

  //! Get quaternion
  Quaterniond getQuat() const;

  //! Get quaternion [w;x;y;z]
  Vector4d getQuatVec() const;

  //! Set rotation matrix  by given quaternion vector [w;x;y;z]
  void setRotVec(const Vector4d &rotVec);

  //! Get Euler angle [z;y;x] [rad]
  //! The returned angles are in the ranges [0:pi]x[-pi:pi]x[-pi:pi]
  Vector3d getEulerAngleZYX() const;

  //! Set Euler angle [z;y;x] [rad]
  void setEulerAngleZYX(const double thetaZ, const double thetaY,
                        const double thetaX);

  //! get user euler angle [z;y;x] [rad]
  Vector3d getUserEulerAngleZYX() const;

  //! set user euler angle [z;y;x] [rad]
  void setUserEulerAngleZYX(const double thetaZ, const double thetaY,
                            const double thetaX);

  // ! Set rotation vector [w_x; w_y; w_z] [rad], different from EulerAngle,
  // ! a rotation vector represents a rotation axis and rotation angle.
  void setAxisAngleRotation(const Vector3d &rotV);

  //! get axis angle for the rotation matrix
  AngleAxisd getAxisAngleRotation();

  //! orthonormalize rotation matrix
  void orthonormalizeRotationMatrix();

  //! compute the velocity between two frames
  //! [m/s m/s m/s rad/s rad/s rad/s]
  Vector6d computeRelativeVelocity(const Frame &f, double t = 1.0) const;

  //! compute the Euler angle distance [z;y;z] [rad]
  //! return (targetPose - currentPose)
  Vector3d computeEularAngleDistance(const Frame &targetPose) const;

  //! compute the distance pos [m]
  double computePosDistance(const Frame &targetPos) const;

  //! reset the transformation to identity matrix
  void clear();

  //! print as string
  std::string str() const;

  //! rotation matrix
  Matrix3d m_rot;

  //! translation  vector
  Vector3d m_pos;

  //! original euler user angle, only used by UIApp [z;y;x] [rad]
  Vector3d m_userEulerAngle;

private:
  double computeQuaternionDistance(const Vector4d &v1,
                                   const Vector4d &v2) const;
};

} // namespace adi
