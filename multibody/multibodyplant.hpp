#pragma once

#include "common/eigen_types.hpp"
#include "common/frame.hpp"
#include "spdlog/spdlog.h"

namespace adi {
namespace multibody {
class MultiBodyPlant {
public:
  //! constructor
  MultiBodyPlant(std::string urdfPath, bool floatingBase);

  //! destructor
  ~MultiBodyPlant();

  //! update kinematics
  virtual void updateKinematics(const VectorXd pos) {
    spdlog::error("NotImplementedException: updateKinematics");
  }

  //! get transformation matrix
  virtual void getTransformationMatrix(const int linkId, Frame &T) const {
    spdlog::error("NotImplementedException: getTransformationMatrix");
  }

  //! get jacobian
  virtual void getJacobian(const int linkId, const Vector3d &argPos,
                           const bool global, MatrixXd &argJ) const {
    spdlog::error("NotImplementedException: getJacobian");
  }

  //! set (switch) tcp index
  void setTcpIndex(const int tcpIndex) { m_currentTCPIndex = tcpIndex; }

  //! get current tcp index
  int getTcpIndex() const { return m_currentTCPIndex; }

protected:
  // urdf path
  std::string m_urdfPath;

  // floating base
  bool m_floatingBase;

  //! current TCP index
  int m_currentTCPIndex;

  //! current jnt angle
  VectorXd mJntAngle;
};
} // namespace multibody
} // namespace adi