#include "multibody/rbdlmultibody.hpp"

namespace adi {
namespace multibody {
//! constructor
RBDLMultiBody::RBDLMultiBody(std::string urdfPath, bool floatingBase)
    : MultiBodyPlant(urdfPath, floatingBase) {
  m_model = new RigidBodyDynamics::Model();
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(urdfPath.c_str(), m_model,
                                                   m_urdfModel, floatingBase)) {
    return;
  }
  mJntAngle = VectorXd::Zero(getDoFs());
}

//! destructor
RBDLMultiBody::~RBDLMultiBody() { delete (m_model); }

//! update kinematics
void RBDLMultiBody::updateKinematics(const VectorXd pos, const VectorXd vel) {
  ADI_ASSERT(m_model->dof_count == pos.size());
  mJntAngle = pos;
  mJntVelocity = vel;
  RigidBodyDynamics::UpdateKinematicsCustom(*m_model, &pos, &mJntVelocity,
                                            nullptr);
}

//! get transformation matrix
void RBDLMultiBody::getTransformationMatrix(const int bodyId, Frame &T) const {
  ADI_ASSERT(m_model->IsBodyId(bodyId) == true);
  const RigidBodyDynamics::Math::SpatialTransform &st = m_model->X_base[bodyId];
  Affine3d affine = Translation(st.r) * st.E.transpose();
  T.m_pos = affine.translation();
  T.m_rot = affine.rotation();
}

//! get jacobian
void RBDLMultiBody::getJacobian(const int linkId, const Vector3d &argPos,
                                const bool global, MatrixXd &argJ) const {
  size_t dofs = getDoFs();
  // convert to local coordinate
  Frame frame;
  getTransformationMatrix(linkId, frame);
  Vector3d localPos;
  if (global) {
    localPos = frame.m_rot.transpose() * (argPos - frame.m_pos);
  } else {
    localPos = argPos;
  }
  MatrixXd tempJ = MatrixXd::Zero(adi::kCartPoseDofs, dofs);
  RigidBodyDynamics::CalcPointJacobian6D(*m_model, mJntAngle, linkId, localPos,
                                         tempJ, false);

  // copy jacobian
  argJ = MatrixXd::Zero(6, dofs);
  argJ.block(0, 0, 3, dofs) = tempJ.block(3, 0, 3, dofs);
  argJ.block(3, 0, 3, dofs) = tempJ.block(0, 0, 3, dofs);
}

//! get body name
std::string RBDLMultiBody::getBodyName(const int bodyId) {
  return m_model->GetBodyName(bodyId);
}
} // namespace multibody
} // namespace adi