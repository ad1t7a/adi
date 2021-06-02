#pragma once
#include "common/adi_assert.hpp"
#include "multibody/multibodyplant.hpp"

#include <rbdl/rbdl.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include <urdfreader/urdfreader.h>
namespace adi {
namespace multibody {
class RBDLMultiBody : public MultiBodyPlant {
public:
  //! constructor
  RBDLMultiBody(std::string urdfPath, bool floatingBase);

  //! destructor
  ~RBDLMultiBody();

  //! update kinematics
  virtual void updateKinematics(const VectorXd pos) override;

  //! get transformation matrix
  virtual void getTransformationMatrix(const int linkId,
                                       Frame &T) const override;

  //! get jacobian
  virtual void getJacobian(const int linkId, const Vector3d &argPos,
                           const bool global, MatrixXd &argJ) const override;

  //! get body name
  std::string getBodyName(const int bodyId);

  //! get number of degrees of freedom
  const size_t getDoFs() const { return m_model->dof_count; }

  //! get rigid body dynamics model
  RigidBodyDynamics::ModelPtr getURDFModel() { return m_urdfModel; }

private:
  //! robot model
  RigidBodyDynamics::Model *m_model;

  //! robot urdf model
  RigidBodyDynamics::ModelPtr m_urdfModel;
};
} // namespace multibody
} // namespace adi