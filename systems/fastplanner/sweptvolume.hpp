#pragma once
#include "common/adi_assert.hpp"
#include "common/copyable.hpp"
#include "common/eigen_types.hpp"

#include <rbdl/rbdl.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include <urdfreader/urdfreader.h>

namespace adi {
namespace systems {
class SweptVolume {
public:
  ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SweptVolume);

  //! constructor
  SweptVolume(std::string prefix, std::string filename, bool floatingBase);

  //! destructor
  ~SweptVolume();

  //! calculate swept volume
  std::vector<Vector3d>
  calculateSweptVolume(RigidBodyDynamics::Math::VectorNd qStart,
                       RigidBodyDynamics::Math::VectorNd qGoal,
                       size_t numSteps = 10);

  //! get number of links
  int getDoF() { return m_model->dof_count; }

private:
  void getObjectVertices(std::string &objData);

  //! get visuals
  void getMeshDataPoints();

  //! link points
  void getLinkPoints(urdf::Link &link, int linkIndex);

  //! robot model
  RigidBodyDynamics::Model *m_model;

  //! robot urdf model
  RigidBodyDynamics::ModelPtr m_urdfModel;

  //! path prefix
  std::string m_pathPrefix;

  // point vertices for each all links
  std::vector<std::vector<Vector3d>> m_linksVertices;
};
} // namespace systems
} // namespace adi