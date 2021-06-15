#pragma once

#include "visualization/visualization.hpp"

#include <rbdl/rbdl.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include <urdfreader/urdfreader.h>

namespace adi {
namespace visualization {

class URDF : public Visualization {
public:
  //! constructor
  URDF(std::string ipAddress);

  //! destructor
  ~URDF();

  //! read URDF file
  bool loadURDF(std::string filename, bool floatingBase,
                const std::string &texturePath = "",
                const std::string &robotName = "");

  //! sync visual transformation
  void syncVisualTransforms(RigidBodyDynamics::Math::VectorNd Q);

  //! sync visual transformation
  void syncVisualTransforms(std::vector<RigidBodyDynamics::Math::SpatialTransform> transforms);

  //! sync visual transformation
  void
  syncVisualTransforms(size_t linkIndex,
                       RigidBodyDynamics::Math::SpatialTransform transforms);

  //! get number of degrees of freedom
  int getDoF() { return m_model->dof_count; }

  //! set path prefix
  void setPathPrefix(std::string pathPrefix) { m_pathPrefix = pathPrefix; }

  //! delete
  void deleteMultibody();

  //! texture data
  std::string m_textureData;

protected:
  //! convert visuals
  void convertVisuals(const std::string &texturePath = "");

  //! load texture
  void loadTexture(const std::string &texturePath);

  //! convert link visuals
  void convertLinkVisuals(urdf::Link &link, int linkIndex, bool useTextureUUID);

  /***************************************************************************/ /**/
  //! robot model
  RigidBodyDynamics::Model *m_model;

  //! robot urdf model
  RigidBodyDynamics::ModelPtr m_urdfModel;

  //! robot name
  std::string m_robotName;

  //! path prefix
  std::string m_pathPrefix;

  //! texture uuid
  std::string m_textureUUID;

  //! link name index map
  std::map<int, std::string> m_linkNameToIndex;
};
} // namespace visualization
} // namespace adi