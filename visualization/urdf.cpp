#include "visualization/urdf.hpp"
#include "visualization/base64.hpp"

#include <fstream>
#include <string>

namespace adi {
namespace visualization {

/***************************************************************************/ /**
* Constructor
*
* @param ipAddress IP Address of the ZMQ TCP socket
******************************************************************************/
URDF::URDF(std::string ipAddress) : Visualization(ipAddress) {
  m_pathPrefix = "";
  m_textureData = "";
  m_textureUUID = "";
  m_model = nullptr;
  m_linkNameToIndex.clear();
}

/***************************************************************************/ /**
* Destructor
******************************************************************************/
URDF::~URDF() {
  if (m_model != nullptr) {
    delete (m_model);
  }
}

/***************************************************************************/ /**
* Load URDF file
*
* @param filename   file name
* @param floatingBase floating base
* @param texturePath path to texture file
* @param robotName   robot name
* @return true if the robot loads
******************************************************************************/
bool URDF::loadURDF(std::string filename, bool floatingBase,
                    const std::string &texturePath,
                    const std::string &robotName) {
  m_model = new RigidBodyDynamics::Model();
  std::string fullPath = m_pathPrefix + filename;
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(fullPath.c_str(), m_model,
                                                   m_urdfModel, floatingBase)) {
    return false;
  }
  m_robotName = m_urdfModel->name_;
  if (robotName != "") {
    m_robotName = robotName;
  }
  deleteMultibody();
  convertVisuals(texturePath);
  return true;
}

/***************************************************************************/ /**
* Sync visual transform
*
* @param Q   pose update
******************************************************************************/
void URDF::syncVisualTransforms(RigidBodyDynamics::Math::VectorNd Q) {
  RigidBodyDynamics::Math::VectorNd QDot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->q_size);
  RigidBodyDynamics::Math::VectorNd QDDot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->q_size);
  RigidBodyDynamics::UpdateKinematics(*m_model, Q, QDot, QDDot);
  for (size_t i = 0; i < m_model->mBodies.size(); i++) {
    if (m_model->IsBodyId(i)) {
      RigidBodyDynamics::Math::SpatialTransform baseFrame = m_model->X_base[i];
      double world_pos[3] = {baseFrame.r[0], baseFrame.r[1], baseFrame.r[2]};
      double world_mat[9] = {
          baseFrame.E(0, 0), baseFrame.E(0, 1), baseFrame.E(0, 2),
          baseFrame.E(1, 0), baseFrame.E(1, 1), baseFrame.E(1, 2),
          baseFrame.E(2, 0), baseFrame.E(2, 1), baseFrame.E(2, 2)};
      std::string vis_name = kVisualizerPath + m_robotName + std::string("/") +
                             m_linkNameToIndex[i];
      nlohmann::json tr_cmd =
          createTransformCmd(world_pos, world_mat, vis_name.c_str());
      sendZMQ(tr_cmd);
    }
  }
}

/***************************************************************************/ /**
* Sync visual transform
*
* @param transforms   pose update
******************************************************************************/
void URDF::syncVisualTransforms(std::vector<RigidBodyDynamics::Math::SpatialTransform> transforms) {

  for (size_t i = 0; i < m_model->mJoints.size(); i++) {
    syncVisualTransforms(i, transforms[i]);
  }
}

/***************************************************************************/ /**
                                                                               * Sync visual transform
                                                                               *
                                                                               * @param transforms   pose update
                                                                               ******************************************************************************/
void URDF::syncVisualTransforms(
    size_t linkIndex, RigidBodyDynamics::Math::SpatialTransform transforms) {
  double world_pos[3] = {transforms.r[0], transforms.r[1], transforms.r[2]};
  double world_mat[9] = {
      transforms.E(0, 0), transforms.E(0, 1), transforms.E(0, 2),
      transforms.E(1, 0), transforms.E(1, 1), transforms.E(1, 2),
      transforms.E(2, 0), transforms.E(2, 1), transforms.E(2, 2)};
  std::string vis_name = kVisualizerPath + m_robotName + std::string("/") +
                         m_linkNameToIndex[linkIndex];
  nlohmann::json tr_cmd =
      createTransformCmd(world_pos, world_mat, vis_name.c_str());
  sendZMQ(tr_cmd);
}

/***************************************************************************/ /**
* Convert visuals and load the robot visuals
* TODO (Adi): Works only for serial link URDFs. Closed links or branches has to be handled
* @param texturePath   texture path
******************************************************************************/
void URDF::convertVisuals(const std::string &texturePath) {
  if (texturePath != "") {
    loadTexture(texturePath);
  }
  m_linkNameToIndex.clear();
  std::vector<my_shared_ptr<urdf::Link>> links;
  m_urdfModel->getLinks(links);

  for (size_t i = 0; i < links.size(); i++) {
    m_linkNameToIndex[m_model->GetBodyId(links[i]->name.c_str())] =
        links[i]->name;
    urdf::Link a = *links[i];
    convertLinkVisuals(a, i, false);
  }
}

/***************************************************************************/ /**
* Delete the multibody
******************************************************************************/
void URDF::deleteMultibody() {
  std::string robotPath = kVisualizerPath + m_robotName;
  nlohmann::json delCmd = createDeleteCmd(robotPath);
  sendZMQ(delCmd);
}

/***************************************************************************/ /**
* Load texture file
* @param texture file path
******************************************************************************/
void URDF::loadTexture(const std::string &texturePath) {
  m_textureData = kTextureDataBrokenRobot;

  FILE *fp = fopen((m_pathPrefix + texturePath).c_str(), "rb");
  if (fp) {
    fseek(fp, 0, SEEK_END);
    unsigned int datasize = static_cast<unsigned int>(ftell(fp));
    fseek(fp, 0, SEEK_SET);
    unsigned char *data = static_cast<unsigned char *>(malloc(datasize));
    if (data) {
      int bytesRead;
      bytesRead = fread(data, 1, datasize, fp);
      m_textureData =
          std::string("data:image/png;base64,") + base64_encode(data, datasize);
    }
    free(data);
    fclose(fp);
  }
}

/***************************************************************************/ /**
* Convert link visuals
* @param link link path
* @param linkIndex link index
* @param useTextureUUID texture UUID
******************************************************************************/
void URDF::convertLinkVisuals(urdf::Link &link, int linkIndex,
                              bool useTextureUUID) {
  for (int vis_index = 0; vis_index < (int)link.visual_array.size();
       vis_index++) {
    my_shared_ptr<urdf::Visual> &v = link.visual_array[vis_index];
    std::string vis_name =
        kVisualizerPath + m_robotName + std::string("/") + link.name;

    int color_rgb = 0xffffff;
    double world_pos[3] = {0, 0, 0};

    if (v->geometry->type == urdf::Geometry::MESH) {
      std::string objData;
      FILE *fp = fopen((m_pathPrefix + v->geometry->filename).c_str(), "r");
      if (fp) {
        fseek(fp, 0, SEEK_END);
        int datasize = (int)ftell(fp);
        fseek(fp, 0, SEEK_SET);
        char *data = static_cast<char *>(malloc(datasize + 1));
        if (data) {
          int bytesRead;
          bytesRead = fread(data, 1, datasize, fp);
          data[datasize] = 0;
          objData = std::string(data);
        }
        free(data);
        fclose(fp);
      }
      int str_len = objData.length();

      if (str_len) {
        std::string obj_data_utf8 = correct_non_utf_8(objData);
        if (useTextureUUID) {
          nlohmann::json cmd =
              createTexturedMeshCmd2(obj_data_utf8.c_str(), m_textureUUID,
                                     world_pos, color_rgb, vis_name.c_str());
          sendZMQ(cmd);
        } else {
          nlohmann::json cmd = createTexturedMeshCmd(
              obj_data_utf8.c_str(), m_textureData.c_str(), world_pos,
              color_rgb, vis_name.c_str());

          nlohmann::json ob = cmd["object"];
          nlohmann::json texs = ob["textures"];
          nlohmann::json tex = texs[0];
          nlohmann::json uuid = tex["uuid"];
          m_textureUUID = uuid;
          sendZMQ(cmd);
        }
      }
    }
  }
}

/***************************************************************************/ /**
                                                                               * Initialize the link surface points
                                                                               * @param link index
                                                                               ******************************************************************************/
void URDF::initializeLinkSurfacePoints() {
  std::vector<my_shared_ptr<urdf::Link>> links;
  m_urdfModel->getLinks(links);

  for (size_t i = 0; i < links.size(); i++) {
    m_linkNameToIndex[m_model->GetBodyId(links[i]->name.c_str())] =
        links[i]->name;
    urdf::Link a = *links[i];
    getSurfacePoints(a, i, 10);
  }
}

/***************************************************************************/ /**
                                                                               * Update surface points
                                                                               * @param link link path
                                                                               * @param linkIndex link index
                                                                               * @param useTextureUUID texture UUID
                                                                               ******************************************************************************/
void URDF::getSurfacePoints(urdf::Link &link, int linkIndex,
                            int numSurfacePoints) {
  // variables for point
  char c;
  double x, y, z;
  // variables to save the points
  std::vector<RigidBodyDynamics::Math::Vector3d> points;
  for (int vis_index = 0; vis_index < (int)link.visual_array.size();
       vis_index++) {
    my_shared_ptr<urdf::Visual> &v = link.visual_array[vis_index];
    if (v->geometry->type == urdf::Geometry::MESH) {
      std::ifstream input((m_pathPrefix + v->geometry->filename).c_str());
      std::string line;
      while (std::getline(input, line)) {
        if (line[0] == 'v' && line[1] == ' ') {
          RigidBodyDynamics::Math::Vector3d point;
          std::stringstream ss(line);
          ss >> c >> x >> y >> z;
          point[0] = x;
          point[1] = y;
          point[2] = z;
          points.push_back(point);
        }
      }
    }
  }
  while (points.size() > numSurfacePoints) {
    int randomIdx = rand() % points.size();
    points.erase(points.begin() + randomIdx);
  }
  m_indexSurfacePoints[linkIndex] = points;
}

void URDF::visualizeLinkSurfacePoints() {
  for (size_t index = 0; index < m_model->mBodies.size(); index++) {
    if (m_model->IsBodyId(index)) {
      double worldPos[3] = {0.0, 0.0, 0.0};
      std::vector<RigidBodyDynamics::Math::Vector3d> points =
          m_indexSurfacePoints[index];
      for (size_t i = 0; i < points.size(); i++) {
        std::string surfacePt = kVisualizerPath + m_robotName +
                                std::string("/") + "surfacePoint/" +
                                std::to_string(index) + "/" + std::to_string(i);
        worldPos[0] = points[i][0];
        worldPos[1] = points[i][1];
        worldPos[2] = points[i][2];
        sendZMQ(createSphere(0.01, worldPos, 0, surfacePt.c_str()));
      }
    }
  }
}

/***************************************************************************/ /**
                                                                               * Sync visual transform
                                                                               *
                                                                               * @param Q   pose update
                                                                               ******************************************************************************/
void URDF::syncSurfacePointsTransforms(RigidBodyDynamics::Math::VectorNd Q) {
  RigidBodyDynamics::Math::VectorNd QDot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->q_size);
  RigidBodyDynamics::Math::VectorNd QDDot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->q_size);
  RigidBodyDynamics::UpdateKinematics(*m_model, Q, QDot, QDDot);
  for (size_t index = 0; index < m_model->mBodies.size(); index++) {
    if (m_model->IsBodyId(index)) {
      RigidBodyDynamics::Math::SpatialTransform baseFrame =
          m_model->X_base[index];
      double world_mat[9] = {
          baseFrame.E(0, 0), baseFrame.E(0, 1), baseFrame.E(0, 2),
          baseFrame.E(1, 0), baseFrame.E(1, 1), baseFrame.E(1, 2),
          baseFrame.E(2, 0), baseFrame.E(2, 1), baseFrame.E(2, 2)};
      std::vector<RigidBodyDynamics::Math::Vector3d> points =
          m_indexSurfacePoints[index];
      for (size_t j = 0; j < points.size(); j++) {
        double world_pos[3] = {baseFrame.r[0], baseFrame.r[1], baseFrame.r[2]};
        std::string surfacePt = kVisualizerPath + m_robotName +
                                std::string("/") + "surfacePoint/" +
                                std::to_string(index) + "/" + std::to_string(j);
        nlohmann::json tr_cmd =
            createTransformCmd(world_pos, world_mat, surfacePt.c_str());
        sendZMQ(tr_cmd);
      }
    }
  }
}

/***************************************************************************/ /**
                                                                               * Sync visual transform
                                                                               *
                                                                               * @param Q   pose update
                                                                               ******************************************************************************/
std::map<int, std::vector<RigidBodyDynamics::Math::Vector3d>>
URDF::getSurfacePoints(RigidBodyDynamics::Math::VectorNd Q) {
  /*RigidBodyDynamics::Math::VectorNd QDot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->q_size);
  RigidBodyDynamics::Math::VectorNd QDDot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->q_size);
  RigidBodyDynamics::UpdateKinematics(*m_model, Q, QDot, QDDot);
  std::map<int, std::vector<RigidBodyDynamics::Math::Vector3d>> linksPoints;
  for(size_t index=0; index<m_model->mBodies.size(); index++) {
    std::vector<RigidBodyDynamics::Math::Vector3d> linkPoints;
    if (m_model->IsBodyId(index)) {
      RigidBodyDynamics::Math::SpatialTransform baseFrame =
  m_model->X_base[index]; std::vector<RigidBodyDynamics::Math::Vector3d> points
  = m_indexSurfacePoints[index]; for(size_t j=0; j<points.size(); j++) {
        linkPoints.push_back(baseFrame.r + (baseFrame.E * points[j]));
      }
    }
    linksPoints[index] = linkPoints;
  }*/
  return m_indexSurfacePoints;
}

} // namespace visualization
} // namespace adi