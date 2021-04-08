#include "systems/fastplanner/sweptvolume.hpp"
#include "common/frame.hpp"

namespace adi {
namespace systems {
/***************************************************************************/ /**
* Constructor
* @param filename file name
* @param floatingBase floating base flag
******************************************************************************/
SweptVolume::SweptVolume(std::string prefix, std::string filename,
                         bool floatingBase) {
  m_pathPrefix = prefix;
  m_model = new RigidBodyDynamics::Model();
  ADI_ASSERT(RigidBodyDynamics::Addons::URDFReadFromFile(
                 (m_pathPrefix + filename).c_str(), m_model, m_urdfModel,
                 floatingBase) == true);
  getMeshDataPoints();
}

/***************************************************************************/ /**
* Destructor
******************************************************************************/
SweptVolume::~SweptVolume() { delete (m_model); }

/***************************************************************************/ /**
* Calculate swept volume
* @param qStart start position
* @param qGoal goal position
* @return swept points
******************************************************************************/
std::vector<Vector3d>
SweptVolume::calculateSweptVolume(RigidBodyDynamics::Math::VectorNd qStart,
                                  RigidBodyDynamics::Math::VectorNd qGoal,
                                  size_t numSteps) {
  // swept points
  std::vector<Vector3d> sweptPoints;
  RigidBodyDynamics::Math::VectorNd QDotAndQddot =
      RigidBodyDynamics::Math::VectorNd::Zero(m_model->dof_count);

  RigidBodyDynamics::Math::VectorNd step =
      (qGoal - qStart) / static_cast<double>(numSteps);
  for (size_t i = 0; i <= numSteps; i++) {
    RigidBodyDynamics::Math::VectorNd qStep =
        qStart + static_cast<double>(i) * step;
    RigidBodyDynamics::UpdateKinematics(*m_model, qStep, QDotAndQddot,
                                        QDotAndQddot);

    // get the transformed points
    for (size_t j = 0; j < m_model->mJoints.size(); j++) {
      Frame frame(m_model->X_base[j].r, m_model->X_base[j].E);
      for (size_t k = 0; k < m_linksVertices[j].size(); k++) {
        sweptPoints.push_back(frame * m_linksVertices[j][k]);
      }
    }
  }
  return sweptPoints;
}

/***************************************************************************/ /**
* Load mesh points and save them into m_linksVertices
******************************************************************************/
void SweptVolume::getMeshDataPoints() {
  std::vector<my_shared_ptr<urdf::Link>> links;
  m_urdfModel->getLinks(links);
  for (size_t i = 0; i < links.size(); i++) {
    urdf::Link a = *links[i];
    getLinkPoints(a, i);
  }
}

/***************************************************************************/ /**
* parse link visual points and covert to Vectors
* @param link link path
* @param linkIndex link index
******************************************************************************/
void SweptVolume::getLinkPoints(urdf::Link &link, int linkIndex) {
  for (int vis_index = 0; vis_index < (int)link.visual_array.size();
       vis_index++) {
    my_shared_ptr<urdf::Visual> &v = link.visual_array[vis_index];

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
      getObjectVertices(objData);
    }
  }
}

/***************************************************************************/ /**
* Get object vertices and load them to m_linksVertices
* @param objData parsed object data
******************************************************************************/
void SweptVolume::getObjectVertices(std::string &objData) {
  std::string findString = "v ";
  long int objectFileLength;
  std::vector<Vector3d> linkVertex;
  do {
    objectFileLength = objData.length();
    long int m = objData.find(findString);
    long int n = objData.find_first_of('\n', m + findString.length());
    if (m < 0) {
      // break after all points are read
      break;
    }
    long int k = m + findString.length();
    std::string vertex = objData.substr(k, n - k + 1);
    std::stringstream vertexStream(vertex);
    double xPt, yPt, zPt;
    vertexStream >> xPt;
    vertexStream >> yPt;
    vertexStream >> zPt;
    Vector3d point = Vector3d::Zero();
    point[0] = xPt;
    point[1] = yPt;
    point[2] = zPt;
    linkVertex.push_back(point);
    objData.erase(m, n - m + 1);
  } while (objectFileLength != objData.length());
  m_linksVertices.push_back(linkVertex);
}
} // namespace systems
} // namespace adi