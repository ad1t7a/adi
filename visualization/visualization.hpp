#pragma once

#include "visualization/constants.hpp"

#include "nlohmann/json.hpp"
#include "zmq.hpp"
#include "zmq_addon.hpp"

namespace adi {
namespace visualization {
class Visualization {
public:
  //! constructor
  Visualization(std::string ipAddress);

  //! destructor
  ~Visualization();

  //! create sphere
  nlohmann::json createSphere(double radius, double worldPos[3], int colorRGB,
                              const char *path, bool transparent = false,
                              double opacity = 1.0);

  //! create box
  nlohmann::json createBoxCmd(double width, double height, double length,
                              double worldPos[3], int colorRGB,
                              const char *path);

  //! create mesh
  nlohmann::json createMeshCmd(const char *obj_data, double worldPos[3],
                               int colorRGB, const char *path);

  //! create textured mesh
  nlohmann::json createTexturedMeshCmd(const char *objData,
                                       const char *textureData,
                                       double worldPos[3], int colorRGB,
                                       const char *path);

  //! create textured mesh
  nlohmann::json createTexturedMeshCmd2(const char *objData,
                                        std::string mapUUID, double worldPos[3],
                                        int colorRGB, const char *path);

  //! frame transform
  nlohmann::json createTransformCmd(double worldPos[3], double worldMat[9],
                                    const char *path);

  //! create delete
  nlohmann::json createDeleteCmd(const std::string &path = kVisualizerPath);

  //! send ZMQ
  void sendZMQ(nlohmann::json cmd, bool verbose = false);

protected:
  //! generate UUID
  std::string generateUUID();

private:
  // context
  zmq::context_t m_context;

  //! socket
  zmq::socket_t *m_socket;
};
} // namespace visualization
} // namespace adi