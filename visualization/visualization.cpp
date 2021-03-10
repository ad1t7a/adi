#include "visualization/visualization.hpp"

#include "Guid.hpp"

namespace adi {
namespace visualization {

/***************************************************************************/ /**
* Constructor
*
* @param ipAddress IP Address of the ZMQ TCP socket
******************************************************************************/
Visualization::Visualization(std::string ipAddress) {
  m_socket = new zmq::socket_t(m_context, ZMQ_REQ);
  m_socket->connect(ipAddress);
}

/***************************************************************************/ /**
* Destructor
******************************************************************************/
Visualization::~Visualization() {}

/***************************************************************************/ /**
* Create sphere object
*
* @param radius   radius
* @param worldPos world pose
* @param colorRGB RGB color
* @param path     path
* @param transparent transparency bool
* @param opacity opacity scale
* @return sphere json object
******************************************************************************/
nlohmann::json Visualization::createSphere(double radius, double worldPos[3],
                                           int colorRGB, const char *path,
                                           bool transparent, double opacity) {
  std::string geomUID = generateUUID();
  std::string materialUID = generateUUID();
  std::string objectUID = generateUUID();

  nlohmann::json setObjectSphereCmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{
             {"radius", radius},
             {"type", "SphereGeometry"},
             {"uuid", geomUID},
         }}},
        {"materials",
         {{{"color", colorRGB},
           {"reflectivity", 0.5},
           {"side", 200},
           {"transparent", transparent},
           {"opacity", opacity},
           {"type", "MeshPhongMaterial"},
           {"uuid", materialUID}}}},
        {
            "object",
            {{"geometry", geomUID},
             {"material", materialUID},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               worldPos[0], worldPos[1], worldPos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", objectUID}},
        }}}};
  return setObjectSphereCmd;
}

/***************************************************************************/ /**
* Create box object
*
* @param radius   radius
* @param worldPos world pose
* @param colorRGB RGB color
* @param path     path
* @param transparent transparency bool
* @param opacity opacity scale
* @return box json object
******************************************************************************/
nlohmann::json Visualization::createBoxCmd(double width, double height,
                                           double length, double worldPos[3],
                                           int colorRGB, const char *path) {
  std::string geomUID = generateUUID();
  std::string materialUID = generateUUID();
  std::string objectUID = generateUUID();

  nlohmann::json setObjectBoxCmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{{"depth", length},
           {"height", height},
           {"type", "BoxGeometry"},
           {"uuid", geomUID},
           {"width", width}}}},
        {"materials",
         {{{"color", colorRGB},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"type", "MeshPhongMaterial"},
           {"uuid", materialUID}}}},
        {
            "object",
            {{"geometry", geomUID},
             {"material", materialUID},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               worldPos[0], worldPos[1], worldPos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", objectUID}},
        }}}};

  return setObjectBoxCmd;
}

/***************************************************************************/ /**
* Create mesh object
*
* @param objData   object data
* @param worldPos world pose
* @param colorRGB RGB color
* @param path     path
* @return mesh json object
******************************************************************************/
nlohmann::json Visualization::createMeshCmd(const char *objData,
                                            double worldPos[3], int colorRGB,
                                            const char *path) {
  std::string geomUID = generateUUID();
  std::string materialUID = generateUUID();
  std::string objectUID = generateUUID();

  nlohmann::json setObjectBoxCmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{
             {"type", "_meshfile"},
             {"uuid", geomUID},
             {"format", "obj"},
             {"data", objData},
         }}},
        {"materials",
         {{{"color", colorRGB},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"type", "MeshPhongMaterial"},
           {"uuid", materialUID}}}},
        {
            "object",
            {{"geometry", geomUID},
             {"material", materialUID},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               worldPos[0], worldPos[1], worldPos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", objectUID}},
        }}}};

  return setObjectBoxCmd;
}

/***************************************************************************/ /**
* Create mesh object
*
* @param objData   object data
* @param textureData   texture data
* @param worldPos world pose
* @param colorRGB RGB color
* @param path     path
* @return mesh json object
******************************************************************************/
nlohmann::json Visualization::createTexturedMeshCmd(const char *objData,
                                                    const char *textureData,
                                                    double worldPos[3],
                                                    int colorRGB,
                                                    const char *path) {
  std::string mapUUID = generateUUID();
  std::string imageUUID = generateUUID();
  std::string geomUUID = generateUUID();
  std::string materialUUID = generateUUID();
  std::string objectUUID = generateUUID();

  nlohmann::json setObjectBoxCmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{

             {"type", "_meshfile"},
             {"uuid", geomUUID},
             {"format", "obj"},
             {"data", objData},
         }}},
        {"images",
         {{
             {"uuid", imageUUID},
             {"url", textureData},
         }}},
        {"textures",
         {{{"wrap", {1, 1}},
           {"uuid", mapUUID},
           {"repeat", {1, 1}},
           {"image", {imageUUID}}}}},
        {"materials",
         {{{"color", colorRGB},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"map", mapUUID},
           {"type", "MeshPhongMaterial"},
           {"uuid", materialUUID}}}},
        {
            "object",
            {{"geometry", geomUUID},
             {"material", materialUUID},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               worldPos[0], worldPos[1], worldPos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", objectUUID}},
        }}}};
  return setObjectBoxCmd;
}

/***************************************************************************/ /**
* Create mesh object
*
* @param objData   object data
* @param mapUUID   map UUID
* @param worldPos world pose
* @param colorRGB RGB color
* @param path     path
* @return mesh json object
******************************************************************************/
nlohmann::json Visualization::createTexturedMeshCmd2(const char *objData,
                                                     std::string mapUUID,
                                                     double worldPos[3],
                                                     int colorRGB,
                                                     const char *path) {
  std::string imageUUID = generateUUID();
  std::string geomUUID = generateUUID();
  std::string materialUUID = generateUUID();
  std::string objectUUID = generateUUID();

  nlohmann::json setObjectBoxCmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{

             {"type", "_meshfile"},
             {"uuid", geomUUID},
             {"format", "obj"},
             {"data", objData},
         }}},
        {"materials",
         {{{"color", colorRGB},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"map", mapUUID},
           {"type", "MeshPhongMaterial"},
           {"uuid", materialUUID}}}},
        {
            "object",
            {{"geometry", geomUUID},
             {"material", materialUUID},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               worldPos[0], worldPos[1], worldPos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", objectUUID}},
        }}}};

  return setObjectBoxCmd;
}

/***************************************************************************/ /**
* Create frame transformation object
*
* @param worldPos world pose
* @param worldMat world rotation matrix
* @param path     path
* @return frame transform json object
******************************************************************************/
nlohmann::json Visualization::createTransformCmd(double worldPos[3],
                                                 double worldMat[9],
                                                 const char *path) {
  nlohmann::json transformCmd = {
      {"type", "set_transform"},
      {"path", path},
      {"matrix",
       {worldMat[0], worldMat[1], worldMat[2], 0., worldMat[3], worldMat[4],
        worldMat[5], 0., worldMat[6], worldMat[7], worldMat[8], 0., worldPos[0],
        worldPos[1], worldPos[2], 1.0}},
  };
  return transformCmd;
}

/***************************************************************************/ /**
* Delete object in visualizer
*
* @param path path to delete
* @return delete json object
******************************************************************************/
nlohmann::json Visualization::createDeleteCmd(const std::string &path) {
  nlohmann::json deleteCmd = {{"type", "delete"}, {"path", path.c_str()}};
  return deleteCmd;
}

/***************************************************************************/ /**
* Send ZMQ command of the JSON to the predefined IP address
*
* @param cmd JSON object to send in as a command
* @param verbose boolean flag to enable verbose
******************************************************************************/
void Visualization::sendZMQ(nlohmann::json cmd, bool verbose) {
  std::vector<uint8_t> packed = nlohmann::json::to_msgpack(cmd);
  std::string typeStr = cmd["type"];
  std::string pathStr = cmd["path"];

  zmq::multipart_t multipart;
  multipart.add(zmq::message_t(typeStr.c_str(), typeStr.size()));
  multipart.add(zmq::message_t(pathStr.c_str(), pathStr.size()));
  multipart.add(zmq::message_t(&packed[0], packed.size()));
  multipart.send(*m_socket);

  zmq::message_t reply(1024);
  char buf[1024];
  bool recv_result = m_socket->recv(&reply);
  if (recv_result) {
    memcpy(buf, reply.data(), reply.size());
    buf[reply.size()] = 0;
    std::string d(buf);
    nlohmann::json result = nlohmann::json::from_msgpack(d, false);
  }
}

/***************************************************************************/ /**
* Generates a Universal Unique Identifier
*
* @return a UUID string
******************************************************************************/
std::string Visualization::generateUUID() { return xg::newGuid().str(); }

} // namespace visualization
} // namespace adi