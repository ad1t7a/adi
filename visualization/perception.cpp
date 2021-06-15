#include "visualization/perception.hpp"
#include "visualization/base64.hpp"

namespace adi {
namespace visualization {

Perception::Perception(std::string ipAddress) : URDF(ipAddress) {}

Perception::~Perception() {
  // destructor
}

void Perception::loadTexture(const std::string &texturePath) {
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
* Create mesh object
*
* @param textureData   texture data
* @param worldPos world pose
* @param colorRGB RGB color
* @param path     path
* @return mesh json object
******************************************************************************/
nlohmann::json Perception::createTextureCmd(const char *textureData,
                                            double worldPos[3], int colorRGB,
                                            const char *path) {
  std::string mapUUID = generateUUID();
  std::string imageUUID = generateUUID();
  std::string geomUUID = generateUUID();
  std::string materialUUID = generateUUID();

  nlohmann::json setObjectBoxCmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {
           {"metadata", {{"type", "Object"}, {"version", 4.5}}},
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
       }}};
  return setObjectBoxCmd;
}

} // namespace visualization
} // namespace adi