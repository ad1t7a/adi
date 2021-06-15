#pragma once

#include "visualization/urdf.hpp"

namespace adi {
namespace visualization {

class Perception : public URDF {
public:
  //! constructor
  Perception(std::string ipAddress);

  //! destructor
  ~Perception();

  nlohmann::json createTextureCmd(const char *textureData, double worldPos[3],
                                  int colorRGB, const char *path);

  //! load texture
  void loadTexture(const std::string &texturePath);
};
} // namespace visualization
} // namespace adi