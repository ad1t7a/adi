#pragma once

#include "visualization/visualization.hpp"

namespace adi {
namespace visualization {

class PointCloud : public Visualization {
public:
  //! constructor
  PointCloud(std::string ipAddress);

  //! destructor
  ~PointCloud();

  //! load point cloud
  void loadPointCloud(const char *path);
};
} // namespace visualization
} // namespace adi