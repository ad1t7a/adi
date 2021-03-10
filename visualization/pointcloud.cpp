#include "visualization/pointcloud.hpp"
#include <msgpack.hpp>

namespace adi {
namespace visualization {
  //! constructor
  PointCloud::PointCloud(std::string ipAddress) : Visualization(ipAddress) {}

  //! destructor
  PointCloud::~PointCloud() {}

  //! load point cloud
  void PointCloud::loadPointCloud(const char *path) {
    double worldPos[3] = {0, 0, 0};
    int colorRGB = 0xFFFFFF;
    std::string geomUID = generateUUID();
    std::string materialUID = generateUUID();
    std::string objectUID = generateUUID();
    std::vector<std::vector<std::vector<double>>> arr = {{{0.5207591 , 0.07896397, 0.89273536}, {0.88037544, 0.75724393, 0.33980307}, {0.36066929, 0.9843244 , 0.67521834}}};
    std::stringstream ss;
    msgpack::pack(ss, arr);

    nlohmann::json setPointCloudCmd = {
        {"type", "set_object"},
        {"path", path},
        {"object",
        {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
            {"geometries",
                {
                    {
                        {"uuid", geomUID},
                        {"type", "BufferGeometry"},
                        {"data", 
                            {"attributes",  
                                {"position", 
                                    {"itemSize", 3},
                                    {"type", "Float32Array"},
                                    {"normalized", false},
                                    {"array", ss.str().c_str()}
                                }
                            }
                        },
                    }
                }
            },
            {"materials",
                {
                    {
                        {"vertexColors", 2},
                        {"size", 0.1},
                        {"color", colorRGB},
                        {"type", "PointsMaterial"},
                        {"uuid", materialUID}
                    }
                }
            },
            {"object",
                {
                    {"geometry", geomUID},
                    {"material", materialUID},
                    {"matrix", {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, worldPos[0], worldPos[1], worldPos[2], 1.0}},
                    {"type", "Points"},
                    {"uuid", objectUID}
                },
            }
        }}
    };
    sendZMQ(setPointCloudCmd, true);
  }

} // namespace visualization
} // namespace adi