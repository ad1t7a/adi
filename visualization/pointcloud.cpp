#include "visualization/pointcloud.hpp"
#include <iostream>

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
    std::string filePath =
        "/Users/ad1t7a/Developer/adi/robots/robotiq/graphics/text.txt";
    std::string objData;
    FILE *fp = fopen(filePath.c_str(), "r");
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
    std::string obj_data_utf8 = correct_non_utf_8(objData);
    std::cout << obj_data_utf8 << "\n";
    nlohmann::json setPointCloudCmd = {
        {"type", "set_object"},
        {"path", path},
        {"object",
         {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
          {"geometries",
           {{
               {"uuid", geomUID},
               {"type", "BufferGeometry"},
               {"data",
                {"attributes",
                 {"position",
                  {"itemSize", 3},
                  {"type", "Float32Array"},
                  {"normalized", false},
                  {"array", obj_data_utf8.c_str()}}}},
           }}},
          {"materials",
           {{{"vertexColors", 2},
             {"size", 0.1},
             {"color", colorRGB},
             {"type", "PointsMaterial"},
             {"uuid", materialUID}}}},
          {
              "object",
              {{"geometry", geomUID},
               {"material", materialUID},
               {"matrix",
                {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                 worldPos[0], worldPos[1], worldPos[2], 1.0}},
               {"type", "Points"},
               {"uuid", objectUID}},
          }}}};
    sendZMQ(setPointCloudCmd, true);
  }

} // namespace visualization
} // namespace adi