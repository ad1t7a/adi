#pragma once
#include "librealsense2/rs.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/surface/gp3.h>

#include <algorithm>

namespace adi {
namespace io {
namespace camera {
using pclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class Realsense {
public:
  //! constructor
  Realsense();

  //! destructor
  ~Realsense();

  // load ply
  rs2::video_frame loadPointCloud();

  // get pointcloud
  rs2::points getPoints() { return mPoints; }

  // get rgb image
  rs2::video_frame waitFramesetLoadRGB(rs2::frameset &frames);

private:
  // realsense point cloud to pcl
  pclPtr pointsToPcl(const rs2::points &points);

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud mPc;

  // We want the points object to be persistent so we can display the last cloud
  // when a frame drops
  rs2::points mPoints;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline mPipe;
};
} // namespace camera
} // namespace io
} // namespace adi