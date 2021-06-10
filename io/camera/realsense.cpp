#include "io/camera/realsense.hpp"
namespace adi {
namespace io {
namespace camera {
/***************************************************************************/ /**
* Constructor
*
* @param ipAddress IP Address of the ZMQ TCP socket
******************************************************************************/
Realsense::Realsense() { mPipe.start(); }

/***************************************************************************/ /**
* Destructor
******************************************************************************/
Realsense::~Realsense() {}

/***************************************************************************/ /**
* Load point cloud file
*
* @param filename   file name
******************************************************************************/
rs2::video_frame Realsense::loadPointCloud() {
  // Wait for the next set of frames from the camera
  auto frames = mPipe.wait_for_frames();

  auto color = frames.get_color_frame();

  // For cameras that don't have RGB sensor, we'll map the pointcloud to
  // infrared instead of color
  if (!color)
    color = frames.get_infrared_frame();

  // Tell pointcloud object to map to this color frame
  mPc.map_to(color);

  auto depth = frames.get_depth_frame();

  mPoints = mPc.calculate(depth);

  // Generate the pointcloud and texture mappings
  return color;
}

/***************************************************************************/ /**
* Point cloud library
*
* @param filename   file name
******************************************************************************/
pcl_ptr Realsense::pointsToPcl(const rs2::points &points) {
  pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  auto sp = points.get_profile().as<rs2::video_stream_profile>();
  cloud->width = sp.width();
  cloud->height = sp.height();
  cloud->is_dense = false;
  cloud->points.resize(points.size());
  auto ptr = points.get_vertices();
  for (auto &p : cloud->points) {
    p.x = ptr->x;
    p.y = ptr->y;
    p.z = ptr->z;
    ptr++;
  }
  return cloud;
}

} // namespace camera
} // namespace io
} // namespace adi