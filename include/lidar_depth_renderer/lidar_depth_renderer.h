#ifndef LIDAR_DEPTH_RENDERER_H
#define LIDAR_DEPTH_RENDERER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/CameraInfo.h>

class LidarDepthRenderer {
 public:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  LidarDepthRenderer();
  LidarDepthRenderer(sensor_msgs::CameraInfoConstPtr &camera_info);

  void render_points();

 private:
};  // class LidarDepthRenderer

#endif  // LIDAR_DEPTH_RENDERER_H
