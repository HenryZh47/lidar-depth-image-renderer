#ifndef LIDAR_DEPTH_RENDERER_H
#define LIDAR_DEPTH_RENDERER_H

/*
 *
 * Base class for LidarDepthRenderer
 * Provides a sequential version of lidar renderer.
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

class LidarDepthRenderer {
 public:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  LidarDepthRenderer() = default;
  ~LidarDepthRenderer() = default;

  virtual cv::Mat render(const sensor_msgs::CameraInfo &camera_info,
                         const tf2::Transform &map_to_camera_tf,
                         const int bloat_factor);
  void set_cloud(const PointCloudConstPtr &new_cloud_ptr);

 private:
  Eigen::Affine3f eigen_tf_from_tf2(const tf2::Transform &tf);
  PointCloudConstPtr cloud_ptr;
};  // class LidarDepthRenderer

#endif  // LIDAR_DEPTH_RENDERER_H
