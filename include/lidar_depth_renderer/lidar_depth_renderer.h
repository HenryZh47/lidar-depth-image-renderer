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

// #define OMP (1)

class LidarDepthRenderer {
 public:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  LidarDepthRenderer() = default;
  ~LidarDepthRenderer() = default;

  virtual void render(cv::Mat &result,
                      const sensor_msgs::CameraInfo &camera_info,
                      const tf2::Transform &to_camera_tf,
                      const int bloat_factor);
  void set_cloud(const PointCloudConstPtr &new_cloud_ptr);

  /**
   * @brief Queries whether OpenMP implementation is chosen
   *
   * @return int 0 if serial; 1 if OpenMP
   */
  int query_omp(void);

  /**
   * @brief Initializes renderer parameters
   *
   * @param height Height of output image
   * @param width Width of output image
   */
  void init(int height, int width);

 private:
  PointCloudConstPtr cloud_ptr;

  int n_threads;
  int image_height;
  int image_width;

#if OMP
  std::vector<cv::Mat> omp_mats;
#endif
};  // class LidarDepthRenderer

#endif  // LIDAR_DEPTH_RENDERER_H
