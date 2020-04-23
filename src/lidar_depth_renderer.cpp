#include "lidar_depth_renderer/lidar_depth_renderer.h"
#include <bits/stdint-uintn.h>

void LidarDepthRenderer::set_cloud(const PointCloudConstPtr new_cloud_ptr) {
  cloud_ptr = new_cloud_ptr;
}

cv::Mat LidarDepthRenderer::render(
    const sensor_msgs::CameraInfo &camera_info,
    const geometry_msgs::Transform &map_to_camera_tf, const int bloat_factor) {
  // transform points in current point clout
  PointCloudPtr camera_cloud(new PointCloud);
  pcl_ros::transformPointCloud(*cloud_ptr, *camera_cloud, map_to_camera_tf);

  // get camera_info
  const auto width = camera_info.width;
  const auto height = camera_info.height;
  const auto fx = camera_info.K[0];
  const auto fy = camera_info.K[4];
  const auto cx = camera_info.K[2];
  const auto cy = camera_info.K[5];

  // create empty result image
  cv::Mat result = cv::Mat::zeros(width, height, CV_16UC1);

  // render points in camera_cloud
  for (const auto &point : *camera_cloud) {
    // discard points that are behind the camera
    if (point.z <= 0.0) continue;

    // compute projected image coordinates
    const auto u = static_cast<int>((fx * point.x) / point.z + cx);
    const auto v = static_cast<int>((fy * point.y) / point.z + cy);

    // bloat points in the image using bloat_factor
    if (u - bloat_factor >= 0 && u + bloat_factor <= width &&
        v - bloat_factor >= 0 && v + bloat_factor <= height) {
      const auto cur_depth = static_cast<uint64_t>(point.z);
      for (int i = -bloat_factor; i <= bloat_factor; i++) {
        for (int j = -bloat_factor; j <= bloat_factor; j++) {
          // get the previous depth
          const auto prev_depth = result.at<uint64_t>(v + i, u + j);
          // update if no previous depth or closer than previous depth
          if (prev_depth == 0.0 || cur_depth < prev_depth) {
            result.at<uint64_t>(v + i, u + j) = cur_depth;
          }
        }
      }
    }
  }

  return result;
}
