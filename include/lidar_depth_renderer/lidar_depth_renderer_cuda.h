#ifndef LIDAR_DEPTH_RENDERER_CUDA_H
#define LIDAR_DEPTH_RENDERER_CUDA_H

/*
 * LidarDepthRenderer CUDA Parallel Version
 */

#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <deque>
#include <opencv2/core.hpp>
#include <utility>

class LidarDepthRendererCuda {
  using CloudWindow = std::deque<std::pair<void *, size_t>>;
  using CloudWindowPtr = std::deque<std::pair<void *, size_t>> *;

 public:
  LidarDepthRendererCuda() = default;
  ~LidarDepthRendererCuda() = default;

  void render(cv::Mat &result, const sensor_msgs::CameraInfo,
              const tf2::Transform &to_camera_tf, const int bloat_factor);
  void set_cloud(const CloudWindowPtr new_cloud_ptr);

  int query_implementation();
  void init(int height, int width);

 private:
  void print_cuda_info();
  // initialize image buffers
  void init_image_buf();
  // reset image scratch buffer for each frame to 255
  void set_image_buf();

  // cloud_ptr is a pointer to multiple frames of point clouds in cuda memory
  // void* is the pointer to points in that frame
  // size_t is the size of the point cloud
  const std::deque<std::pair<void *, size_t>> *cloud_ptr;

  int image_height;
  int image_width;

  // each thread writes to image_scratch, with 0 at MAX_UINT32_T
  // has to be 32-bit because of atomicMin
  uint32_t *image_scratch_buf = nullptr;
  // image_buf changes 255 to 0
  uint8_t *image_buf = nullptr;

};  // class LidarDepthRendererCuda

#endif  // LIDAR_DEPTH_RENDERER_CUDA_H
