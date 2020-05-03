#include "lidar_depth_renderer/lidar_depth_renderer.h"
#include "instrument.h"

#include <bits/stdint-uintn.h>
#include <stdio.h>
#include "lidar_depth_renderer/common_utils.h"

#if OMP
#include <omp.h>
#else
#include "fake_omp.h"
#endif

void LidarDepthRenderer::set_cloud(const PointCloudConstPtr &new_cloud_ptr) {
  cloud_ptr = new_cloud_ptr;
}

void LidarDepthRenderer::render(cv::Mat &result,
                                const sensor_msgs::CameraInfo &camera_info,
                                const tf2::Transform &to_camera_tf,
                                const int bloat_factor) {
  // get camera_info
  // the kitti conversion script put width and height inverted
  const auto width = camera_info.width;
  const auto height = camera_info.height;
  const auto fx = camera_info.K[0];
  const auto fy = camera_info.K[4];
  const auto cx = camera_info.K[2];
  const auto cy = camera_info.K[5];

  // erase result image
  result.setTo(0);
#if OMP
  // erase local image
  for (int ii = 0; ii < omp_mats.size(); ii++) {
    omp_mats.at(ii).setTo(0);
  }
#endif

  // render points in camera_cloud
  START_ACTIVITY(ACTIVITY_RENDER);
  int processed_pts = 0;
  size_t total_pts = (*cloud_ptr).size();
#if OMP
#pragma omp parallel firstprivate(processed_pts)
  {
    int tid = omp_get_thread_num();

#pragma omp for
#else
  int tid = 0;
#endif
    for (size_t ii = 0; ii < total_pts; ii++) {
      const auto &point = (*cloud_ptr).at(ii);
      processed_pts++;

      cv::Mat *local_result;
#if OMP
      local_result = &(omp_mats.at(tid));
#else
    local_result = &(result);
#endif

      // transform point to camera frame
      const auto cam_point = transform_point(point, to_camera_tf);

      // discard points that are behind the camera
      if (cam_point.z <= 0.0) continue;

      // compute projected image coordinates
      const auto u = static_cast<int>((fx * cam_point.x) / cam_point.z + cx);
      const auto v = static_cast<int>((fy * cam_point.y) / cam_point.z + cy);

      // bloat points in the image using bloat_factor
      if (u - bloat_factor >= 0 && u + bloat_factor < width &&
          v - bloat_factor >= 0 && v + bloat_factor < height) {
        // const auto cur_depth = static_cast<uint8_t>(cam_point.z * 1000.0);
        const auto cur_depth = static_cast<uint8_t>(cam_point.z * 2.0);
        for (int i = -bloat_factor; i <= bloat_factor; i++) {
          for (int j = -bloat_factor; j <= bloat_factor; j++) {
            // get the previous depth
            const auto prev_depth = local_result->at<uint8_t>(v + i, u + j);
            // update if no previous depth or closer than previous depth
            // Cap values to 254 so that 255 can represent 0 for easy reduction
            if ((prev_depth == 0 || cur_depth < prev_depth) &&
                cur_depth < 255) {
              local_result->at<uint8_t>(v + i, u + j) = cur_depth;
            }
          }
        }
      }
    }

#if OMP
  }
#endif

  FINISH_ACTIVITY(ACTIVITY_RENDER);

  START_ACTIVITY(ACTIVITY_REDUCE);
  // Reduce serially
#if OMP
  cv::Mat f2_zero_mask;
  cv::Mat and_mask;
  cv::Mat temp;

  // First frame
  result = omp_mats.at(0);
  f2_zero_mask = result == 0;
  result.setTo(255, f2_zero_mask);
  for (cv::Mat &local_mat : omp_mats) {
    f2_zero_mask = local_mat == 0;
    // Set zeros to MAX to simplify min()
    local_mat.setTo(255, f2_zero_mask);
    cv::min(local_mat, result, result);
  }
  // Set MAX back to zeros for correctness
  f2_zero_mask = result == 255;
  result.setTo(0, f2_zero_mask);
#endif
  FINISH_ACTIVITY(ACTIVITY_REDUCE);
}

int LidarDepthRenderer::query_implementation(void) {
  int result = 0;
#if OMP
  result = 1;
#endif
  return result;
}

void LidarDepthRenderer::init(int height, int width) {
  if (height < 0 || width < 0) return;
  int nprocessors = omp_get_max_threads();
  fprintf(stderr, "nprocessors = %d\n", nprocessors);
  omp_set_num_threads(nprocessors);
  n_threads = nprocessors;
  image_height = height;
  image_width = width;

#if OMP
  // #pragma omp parallel
  //  { fprintf(stderr, "omp nthread=%d\n", omp_get_num_threads()); }
  //  fprintf(stderr, "init mat, nthread=%d\n", n_threads);
  for (int i = 0; i < n_threads; i++) {
    cv::Mat new_mat = cv::Mat::zeros(image_height, image_width, CV_8UC1);
    omp_mats.push_back(new_mat);
  }
#endif
}

double clockToMilliseconds(clock_t ticks) {
  // units/(units/time) => time (seconds) * 1000 = milliseconds
  return (ticks / (double)CLOCKS_PER_SEC) * 1000.0;
}
