#include "lidar_depth_renderer/lidar_depth_renderer.h"
#include "instrument.h"

#include <stdio.h>
#include <bits/stdint-uintn.h>
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

  // render points in camera_cloud

  int processed_pts = 0;
  size_t total_pts = (*cloud_ptr).size();
#if OMP
  #pragma omp parallel firstprivate (processed_pts)
  {
  int tid = omp_get_thread_num();
  // processed_pts = 0;

  #pragma omp for
#else
  int tid = 0;
#endif
  for (size_t ii = 0; ii < total_pts; ii++)
  {
    const auto &point = (*cloud_ptr).at(ii);
    processed_pts ++;

  cv::Mat *local_result;
#if OMP
  local_result = &(omp_mats.at(tid));
#else
  local_result = &(result);
#endif

    // transform point to camera frame
    START_ACTIVITY(ACTIVITY_TRANSFORM);
    const auto cam_point = transform_point(point, to_camera_tf);
    FINISH_ACTIVITY(ACTIVITY_TRANSFORM);

    // discard points that are behind the camera
    if (cam_point.z <= 0.0) continue;

    // compute projected image coordinates
    START_ACTIVITY(ACTIVITY_RENDER);
    const auto u = static_cast<int>((fx * cam_point.x) / cam_point.z + cx);
    const auto v = static_cast<int>((fy * cam_point.y) / cam_point.z + cy);
    FINISH_ACTIVITY(ACTIVITY_RENDER);

    START_ACTIVITY(ACTIVITY_PROJECT);
    // bloat points in the image using bloat_factor
    if (u - bloat_factor >= 0 && u + bloat_factor < width &&
        v - bloat_factor >= 0 && v + bloat_factor < height) {
      const auto cur_depth = static_cast<uint16_t>(cam_point.z * 1000.0);
      for (int i = -bloat_factor; i <= bloat_factor; i++) {
        for (int j = -bloat_factor; j <= bloat_factor; j++) {
          // get the previous depth
          const auto prev_depth = local_result->at<uint16_t>(v + i, u + j);
          // update if no previous depth or closer than previous depth
          if (prev_depth == 0.0 || cur_depth < prev_depth) {
            local_result->at<uint16_t>(v + i, u + j) = cur_depth;
          }
        }
      }
    }
    FINISH_ACTIVITY(ACTIVITY_PROJECT);
  }

#if OMP
  fprintf(stderr, "Thread %d processed %d\\%lupoints\n",
    tid, processed_pts, (*cloud_ptr).size());
  }
#endif

  // Reduce serially
// #if OMP
//   result = omp_mats.at(0);
//   for (cv::Mat &local_mat : omp_mats)
//   {
//     result = cv::min(result, local_mat);
//   }
// #endif

}

int LidarDepthRenderer::query_omp(void)
{
  int result = 0;
#if OMP
  result = 1;
#endif
  return result;
}

void LidarDepthRenderer::init(int height, int width)
{
  if (height < 0 || width < 0) return;
  int nprocessors = omp_get_max_threads();
  fprintf(stderr, "nprocessors = %d\n", nprocessors);
  omp_set_num_threads(nprocessors);
  n_threads = nprocessors;
  image_height = height;
  image_width = width;

#if OMP
#pragma omp parallel
{
  fprintf(stderr, "omp nthread=%d\n", omp_get_num_threads());
}
  fprintf(stderr, "init mat, nthread=%d\n", n_threads);
  for (int i = 0; i < n_threads; i++)
  {
    cv::Mat new_mat = cv::Mat::zeros(image_height, image_width, CV_16UC1);
    omp_mats.push_back(new_mat);
  }
#endif
}

double clockToMilliseconds(clock_t ticks){
  // units/(units/time) => time (seconds) * 1000 = milliseconds
  return (ticks/(double)CLOCKS_PER_SEC)*1000.0;
}
