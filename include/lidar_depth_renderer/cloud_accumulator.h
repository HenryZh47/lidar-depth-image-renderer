#ifndef LIDAR_RENDER_CLOUD_ACCUMULATOR_H
#define LIDAR_RENDER_CLOUD_ACCUMULATOR_H

/*
 * Cloud accumulator for lidar depth image renderer
 * Accumulated a sliding window of clouds in world frame
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>
#include "lidar_depth_renderer/common_utils.h"

class CloudAccumulator {
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

 public:
  CloudAccumulator()
      : window_size(0), num_clouds(0), num_points(0), updated(false) {}
  CloudAccumulator(size_t input_size)
      : window_size(input_size), num_clouds(0), num_points(0), updated(false) {}
  ~CloudAccumulator() = default;

  void add(const PointCloudPtr &new_cloud_ptr) {
    // add to sliding window deque
    // also maintain num_clouds and num_points
    cloud_window.push_back(new_cloud_ptr);
    num_clouds++;
    num_points += new_cloud_ptr->size();
    if (num_clouds > window_size) {
      num_points -= cloud_window.front()->size();
      cloud_window.pop_front();
      num_clouds--;
    }
    updated = true;
  }

  PointCloudPtr get_cloud() {
    // only re-populate the cloud when updated
    if (updated) {
      updated = false;
      // reset pointer to new cloud
      cloud_window_ptr.reset(new PointCloud);
      // combine the clouds in sliding window
      cloud_window_ptr->reserve(num_points);
      for (const auto cloud_ptr : cloud_window) {
        cloud_window_ptr->insert(cloud_window_ptr->end(), cloud_ptr->begin(),
                                 cloud_ptr->end());
      }
    }

    // ROS_INFO_STREAM_THROTTLE(
    //    1, "got " << num_clouds << " clouds with " << num_points << "
    //    points");
    return cloud_window_ptr;
  }

  void set_window_size(size_t input_size) { window_size = input_size; }

 private:
  size_t window_size;

  std::deque<PointCloudPtr> cloud_window;
  size_t num_clouds;
  size_t num_points;
  PointCloudPtr cloud_window_ptr;
  bool updated;

};  // Class CloudAccumulator

void *cuda_malloc(const size_t size);
void cuda_memcpy_to_dev(void *dst, const void *src, const size_t size);
void cuda_free(void *dst);

class CloudAccumulatorCuda {
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using CloudWindow = std::deque<std::pair<void *, size_t>>;
  using CloudWindowPtr = std::deque<std::pair<void *, size_t>> *;

 public:
  CloudAccumulatorCuda() : window_size(0), num_clouds(0), num_points(0) {}
  CloudAccumulatorCuda(size_t input_size)
      : window_size(input_size), num_clouds(0), num_points(0) {}
  ~CloudAccumulatorCuda() = default;

  void add(const PointCloudPtr &new_cloud_ptr) {
    // add to sliding window deque
    // also maintain num_clouds and num_points
    cloud_window.emplace_back(
        cuda_malloc(new_cloud_ptr->size() * sizeof(Point)),
        new_cloud_ptr->size());
    cuda_memcpy_to_dev(cloud_window.back().first, new_cloud_ptr->points.data(),
                       new_cloud_ptr->size() * sizeof(Point));
    num_clouds++;
    num_points += new_cloud_ptr->size();
    if (num_clouds > window_size) {
      num_points -= cloud_window.front().second;
      cuda_free(cloud_window.front().first);
      cloud_window.pop_front();
      num_clouds--;
    }
  }

  CloudWindowPtr get_cloud() { return &cloud_window; }

  void set_window_size(size_t input_size) { window_size = input_size; }

 private:
  size_t window_size;

  CloudWindow cloud_window;
  size_t num_clouds;
  size_t num_points;

};  // Class CloudAccumulatorCuda

#endif  // CLOUD_ACCUMULATOR_H
