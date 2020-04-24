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
 public:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  CloudAccumulator()
      : window_size(0), num_clouds(0), num_points(0), updated(false) {}
  CloudAccumulator(size_t input_size)
      : window_size(input_size), num_clouds(0), num_points(0), updated(false) {}
  ~CloudAccumulator() = default;

  void add(const PointCloudConstPtr &new_cloud_ptr,
           const tf2::Transform &to_map_tf) {
    // first transform clouds from velo_link to world
    PointCloudPtr new_map_cloud_ptr =
        transform_cloud_frame(new_cloud_ptr, to_map_tf);
    // add to sliding window deque
    // also maintain num_clouds and num_points
    cloud_window.push_back(new_map_cloud_ptr);
    num_clouds++;
    num_points += new_map_cloud_ptr->size();
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

#endif  // CLOUD_ACCUMULATOR_H
