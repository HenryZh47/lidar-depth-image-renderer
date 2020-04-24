#ifndef LIDAR_RENDER_COMMON_UTILS_H
#define LIDAR_RENDER_COMMON_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

Eigen::Affine3f eigen_tf_from_tf2(const tf2::Transform &tf) {
  const auto &origin = tf.getOrigin();
  const auto &rotation = tf.getRotation();

  return Eigen::Affine3f(Eigen::Translation3f(origin[0], origin[1], origin[2]) *
                         Eigen::Quaternionf(rotation.w(), rotation.x(),
                                            rotation.y(), rotation.z()));
}

PointCloudPtr transform_cloud_frame(const PointCloudConstPtr &cloud_ptr,
                                    const tf2::Transform &tf) {
  PointCloudPtr transformed_cloud_ptr(new PointCloud);
  const auto tf_eigen = eigen_tf_from_tf2(tf);
  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, tf_eigen);
  return transformed_cloud_ptr;
}

#endif
