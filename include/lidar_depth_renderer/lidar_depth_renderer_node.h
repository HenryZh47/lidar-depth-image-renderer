#ifndef LIDAR_DEPTH_RENDERER_NODE_H
#define LIDAR_DEPTH_RENDERER_NODE_H

/*
 * ROS node for lidar depth renderer
 */

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "lidar_depth_renderer/lidar_depth_renderer.h"

class LidarDepthRendererNode {
 public:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  LidarDepthRendererNode(const std::string node_name);
  ~LidarDepthRendererNode();

  void lidar_cb(const PointCloudConstPtr &cloud_ptr);
  void camera_info_cb(const sensor_msgs::CameraInfoConstPtr &info_ptr,
                      const image_transport::ImageTransport image_it);

 private:
  ros::NodeHandle nh;
  LidarDepthRenderer renderer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // publishers and subscribers
  image_transport::ImageTransport it;
  ros::Subscriber lidar_sub;
  ros::Subscriber camera_info_sub;

  // TODO(hengruiz): need a cloud accumulator class to combine multiple laser
  // frames

};  // class LidarDepthRendererNode

#endif
