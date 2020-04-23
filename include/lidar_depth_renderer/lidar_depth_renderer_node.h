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

  LidarDepthRendererNode(ros::NodeHandle *node_handle,
                         ros::NodeHandle *private_node_handle);
  ~LidarDepthRendererNode() = default;

  void lidar_cb(const PointCloudConstPtr &cloud_ptr);
  void camera_info_cb(const sensor_msgs::CameraInfoConstPtr &info_ptr,
                      const image_transport::ImageTransport image_it);

 private:
  void initialize_sub();
  void initialize_pub();
  void get_topic_names();

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  LidarDepthRenderer renderer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // node info
  std::string lidar_topic;
  std::string camera_info_topic;
  std::string pub_topic;

  // publishers and subscribers
  image_transport::ImageTransport it;
  ros::Subscriber lidar_sub;
  ros::Subscriber camera_info_sub;

  // TODO(hengruiz): need a cloud accumulator class to combine multiple laser
  // frames

};  // class LidarDepthRendererNode

#endif
