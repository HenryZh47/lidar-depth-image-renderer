#ifndef LIDAR_DEPTH_RENDERER_NODE_H
#define LIDAR_DEPTH_RENDERER_NODE_H

/*
 * ROS node for lidar depth renderer
 */

#include <cv_bridge/cv_bridge.h>
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

  // void lidar_cb(const PointCloudConstPtr &cloud_ptr);
  void lidar_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr);
  void camera_info_cb(const sensor_msgs::CameraInfoConstPtr &info_ptr,
                      const image_transport::ImageTransport image_it);

 private:
  void initialize_sub();
  void initialize_pub();
  void setup_ros();

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  LidarDepthRenderer renderer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // node info
  std::string lidar_topic;
  std::string camera_info_topic;
  std::string pub_topic;
  int cloud_size;
  int bloat_factor;

  const std::string CLOUD_FRAME = "velo_link";
  bool have_cloud = false;

  // publishers and subscribers
  image_transport::ImageTransport it;
  image_transport::Publisher lidar_depth_pub;
  ros::Subscriber lidar_sub;
  ros::Subscriber camera_info_sub;

  // TODO(hengruiz): need a cloud accumulator class to combine multiple laser
  // frames

};  // class LidarDepthRendererNode

#endif
