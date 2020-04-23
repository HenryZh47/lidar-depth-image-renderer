#include "lidar_depth_renderer/lidar_depth_renderer_node.h"

LidarDepthRendererNode::LidarDepthRendererNode(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle)
    : nh(*node_handle),
      pnh(*private_node_handle),
      it(*node_handle),
      tf_listener(tf_buffer) {
  get_topic_names();
  initialize_sub();
  initialize_pub();
}

void LidarDepthRendererNode::get_topic_names() {
  // read in ros params
  nh.getParam("lidar_topic", lidar_topic);
  nh.getParam("camera_info_topic", camera_info_topic);
  nh.getParam("pub_topic", pub_topic);

  ROS_INFO_STREAM("lidar points topic: " << lidar_topic);
  ROS_INFO_STREAM("camera_info topic: " << camera_info_topic);
  ROS_INFO_STREAM("depth image publish topic: " << pub_topic);
}

void LidarDepthRendererNode::initialize_sub() {
  // velodyne subscriber
  // camera_info subscriber
}

void LidarDepthRendererNode::initialize_pub() {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_renderer_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  ROS_INFO("Instantiating LidarDepthRendererNode");
  LidarDepthRendererNode renderer_node(&node_handle, &private_node_handle);

  ros::spin();
  return 0;
}
