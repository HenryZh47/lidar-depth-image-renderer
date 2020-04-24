#include "lidar_depth_renderer/lidar_depth_renderer_node.h"
#include "instrument.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>

LidarDepthRendererNode::LidarDepthRendererNode(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle)
    : nh(*node_handle),
      pnh(*private_node_handle),
      it(*node_handle),
      tf_listener(tf_buffer) {
  setup_ros();
  initialize_sub();
  initialize_pub();
}

void LidarDepthRendererNode::setup_ros() {
  // read in ros params
  pnh.getParam("lidar_topic", lidar_topic);
  pnh.getParam("camera_info_topic", camera_info_topic);
  pnh.getParam("pub_topic", pub_topic);

  // load cloud size parameters
  // TODO(hengruiz): ignored for now, need to implement accumulator
  pnh.getParam("cloud_size", cloud_size);
  cloud_accumulator.set_window_size(cloud_size);
  pnh.getParam("bloat_factor", bloat_factor);

  ROS_INFO_STREAM("lidar points topic: " << lidar_topic);
  ROS_INFO_STREAM("camera_info topic: " << camera_info_topic);
  ROS_INFO_STREAM("depth image publish topic: " << pub_topic);
  ROS_INFO_STREAM("accumulate cloud size: " << cloud_size);
  ROS_INFO_STREAM("bloat factor: " << bloat_factor);
}

void LidarDepthRendererNode::lidar_cb(
    const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
  // convert to pcl format
  // use PCLPointCloud2 here since the conversion only works this way..
  pcl::PCLPointCloud2 pcl2_cloud;
  pcl_conversions::toPCL(*cloud_ptr, pcl2_cloud);
  PointCloudPtr pcl_cloud_ptr(new PointCloud);
  pcl::fromPCLPointCloud2(pcl2_cloud, *pcl_cloud_ptr);

  // add to cloud accumulator
  const auto query_time = cloud_ptr->header.stamp;
  // get the transform from current velo_link to world frame
  tf2::Transform to_map_tf;
  try {
    // block for up to 100ms to wait for sensor pose tfs
    const auto map_to_camera = tf_buffer.lookupTransform(
        CLOUD_FRAME, cloud_ptr->header.frame_id, query_time, ros::Duration(.1));
    tf2::fromMsg(map_to_camera.transform, to_map_tf);
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return;
  }
  // add to sliding window
  cloud_accumulator.add(pcl_cloud_ptr, to_map_tf);

  // TODO(hengruiz): need to update accumulated cloud
  //                 just update to current laser frame for now
  renderer.set_cloud(cloud_accumulator.get_cloud());
  have_cloud = true;
}

void LidarDepthRendererNode::camera_info_cb(
    const sensor_msgs::CameraInfoConstPtr &info_ptr,
    const image_transport::ImageTransport image_it) {
  if (!have_cloud) {
    ROS_WARN("Node hasn't received any pointcloud yet");
    return;
  }
  // render points in the current view
  // get current transform
  const auto query_time = info_ptr->header.stamp;
  tf2::Transform map_to_camera_tf;
  try {
    // block for up to 100ms to wait for sensor pose tfs
    // TODO(hengruiz): this is just a static transform from velodyne to camera
    // for now, need to change to "world" frame to camera once we move to
    // accumulated cloud
    const auto map_to_camera = tf_buffer.lookupTransform(
        info_ptr->header.frame_id, CLOUD_FRAME, query_time, ros::Duration(.1));
    tf2::fromMsg(map_to_camera.transform, map_to_camera_tf);
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return;
  }

  // form depth image and publish
  cv_bridge::CvImage lidar_depth_bridge;
  lidar_depth_bridge.header = info_ptr->header;
  lidar_depth_bridge.encoding = "16UC1";
  lidar_depth_bridge.image =
      renderer.render(*info_ptr, map_to_camera_tf, bloat_factor);
  lidar_depth_pub.publish(lidar_depth_bridge.toImageMsg());

  SHOW_ACTIVITY(stderr, true);
}

void LidarDepthRendererNode::initialize_sub() {
  // velodyne subscriber
  lidar_sub =
      nh.subscribe(lidar_topic, 10, &LidarDepthRendererNode::lidar_cb, this);
  // camera_info subscriber
  camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
      camera_info_topic, 1,
      boost::bind(&LidarDepthRendererNode::camera_info_cb, this, _1, it));
}

void LidarDepthRendererNode::initialize_pub() {
  // depth image publisher
  lidar_depth_pub = it.advertise(pub_topic, 1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_renderer_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  track_activity(true);

  ROS_INFO("Instantiating LidarDepthRendererNode");
  LidarDepthRendererNode renderer_node(&node_handle, &private_node_handle);

  ros::spin();
  return 0;
}
