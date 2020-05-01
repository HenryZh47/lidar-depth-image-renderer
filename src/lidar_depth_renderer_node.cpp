#include "lidar_depth_renderer/lidar_depth_renderer_node.h"
#include "instrument.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define RENDER_PERF_STATS (1)

/* Framerate variables */
static std::chrono::milliseconds delta_time;

static double clockToMilliseconds(clock_t ticks);

LidarDepthRendererNode::LidarDepthRendererNode(
    ros::NodeHandle *node_handle, ros::NodeHandle *private_node_handle)
    : nh(*node_handle),
      pnh(*private_node_handle),
      it(*node_handle),
      tf_listener(tf_buffer) {
  setup_ros();
  initialize_sub();
  initialize_pub();

  out_im_width = 0;
  out_im_height = 0;

  // Query renderer library implementation
  _implementation = renderer.query_omp();
  switch (_implementation) {
    case 0:
      ROS_INFO("[Library]: Serial");
      break;

    case 1:
      ROS_INFO("[Library]: OpenMP");
      break;

    default:
      ROS_WARN("[Library]: Unknown");
      break;
  }
}

void LidarDepthRendererNode::setup_ros() {
  // read in ros params
  pnh.getParam("lidar_topic", lidar_topic);
  pnh.getParam("camera_info_topic", camera_info_topic);
  pnh.getParam("pub_topic", pub_topic);

  // load cloud size parameters
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
  pcl_cloud_ptr->reserve(cloud_ptr->width);
  pcl::fromPCLPointCloud2(pcl2_cloud, *pcl_cloud_ptr);

  // add to sliding window
  cloud_accumulator.add(pcl_cloud_ptr);

  renderer.set_cloud(cloud_accumulator.get_cloud());
  have_cloud = true;
}

void LidarDepthRendererNode::camera_info_cb(
    const sensor_msgs::CameraInfoConstPtr &info_ptr,
    const image_transport::ImageTransport image_it) {
  if (unlikely(out_im_width == 0 && out_im_height == 0)) {
    // First time initialization
    out_im_width = info_ptr->width;
    out_im_height = info_ptr->height;
    out_im =
        cv::Mat::zeros(out_im_height, out_im_width, CV_8UC1);  // (row, col)
    ROS_INFO("Initializing camera: %dx%d", out_im_width, out_im_height);
    renderer.init(out_im_height, out_im_width);
  }

  track_activity(true);

  if (!have_cloud) {
    ROS_WARN("Node hasn't received any pointcloud yet");
    return;
  }
  // render points in the current view
  // get current transform
  // minus an arbitrary half a second because we don't have timesync
  const auto query_time = info_ptr->header.stamp - ros::Duration(0.5);
  tf2::Transform map_to_camera_tf;
  try {
    // block for up to 100ms to wait for sensor pose tfs
    const auto map_to_camera = tf_buffer.lookupTransform(
        info_ptr->header.frame_id, CLOUD_FRAME, query_time, ros::Duration(.1));
    tf2::fromMsg(map_to_camera.transform, map_to_camera_tf);
  } catch (tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
    return;
  }

  // form depth image and publish
  cv_bridge::CvImage lidar_depth_bridge(info_ptr->header, "8UC1", out_im);

  std::chrono::milliseconds begin_time = get_time_ms();

  renderer.render(out_im, *info_ptr, map_to_camera_tf, bloat_factor);

  std::chrono::milliseconds end_time = get_time_ms();
  delta_time = end_time - begin_time;

#if (RENDER_PERF_STATS)
  char fps_str[30];
  memset(fps_str, '\n', 30);
  sprintf(fps_str, "Frame time: %ld ms", delta_time.count());
  cv::putText(out_im, fps_str, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
              cv::Scalar(0xFFFF));
#endif

  show_activity(stderr, true);

  track_activity(false);

  lidar_depth_pub.publish(lidar_depth_bridge.toImageMsg());
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

  ROS_INFO("Instantiating LidarDepthRendererNode");
  LidarDepthRendererNode renderer_node(&node_handle, &private_node_handle);

  ros::spin();
  return 0;
}

static double clockToMilliseconds(clock_t ticks) {
  // units/(units/time) => time (seconds) * 1000 = milliseconds
  return (ticks / (double)CLOCKS_PER_SEC) * 1000.0;
}
