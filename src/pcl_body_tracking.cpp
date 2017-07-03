#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_body_tracking/lidar_body_trackingConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PCLHeader.h>

#include <sensor_msgs/PointCloud2.h>

// Topics
static const std::string SCAN_TOPIC = "/QP308/pc_QP308";
static const std::string FILTERED_TOPIC = "/pcl_filtered";
static const std::string FILTER_TOPIC = "/pcl_filter";

// Variables
int LEAF_SIZE = 10;
float RESOLUTION = 0.1f;
int MIN_FILTERED_CLOUD_SIZE = 50;

// ROS Publisher
ros::Publisher pub_filtered, pub_filter, pub_clustered;
bool first_msg = true;
sensor_msgs::PointCloud2 msg_clustered, msg_filtered, msg_filter;
pcl::PointCloud<pcl::PointXYZ> cloud, filter;
pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ptr, cloud_ptr;

void dynrcfg_callback(lidar_body_tracking::lidar_body_trackingConfig &config, uint32_t level) {
  LEAF_SIZE = config.leaf_size;
  RESOLUTION = config.resolution;
  MIN_FILTERED_CLOUD_SIZE = config.min_filtered_cloud_size;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert to PCL data type
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*cloud_msg, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (RESOLUTION);
  cloud_ptr = cloud.makeShared();

  if (first_msg){
    first_msg = false;
    filter_ptr = cloud.makeShared();
  }

  octree.setInputCloud(filter_ptr);
  octree.addPointsFromInputCloud();

  octree.switchBuffers();

  octree.setInputCloud(cloud_ptr);
  octree.addPointsFromInputCloud();

  std::vector<int> newPointVector;
  octree.getPointIndicesFromNewVoxels(newPointVector, LEAF_SIZE);

  ROS_INFO("indicies %d", newPointVector.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  filtered_cloud = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>();
  filtered_cloud->points.reserve(newPointVector.size());

  for (std::vector<int>::iterator it = newPointVector.begin (); it != newPointVector.end (); it++)
    filtered_cloud->points.push_back(cloud_ptr->points[*it]);

  pcl::toROSMsg(*filtered_cloud, msg_filtered);
  msg_filtered.header = cloud_msg->header;
  pub_filtered.publish(msg_filtered);

  pcl::toROSMsg(filter, msg_filter);
  msg_filter.header = cloud_msg->header;
  pub_filter.publish(msg_filter);

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "lidar_body_extraction");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<lidar_body_tracking::lidar_body_trackingConfig> server;
  dynamic_reconfigure::Server<lidar_body_tracking::lidar_body_trackingConfig>::CallbackType f;

  f = boost::bind(&dynrcfg_callback, _1, _2);
  server.setCallback(f);

  ros::Subscriber sub = nh.subscribe(SCAN_TOPIC, 1, cloud_cb);

  pub_filtered = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_TOPIC, 1);
  pub_filter = nh.advertise<sensor_msgs::PointCloud2>(FILTER_TOPIC, 1);

  ROS_INFO_STREAM("Spinning");

  ros::spin();

  return 0;
}

