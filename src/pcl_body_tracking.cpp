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
static const std::string CLUSTERED_TOPIC = "/pcl_clustered";

// Variables
int LEAF_SIZE = 10;
float RESOLUTION = 0.1f;
int MIN_FILTERED_CLOUD_SIZE = 50;
int MIN_CLUSTERED_CLOUD_SIZE = 50;
float CLUSTER_TOLERANCE = 0.07;
int MIN_CLUSTER_SIZE = 100;
int MAX_CLUSTER_SIZE = 25000;

// ROS Publisher
ros::Publisher pub_filtered, pub_filter, pub_clustered;
bool first_msg = true;
sensor_msgs::PointCloud2 msg_clustered, msg_filtered, msg_filter;
pcl::PointCloud<pcl::PointXYZ> cloud, filter;
pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ptr, cloud_ptr;

void dynrcfg_callback(lidar_body_tracking::lidar_body_trackingConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %d %d", 
            config.leaf_size,
						config.resolution, 
            config.min_filtered_cloud_size,
            config.min_clustered_cloud_size);
  LEAF_SIZE = config.leaf_size;
  RESOLUTION = config.resolution;
  MIN_FILTERED_CLOUD_SIZE = config.min_filtered_cloud_size;
  MIN_CLUSTERED_CLOUD_SIZE = config.min_clustered_cloud_size;
  CLUSTER_TOLERANCE = config.cluster_tolerance;
  MIN_CLUSTER_SIZE = config.min_cluster_size;
  MAX_CLUSTER_SIZE = config.max_cluster_size;
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

  std::vector<pcl::PointIndices> cluster_indices;
  if(filtered_cloud->size() > MIN_FILTERED_CLOUD_SIZE){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (filtered_cloud);

    // ClusterExtraction
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE); // 7cm
    ec.setMinClusterSize (MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered_cloud);
    ec.extract (cluster_indices);
    ROS_INFO("cluster size %d", cluster_indices.size());
  }

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (filtered_cloud->points[*pit]); //*
      ROS_INFO("PointCloud representing the Cluster: %d data points.", cloud_cluster->points.size());

      if(cloud_cluster->points.size() > MIN_CLUSTERED_CLOUD_SIZE){
          pcl::toROSMsg(*cloud_cluster, msg_clustered);
          msg_clustered.header = cloud_msg->header;
          pub_clustered.publish(msg_clustered);
      }
  }

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
  pub_clustered = nh.advertise<sensor_msgs::PointCloud2>(CLUSTERED_TOPIC, 1);

  ROS_INFO_STREAM("Spinning");

  ros::spin();

  return 0;
}

