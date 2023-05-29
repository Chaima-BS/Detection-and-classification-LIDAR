#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"

#include <vector>
#include <iostream>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <utility>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include "ground_removal.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;

int counta = 0;
ros::Publisher ground_pub; // Publisher
ros::Publisher none_ground_pub;
ros::Publisher all_points_pub;

float filter_z_max;
float filter_z_min;
pcl::ConditionalRemoval<pcl::PointXYZ> condrem;

// Set restrictions: delete data points in the point cloud that do not meet multiple conditions specified by the user.
void filter_mid_area_limitation()
{
  // Create conditional filters

  //GT greater than
  //LT less than
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -15));  // Add a comparison operator greater than -15 on the x field
  range_cond->addComparison(cond_1);
  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 15)); //Add a comparison operator that is less than 5 on the x field
  range_cond->addComparison(cond_2);

  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_3(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -50));
  range_cond->addComparison(cond_3);

  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_4(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 15));
  range_cond->addComparison(cond_4);

  condrem.setCondition(range_cond);  //Deletes data points in a point cloud that do not meet multiple criteria specified.
  //Create a filter and initialize it with a condition definition object
}

//The cloud_cb callback function calls this function
void filter_mid_area(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  condrem.setInputCloud(cloud_in);
  condrem.setKeepOrganized(false);
  condrem.filter(*cloud_in);
}
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{ 
  PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  ;
  PointCloud<pcl::PointXYZ>::Ptr elevatedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Convert from ros msg to PCL::PointCloud data type
  fromROSMsg(*input, *cloud);
  //start processing pcl::pointcloud

  //Filter out points above a certain height
  PointCloud<pcl::PointXYZ>::Ptr z_filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;             //set filter object
  pass.setInputCloud(cloud);                        
  pass.setFilterFieldName("z");                     //Set the z field of the point cloud type required for filtering
  pass.setFilterLimits(filter_z_min, filter_z_max); //Set the range on the filter field (-3.0~1.0)
  //pass.setFilterLimitsNegative (true); 
  pass.filter(*z_filter_cloud); //Execute filtering and save the filtering results in cloud_filtered

  filter_mid_area(z_filter_cloud); //ConditionalRemoval filter Conditional filter (filtering on x, y axis)
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*z_filter_cloud, ros_cloud); //convert back to ros data format
  ros_cloud.header = input->header;
  all_points_pub.publish(ros_cloud); //Publisher ros_cloud message

  //Ground removal, separate ground points and non-ground points
  groundRemove(z_filter_cloud, elevatedCloud, groundCloud);

  // Convert from PCL::PointCloud to ROS data type
  elevatedCloud->header.frame_id = cloud->header.frame_id;
  groundCloud->header.frame_id = cloud->header.frame_id;
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 output2;
  toROSMsg(*groundCloud, output);  
  toROSMsg(*elevatedCloud, output2);

  none_ground_pub.publish(output2);
  ground_pub.publish(output);

  counta++;
  cout << "----------------------------- ground Frame: " << counta << " --------------------------------" << endl;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "remove_ground");
  ros::NodeHandle nh;

  // ros::Subscriber sub = nh.subscribe("/kitti/velo/pointcloud", 160, cloud_cb); 
  ros::Subscriber sub = nh.subscribe("velodyne_points", 160, cloud_cb); 
  nh.param<float>("filter_z_max", filter_z_max, 1.0);                 
  nh.param<float>("filter_z_min", filter_z_min, -3.0);                  // nh.param<std::string>("default_param", default_param, "default_value");
  filter_mid_area_limitation();      

  // Create a ROS publisher for the output point cloud
  ground_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_topic", 1);
  none_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("none_ground_topic", 1);
  all_points_pub = nh.advertise<sensor_msgs::PointCloud2>("all_points", 1);
  // Spin
  ros::spin();
}