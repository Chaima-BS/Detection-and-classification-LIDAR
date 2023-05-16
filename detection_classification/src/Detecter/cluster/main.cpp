#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>

#include <detection_classification/trackbox.h>

#include "clustering.h"
#include "box_fitting.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;

int counta = 0;

ros::Publisher pub;

ros::Publisher vis_pub;

ros::Publisher marker_array_pub_;

ros::Publisher box_pub;

// callback
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

  PointCloud<pcl::PointXYZ>::Ptr none_ground_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // Convert from ros msg to PCL::PointCloud data type
  fromROSMsg (*input, *none_ground_cloud);

  //start processing pcl::pointcloud
  int numCluster = 0; // global variable
  array<array<int, numGrid>, numGrid> cartesianData{};
  componentClustering(none_ground_cloud, cartesianData, numCluster);
  cout << "Cluster ID "<<numCluster<<endl; 

  // for visualization
  PointCloud<pcl::PointXYZ>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  makeClusteredCloud(none_ground_cloud, cartesianData, clusteredCloud);

  // Convert from PCL::PointCloud to ROS data type
  clusteredCloud->header.frame_id = none_ground_cloud->header.frame_id;
  sensor_msgs::PointCloud2 output;
  toROSMsg(*clusteredCloud, output); 


 
  pub.publish(output); 
  counta ++;
  cout << "cluster Frame: "<<counta << "----------------------------------------"<< endl; 

  visualization_msgs::MarkerArray ma; 
  vector<PointCloud<PointXYZ>> bBoxes = boxFitting(none_ground_cloud, cartesianData, numCluster,ma);  // bBoxes

  detection_classification::trackbox boxArray;
    
  boxArray.header = input->header;
  boxArray.box_num = bBoxes.size();
  
  for(int i = 0;i < bBoxes.size();i++)
  {
    boxArray.x1.push_back(bBoxes[i][0].x);
    boxArray.x1.push_back(bBoxes[i][0].y);
    boxArray.x1.push_back(bBoxes[i][0].z);

    boxArray.x2.push_back(bBoxes[i][1].x);
    boxArray.x2.push_back(bBoxes[i][1].y);
    boxArray.x2.push_back(bBoxes[i][1].z);

    boxArray.x3.push_back(bBoxes[i][2].x);
    boxArray.x3.push_back(bBoxes[i][2].y);
    boxArray.x3.push_back(bBoxes[i][2].z);

    boxArray.x4.push_back(bBoxes[i][3].x);
    boxArray.x4.push_back(bBoxes[i][3].y);
    boxArray.x4.push_back(bBoxes[i][3].z);

    boxArray.y1.push_back(bBoxes[i][4].x);
    boxArray.y1.push_back(bBoxes[i][4].y);
    boxArray.y1.push_back(bBoxes[i][4].z);

    boxArray.y2.push_back(bBoxes[i][5].x);
    boxArray.y2.push_back(bBoxes[i][5].y);
    boxArray.y2.push_back(bBoxes[i][5].z);

    boxArray.y3.push_back(bBoxes[i][6].x);
    boxArray.y3.push_back(bBoxes[i][6].y);
    boxArray.y3.push_back(bBoxes[i][6].z);

    boxArray.y4.push_back(bBoxes[i][7].x);
    boxArray.y4.push_back(bBoxes[i][7].y);
    boxArray.y4.push_back(bBoxes[i][7].z);
  }

//************************************cube visualiaztion******************************

  box_pub.publish(boxArray); 
  // cout << "boxArray is " << boxArray<< endl;  // bBoxes
  cout << "size of bBoxes is " << bBoxes.size() << endl;  //size of bBoxes is 2
  cout << "size of marker is " << ma.markers.size() << endl; // size of marker is 2
  marker_array_pub_.publish(ma); 
//************************************end of cube*************************************




//*********************************************bBoxes visualization***************************************

  visualization_msgs::Marker line_list; 
  line_list.header.frame_id = "velodyne";
  line_list.header.stamp = ros::Time::now();
  line_list.ns =  "boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST; 

  //LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.1;
  // Points are green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  int id = 0;string ids;
  for(int objectI = 0; objectI < bBoxes.size(); objectI ++){ 
    for(int pointI = 0; pointI < 4; pointI++){ 
      assert((pointI+1)%4 < bBoxes[objectI].size());
      assert((pointI+4) < bBoxes[objectI].size());
      assert((pointI+1)%4+4 < bBoxes[objectI].size());
      id ++; ids = to_string(id);
      geometry_msgs::Point p; 
      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);  
      p.x = bBoxes[objectI][(pointI+1)%4].x; 
      p.y = bBoxes[objectI][(pointI+1)%4].y;
      p.z = bBoxes[objectI][(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4+4].x;
      p.y = bBoxes[objectI][(pointI+1)%4+4].y;
      p.z = bBoxes[objectI][(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }

  //line list end
  vis_pub.publish(line_list); 
  // bounding box visualizing end---------------------------------------------

}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "cluster"); 
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("none_ground_topic", 160, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1); 

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("cluster_marker", 10);


  box_pub = nh.advertise<detection_classification::trackbox>("track_box",10); 

  // Spin
  ros::spin ();
}