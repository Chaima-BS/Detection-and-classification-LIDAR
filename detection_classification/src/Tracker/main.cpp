
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

#include <object_tracking/trackbox.h>

#include "imm_ukf_jpda.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

int counta = 0;

ros::Publisher pub;

ros::Publisher vis_pub;

tf::TransformListener* tran;

double yaw_gps;

double v_gps;


void  cloud_cb (const object_tracking::trackbox& input){


  counta ++;
  cout << "Frame: "<<counta << "----------------------------------------"<< endl;

  // --------------------convert local to global-------------------------
  double timestamp = input.header.stamp.toSec(); 
  vector<vector<double>> egoPoints;
  getOriginPoints(timestamp, egoPoints,v_gps,yaw_gps); 
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(egoPoints[0][0], egoPoints[0][1], 0.0) );
  tf::Quaternion q;
  ros::Time input_time = input.header.stamp;
  q.setRPY(0, 0, egoPoints[0][2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, input_time, "velodyne", "global"));
  
  // tf::StampedTransform transform2;
  //   try{
  //     tran->lookupTransform("/global", "/velodyne", input_time, transform2);
  //   }
  //   catch (tf::TransformException &ex) {
  //     ROS_ERROR("%s",ex.what());
  //   }


  int box_num = input.box_num;

  vector<PointCloud<PointXYZ>> bBoxes;
  PointCloud<PointXYZ> oneBbox;

  for(int box_i = 0;box_i < box_num; box_i++)
  {
      PointXYZ o;
      o.x = input.x1[3*box_i];
      o.y = input.x1[3*box_i + 1];
      o.z = input.x1[3*box_i + 2];
      oneBbox.push_back(o);
      o.x = input.x2[3*box_i];
      o.y = input.x2[3*box_i + 1];
      o.z = input.x2[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.x3[3*box_i + 0];
      o.y = input.x3[3*box_i + 1];
      o.z = input.x3[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.x4[3*box_i + 0];
      o.y = input.x4[3*box_i + 1];
      o.z = input.x4[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y1[3*box_i + 0];
      o.y = input.y1[3*box_i + 1];
      o.z = input.y1[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y2[3*box_i + 0];
      o.y = input.y2[3*box_i + 1];
      o.z = input.y2[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y3[3*box_i + 0];
      o.y = input.y3[3*box_i + 1];
      o.z = input.y3[3*box_i + 2];   
      oneBbox.push_back(o);
      o.x = input.y4[3*box_i + 0];
      o.y = input.y4[3*box_i + 1];
      o.z = input.y4[3*box_i + 2];   
      oneBbox.push_back(o);
      bBoxes.push_back(oneBbox);
      oneBbox.clear();
  }
  

  PointCloud<PointXYZ> newBox;
  for(int i = 0; i < bBoxes.size(); i++ ){
    bBoxes[i].header.frame_id = "velodyne";

    // try {
    //   tran->waitForTransform("/global", "/velodyne", input_time, ros::Duration(10.0));
    //   tran->lookupTransform("/global", "/velodyne", input_time, transform2);
    // } catch (tf::TransformException ex) {
    //   ROS_ERROR("%s",ex.what());
    // }
    
    tran->waitForTransform("/global", "/velodyne", input_time, ros::Duration(10.0));

    pcl_ros::transformPointCloud("/global", bBoxes[i], newBox, *tran);
    bBoxes[i] = newBox;
  }
  // ----------------------------end converting----------------------------------------
  PointCloud<PointXYZ> targetPoints;
  vector<vector<double>> targetVandYaw;

  // The main purpose of trace management is to dynamically limit the number of spurious trace lists 
  // (thus preventing false data associations) and keep object traces in case of loss detection 
  vector<bool> isStaticVec;
  vector<int> trackManage;  
  vector<bool> isVisVec;
  vector<PointCloud<PointXYZ>> visBBs;
  immUkfJpdaf(bBoxes, timestamp, targetPoints, targetVandYaw, trackManage, isStaticVec, isVisVec, visBBs);

  assert(targetPoints.size() == trackManage.size());
  assert(targetPoints.size()== targetVandYaw.size());

  

  // --------------converting from global to ego tf for visualization------------------
  // processing targetPoints
  PointCloud<PointXYZ> egoTFPoints;
  targetPoints.header.frame_id = "global";
  pcl_ros::transformPointCloud("/velodyne", targetPoints, egoTFPoints, *tran);

  //processing visBBs
  PointCloud<PointXYZ> visEgoBB;
  for(int i = 0; i < visBBs.size(); i++){
    visBBs[i].header.frame_id = "global";
    pcl_ros::transformPointCloud("/velodyne", visBBs[i], visEgoBB, *tran);
    
    visBBs[i] = visEgoBB;
  }
  //------------------------end converting to ego tf-------------------------



  // -----------------------tracking arrows visualizing start------------------------------------
  for(int i = 0; i < targetPoints.size(); i++){
    visualization_msgs::Marker arrowsG;
    arrowsG.lifetime = ros::Duration(0.1);
    if(trackManage[i] == 0 ) {
      continue;
    }
    if(isVisVec[i] == false ) {
      continue;
    }
    if(isStaticVec[i] == true){
      continue;
    }
    arrowsG.header.frame_id = "velodyne";
    
    arrowsG.header.stamp= ros::Time::now();
    arrowsG.ns = "arrows";
    arrowsG.action = visualization_msgs::Marker::ADD;
    arrowsG.type =  visualization_msgs::Marker::ARROW;
    // green
    arrowsG.color.g = 1.0f;
    arrowsG.color.a = 1.0;  
    arrowsG.id = i;
    geometry_msgs::Point p;
    // assert(targetPoints[i].size()==4);
    p.x = egoTFPoints[i].x;
    p.y = egoTFPoints[i].y;
    p.z = -1.73/2;
    double tv   = targetVandYaw[i][0];
    double tyaw = targetVandYaw[i][1];

    // Set the pose of the marker
    arrowsG.pose.position.x = p.x;
    arrowsG.pose.position.y = p.y;
    arrowsG.pose.position.z = p.z;

    // convert from 3 angles to quartenion
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    arrowsG.pose.orientation.x = q_tf.getX();
    arrowsG.pose.orientation.y = q_tf.getY();
    arrowsG.pose.orientation.z = q_tf.getZ();
    arrowsG.pose.orientation.w = q_tf.getW();

    // Set the scale of the arrowsG 
    arrowsG.scale.x = tv;
    arrowsG.scale.y = 0.1;
    arrowsG.scale.z = 0.1;

    vis_pub.publish(arrowsG);  //post arrow message
// -----------------------tracking arrows visualizing end------------------------------------
  }

  
  // ------------------tracking points visualizing start---------------------------------------------
  
  visualization_msgs::Marker pointsY, pointsG, pointsR, pointsB;
  pointsY.header.frame_id = pointsG.header.frame_id = pointsR.header.frame_id = pointsB.header.frame_id = "velodyne";
  
  pointsY.header.stamp= pointsG.header.stamp= pointsR.header.stamp =pointsB.header.stamp = ros::Time::now();
  pointsY.ns= pointsG.ns = pointsR.ns =pointsB.ns=  "points";
  pointsY.action = pointsG.action = pointsR.action = pointsB.action = visualization_msgs::Marker::ADD;
  pointsY.pose.orientation.w = pointsG.pose.orientation.w  = pointsR.pose.orientation.w =pointsB.pose.orientation.w= 1.0;

  pointsY.id = 1;
  pointsG.id = 2;
  pointsR.id = 3;
  pointsB.id = 4;
  pointsY.type = pointsG.type = pointsR.type = pointsB.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  pointsY.scale.x =pointsG.scale.x =pointsR.scale.x = pointsB.scale.x=0.5;
  pointsY.scale.y =pointsG.scale.y =pointsR.scale.y = pointsB.scale.y = 0.5;

  // yellow 
  pointsY.color.r = 1.0f;
  pointsY.color.g = 1.0f;
  pointsY.color.b = 0.0f;
  pointsY.color.a = 1.0;

  // green
  pointsG.color.g = 1.0f;
  pointsG.color.a = 1.0;

  // red
  pointsR.color.r = 1.0;
  pointsR.color.a = 1.0;

  // blue 
  pointsB.color.b = 1.0;
  pointsB.color.a = 1.0;


  for(int i = 0; i < targetPoints.size(); i++){
    if(trackManage[i] == 0) continue;
    geometry_msgs::Point p;

    p.x = egoTFPoints[i].x;
    p.y = egoTFPoints[i].y;
    p.z = -1.73/2;

//   cout << "is ------------------" << i <<endl;
    // cout << "trackManage[i]  " <<trackManage[i] << endl;
    if(isStaticVec[i] == true){ 
      pointsB.points.push_back(p);    // Blue
    }
    else if(trackManage[i] < 5 ){  // Yellow
      pointsY.points.push_back(p);
    }
    else if(trackManage[i] == 5){  // Green
      pointsG.points.push_back(p);
    }
    else if(trackManage[i] > 5){
      pointsR.points.push_back(p);    // Red
    }
  }
  vis_pub.publish(pointsY);  
  // cout << "pointsG" << pointsG.points[0].x << " "<< pointsG.points[0].y << endl;
  vis_pub.publish(pointsG); 
  vis_pub.publish(pointsR);  
  vis_pub.publish(pointsB);  
  // ----------------------tracking points visualizing end---------------------------------------------
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "tracker");
  ros::NodeHandle nh;

  tf::TransformListener lr(ros::Duration(100)); // How long to store transform information
  tran=&lr;

  // Create a ROS subscriber for the input point cloud

  ros::Subscriber sub = nh.subscribe ("track_box", 160, cloud_cb); 

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);  // visualization_marker 

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 ); 

  ros::spin ();
}