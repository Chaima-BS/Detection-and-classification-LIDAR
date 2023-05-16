
#ifndef COMPONENT_CLUSTERING_H
#define COMPONENT_CLUSTERING_H

#include <array>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace pcl;

const int numGrid = 500;
const float grid_size = 0.1;   

extern float roiM;
extern int kernelSize;

void componentClustering(PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                         array<array<int, numGrid>, numGrid> & cartesianData,
                         int & numCluster);

void mapCartesianGrid(PointCloud<PointXYZ>::Ptr elevatedCloud,
                            array<array<int, numGrid>, numGrid> & cartesianData);

void makeClusteredCloud(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        PointCloud<pcl::PointXYZ>::Ptr& clusterCloud);

#endif
