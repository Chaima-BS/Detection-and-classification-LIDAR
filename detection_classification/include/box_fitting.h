
#ifndef BOX_FITTING_H
#define BOX_FITTING_H

#include <array>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "clustering.h"

using namespace std;
using namespace pcl;

extern float picScale; // picScale * roiM = 30 * 30
extern int ramPoints;
extern int lSlopeDist;
extern int lnumPoints;

extern float tHeightMin;
extern float tHeightMax;
extern float tWidthMin;
extern float tWidthMax;
extern float tLenMin;
extern float tLenMax;
extern float tAreaMax;
extern float tRatioMin;
extern float tRatioMax;
extern float minLenRatio;
extern float tPtPerM3;

vector<PointCloud<PointXYZ>> boxFitting(PointCloud<PointXYZ>::Ptr elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        int numCluster,visualization_msgs::MarkerArray& ma);

#endif 
