#ifndef GROUND_REMOVAL_H
#define GROUND_REMOVAL_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//#include "gaussian_blur.h"

using namespace std;
using namespace pcl;

const int numChannel = 16;
const int numBin = 2000;
extern float rMin;
extern float rMax;
extern float tHmin;
extern float tHmax;

extern float tHDiff;
extern float hSensor;

class Cell{
private:
    float smoothed;
    float height;
    float hDiff;
    float hGround;
    float minZ;
    bool isGround;

public:
    Cell();
    void updateMinZ(float z);
    void updataHeight(float h) {height = h;}
    void updateSmoothed(float s) {smoothed = s;}
    void updateHDiff(float hd){hDiff = hd;}
    void updateGround(){isGround = true; hGround = height;}
    bool isThisGround(){return isGround;}
    float getMinZ() {return minZ;}
    float getHeight(){return height;}
    float getHDiff(){ return hDiff;}
    float getSmoothed() {return smoothed;}
    float getHGround() {return hGround;}
};




void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin>, numChannel>& polarData );

void computeHDiffAdjacentCell(array<Cell, numBin>& channelData);

void groundRemove(PointCloud<pcl::PointXYZ>::Ptr cloud, 
                  PointCloud<pcl::PointXYZ>::Ptr elevatedCloud, 
                  PointCloud<pcl::PointXYZ>::Ptr groundCloud); 


#endif 
