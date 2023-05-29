#include <array>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/OccupancyGrid.h>
#include "clustering.h"


using namespace std;
using namespace pcl;

float roiM = 100;
int kernelSize = 3;

// Initialize the Grid state
void mapCartesianGrid(PointCloud<PointXYZ>::Ptr elevatedCloud,
                             array<array<int, numGrid>, numGrid> & cartesianData){


    array<array<int, numGrid>, numGrid> gridNum{};  // gridNum This array specifically counts the number of point clouds falling on each grid. gridNums[250][250] 
                                                    // refers to: the XY plane of elevatedCloud points is discrete into a grid of m×n units
    for(int cellX = 0; cellX < numGrid; cellX++){    // cellX
        for(int cellY = 0; cellY < numGrid; cellY++){   // cellY
            gridNum[cellX][cellY] = 0;
        }
    }
    
    // elevatedCloud maps to the Cartesian coordinate system and counts how many points fall on this grid!!
    for(int i = 0; i < elevatedCloud->size(); i++){ 
        float x = elevatedCloud->points[i].x; 
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM/2; 
        float yC = y+roiM/2;
        // exclude outside roi points 
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
        int xI = floor(numGrid*xC/roiM);   //  xI .yI    const int numGrid = 250; 
        int yI = floor(numGrid*yC/roiM); 
        gridNum[xI][yI] = gridNum[xI][yI] + 1;  //Count how many points fall on this grid
        // cout << "gridNum[xI][yI]: " << gridNum[xI][yI] << "------------- count how many points fall on this grid!!!------------" << endl; // gridNum[xI][yI]    //    cartesianData[xI][yI] = -1;

    }
//A single cell at x, y position is selected as the center cell, and the clusterID counter is incremented by 1.
// Then go through all adjacent neighbor cells (i.e. x-1, y+1, x, y+1, x+1, y+1 x -1, y, x+1, y, x-1, y- 1, x, check the occupancy status of y − 1, x + 1, y + 1) and mark it with the current cluster ID.
//Repeat this process for each x,y in the mxn grid until all non-empty clusters have been assigned IDs.
    for(int xI = 0; xI < numGrid; xI++){  //   const int numGrid = 250; 
        for(int yI = 0; yI < numGrid; yI++){
            if(gridNum[xI][yI] > 1){ 
                cartesianData[xI][yI] = -1;   //Grid allocation has 2 initial states, empty (0), occupied (-1) and allocated. Subsequently, a single cell at x, y position is selected as the center cell, and the clusterID counter is incremented by 1
                //The following is to set the value of the surrounding points of the current point to -1
                if(xI == 0)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI+1][yI] = -1;  //3 neighboring cells adjacent to the corner
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI][yI-1] = -1; 
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI == numGrid - 1) 
                    {
                        cartesianData[xI][yI-1] = -1; 
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;    
                    }
                }
                else if(xI < numGrid - 1)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;                
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI-1] = -1;
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;                  
                    }
                    else if(yI == numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI][yI-1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;                 
                    } 
                }
                else if(xI == numGrid - 1)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI+1] = -1;
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI-1] = -1;
                        cartesianData[xI][yI+1] = -1;
                    }
                    else if(yI == numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI][yI-1] = -1;    
                    }            
                }

            }
           
            
        }
    }
}

// findComponent will refer to the search function
void search(array<array<int, numGrid>, numGrid> & cartesianData, int clusterId, int cellX, int cellY){   //  cellX(0-249), cellY(0-249)
    cartesianData[cellX][cellY] = clusterId;
    int mean = kernelSize/2;   // kernelSize = 3;  mean  = 1 
    for (int kX = 0; kX < kernelSize; kX++){   // kernelSize = 3;
        int kXI = kX-mean; //    0， -1 ， 1 //   cout << "kXI  is "<<kXI<<endl; 
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid) continue;   // numGrid = 250;
        for( int kY = 0; kY < kernelSize;kY++){
            int kYI = kY-mean; 
            if((cellY + kYI) < 0 || (cellY + kYI) >= numGrid) continue;

            if(cartesianData[cellX + kXI][cellY + kYI] == -1){
                search(cartesianData, clusterId, cellX +kXI, cellY + kYI); 
            }

        }
    }
}

void findComponent(array<array<int, numGrid>, numGrid> & cartesianData, int &clusterId){
    for(int cellX = 0; cellX < numGrid; cellX++){  // numGrid = 250;
        for(int cellY = 0; cellY < numGrid; cellY++){
            if(cartesianData[cellX][cellY] == -1){ 
                clusterId ++;    // cout << "clusterId is "<< clusterId <<endl; 
                search(cartesianData, clusterId, cellX, cellY); 
            }
        }
    }
}

void componentClustering(PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                         array<array<int, numGrid>, numGrid> & cartesianData,
                         int & numCluster){

    mapCartesianGrid(elevatedCloud, cartesianData); // The first step is to set the state of the grid Grid grid array: cartesianData
    findComponent(cartesianData, numCluster); 
}


void makeClusteredCloud(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        PointCloud<pcl::PointXYZ>::Ptr& clusterCloud){
    for(int i = 0; i < elevatedCloud->size(); i++){
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float z = elevatedCloud->points[i].z;
        float xC = x+roiM/2;
        float yC = y+roiM/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
        int xI = floor(numGrid*xC/roiM);  // (0~249)
        int yI = floor(numGrid*yC/roiM);  // (0~249)


        int clusterNum = cartesianData[xI][yI];
        if(clusterNum != 0){
            PointXYZ o;
            o.x = grid_size*xI - roiM/2 + grid_size/2;
            o.y = grid_size*yI - roiM/2 + grid_size/2; 
            o.z = -1; 

            clusterCloud->push_back(o); 
        }
    }
}
