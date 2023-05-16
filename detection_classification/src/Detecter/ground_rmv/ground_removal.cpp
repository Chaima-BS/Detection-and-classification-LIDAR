
#include <pcl/io/pcd_io.h>

#include "ground_removal.h"
#include "gaussian_blur.h"

using namespace std;
using namespace Eigen;
using namespace pcl;


float rMin = 0.2;
float rMax = 100;
float tHmin = -3.5; 
float tHmax = 2.0;
float tHDiff = 0.2;
float hSensor = 0.25;

Cell::Cell(){
    minZ = 1000;
    isGround = false;
}

void Cell::updateMinZ(float z) {
    if (z < minZ) minZ = z;
}


// Radius distance around the filter car（（ rMin = 0.2; rMax = 100;））
void filterCloud(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ> & filteredCloud){
    for (int i = 0; i < cloud->size(); i++) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        float distance = sqrt(x * x + y * y);
        if(distance <= rMin || distance >= rMax) { // checking the Euclidean distance, filtering 
                                                   //and removing abnormal points （ rMin = 0.2; rMax = 100;）
            continue; // filter out
        }
        else{
            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            filteredCloud.push_back(o);
        }
    }
}

// xy coordinates, converted to chP binP coordinates
void getCellIndexFromPoints(float x, float y, int& chI, int& binI){
    float distance = sqrt(x * x + y * y);  // Euclidean distance
    //normalize(polar coordinates)
    float chP = (atan2(y, x) + M_PI) / (2 * M_PI);   // Range (0, 1) atan2(y, x) is 4-quadrant arc tangent    M_PI==3.14
    float binP = (distance - rMin) / (rMax - rMin); // Range (0, 1)  rMax - rMin ~~ (0.2, 100)
    //index
    chI = floor(chP*numChannel);   // numChannel = 16 (The angle is divided into 16 parts)
    binI = floor(binP*numBin);     //    numBin = 2000 radius is divided into 360 parts）
//    cout << "bin ind: "<<binI << " ch ind: "<<chI <<endl;
}

// Create mapped polar grid
void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin>, numChannel>& polarData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);  //  The xy coordinates get the corresponding grid coordinates to get the CellIndex  ： chI, binI（Polar coordinates: angles and radius）
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue; // to prevent segmentation fault remove unreasonable points
        polarData[chI][binI].updateMinZ(z);  // {if (z < minZ) minZ = z;} 
    }
}

// update HDiff with larger value
void computeHDiffAdjacentCell(array<Cell, numBin>& channelData){
    //    std::cout << " channelData.size()   "<< channelData.size() << std::endl;
    for(int i = 0; i < channelData.size(); i++){   
        // edge case
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight(); // difference
            channelData[i].updateHDiff(hD); // height difference per grid
        }
        else if(i == channelData.size()-1){
            float hD = channelData[i].getHeight() - channelData[i-1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        // non-edge case
        else{
            float preHD  = channelData[i].getHeight() - channelData[i-1].getHeight();
            float postHD = channelData[i].getHeight() - channelData[i+1].getHeight();
            if(preHD > postHD) channelData[i].updateHDiff(preHD);
            else channelData[i].updateHDiff(postHD);
        }

    //    cout <<channelData[i].getHeight() <<" " <<channelData[i].getHDiff() << endl; // 2, 0...
    }
}

 // Median filtering deals with missing ground information (common due to occlusion),
 // and as the name suggests, the height values ​​of missing cells are replaced with the median values ​​of neighboring cells.
void applyMedianFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size()-1; channel++){
        for(int bin = 1; bin < polarData[0].size()-1; bin++){
            if(!polarData[channel][bin].isThisGround()){  
                // target cell is non-ground AND surrounded by ground cells
                if(polarData[channel][bin+1].isThisGround()&&  // surrounded by ground
                   polarData[channel][bin-1].isThisGround()&&
                   polarData[channel+1][bin].isThisGround()&&
                   polarData[channel-1][bin].isThisGround())
                   {
                    vector<float> sur{
                                      polarData[channel][bin+1].getHeight(),   // target cell is non-ground AND surrounded by ground cells
                                      polarData[channel][bin-1].getHeight(),
                                      polarData[channel+1][bin].getHeight(),
                                      polarData[channel-1][bin].getHeight()
                                      };
                    sort(sur.begin(), sur.end()); // Sort from smallest to largest
                    float m1 = sur[1]; float m2 = sur[2];// Take the middle 2 values
                    float median = (m1+m2)/2;  // Take the median
                    polarData[channel][bin].updataHeight(median);  //The height value of a missing cell 
                                                                    // will be replaced by the median value of the adjacent cells
                    polarData[channel][bin].updateGround(); 
                }
            }
        }
    }
}

 // smoothen spot with outlier
void outlierFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size() - 1; channel++) {
        for (int bin = 1; bin < polarData[0].size() - 2; bin++) {
            if(polarData[channel][bin].isThisGround()&& 
               polarData[channel][bin+1].isThisGround()&&
               polarData[channel][bin-1].isThisGround()&&
               polarData[channel][bin+2].isThisGround())
            {
                float height1 = polarData[channel][bin-1].getHeight();
                float height2 = polarData[channel][bin].getHeight(); 
                float height3 = polarData[channel][bin+1].getHeight();
                float height4 = polarData[channel][bin+2].getHeight();
                if(height1 != tHmin && height2 == tHmin && height3 != tHmin){ // float tHmin = -2.0; 
                    float newH = (height1 + height3)/2; //Take the mean of both sides
                    polarData[channel][bin].updataHeight(newH);  // Update height value
                    polarData[channel][bin].updateGround(); //     void updateGround(){isGround = true; hGround = height;}
                }
                else if(height1 != tHmin && height2 == tHmin && height3 == tHmin && height4 != tHmin){
                    float newH = (height1 + height4)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}

void groundRemove(PointCloud<pcl::PointXYZ>::Ptr  cloud,  // initial point cloud
              PointCloud<pcl::PointXYZ>::Ptr  elevatedCloud, 
              PointCloud<pcl::PointXYZ>::Ptr  groundCloud){ 

    PointCloud<pcl::PointXYZ> filteredCloud;

    cout << "------------------Starting the ground removal-----------------------" << endl;  

    // checking the Euclidean distance, filtering and removing abnormal points outside
    // the radius around the car（ rMin = 0.2; rMax = 100;）
    filterCloud(cloud, filteredCloud);  
    array<array<Cell, numBin>, numChannel> polarData;  // Array definition polarData[numChannel][numBin], Cell is a class
    createAndMapPolarGrid(filteredCloud, polarData);   //Polar Grid Mapping

    //  polarData.size()  16   channel
    //  polarData[0].size()  2000  bin
    for (int channel = 0; channel < polarData.size(); channel++){   // channel: 16
        for (int bin = 0; bin < polarData[0].size(); bin ++){   //  2000
            float zi = polarData[channel][bin].getMinZ();  //get the minimum
            if(zi > tHmin && zi < tHmax){polarData[channel][bin].updataHeight(zi);}   //Each Cell grid has an updataHeight
            else if(zi > tHmax){polarData[channel][bin].updataHeight(hSensor);}  // float hSensor = 0.25; height greater than 2, set to 0.25
            else {polarData[channel][bin].updataHeight(tHmin);} //  float tHmin = -2.0;  //Height is less than -2, set to -2
        }
        //  computeGradientAdjacentCell(polarData[channel]);
        gaussSmoothen(polarData[channel], 1, 3);  //Gaussian smoothing from src/ground_removal/gaussian_blur.cpp
    //    std::cout << " finished smoothing at channel "<< channel << std::endl; 
        computeHDiffAdjacentCell(polarData[channel]);  

        //Check if it is the ground
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            if(polarData[channel][bin].getSmoothed() < tHmax &&  // float tHmin = -2.0;
                    polarData[channel][bin].getHDiff() < tHDiff){   //  tHDiff = 0.2; The ground should be relatively smooth
                polarData[channel][bin].updateGround();  // void updateGround(){isGround = true; hGround = height;}
            }
            else if(polarData[channel][bin].getHeight() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }
    // implement MedianFilter
    // Median filtering deals with missing ground information (common due to occlusion),
    // as the name suggests, the height values of missing cells are replaced with the median values of neighboring cells
    applyMedianFilter(polarData);
    // smoothen spot with outlier
    outlierFilter(polarData);

    for(int i = 0; i < filteredCloud.size(); i++) {
        float x = filteredCloud.points[i].x;
        float y = filteredCloud.points[i].y;
        float z = filteredCloud.points[i].z;

        pcl::PointXYZ o;
        o.x = x;
        o.y = y;
        o.z = z;
        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        // assert(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin);
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue;
        
        if (polarData[chI][binI].isThisGround()) {
            float hGround = polarData[chI][binI].getHGround();
            // std::cout << " hGround "<< hGround << std::endl;
            // std::cout << " z "<< z << std::endl; 
            if (z < (hGround + 0.25)) { 
                groundCloud->push_back(o);
            } else {
                elevatedCloud->push_back(o);
            }
        } else {
            elevatedCloud->push_back(o); 
        }
    }
      cout << "Initial point cloud size: "<<cloud->size() << " Processed high point size: "<<elevatedCloud->size() << " Processed ground point size: "<<groundCloud->size( )<<endl;
}