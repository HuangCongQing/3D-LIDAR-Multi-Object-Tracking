//
// Created by kosuke on 11/26/17.
//

#ifndef MY_PCL_TUTORIAL_GROUND_REMOVAL_H
#define MY_PCL_TUTORIAL_GROUND_REMOVAL_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//#include "gaus_blur.h"

using namespace std;
using namespace pcl;

const int numChannel = 80;
const int numBin = 120;
//const int numMedianKernel = 1;
extern float rMin;
extern float rMax;
//const float tHmin = -2.15;
extern float tHmin;
extern float tHmax;
//const float tHDiff = 0.3;
// since estimated ground plane = -1.73 by sensor height,
// tMin = -2.0
extern float tHDiff;
extern float hSeonsor;

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
// void groundRemove(PointCloud<pcl::PointXYZ> cloud, 
//                   PointCloud<pcl::PointXYZ>::Ptr elevatedCloud); 

#endif //TEST1_GROUND_REMOVAL_H
