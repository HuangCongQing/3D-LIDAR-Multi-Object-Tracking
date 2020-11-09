//
// Created by kosuke on 11/28/17.
//
#include <array>
#include <pcl/io/pcd_io.h>

#include "component_clustering.h"

using namespace std;
using namespace pcl;

//int numGrid = 2;
float roiM = 30;
int kernelSize = 3;


void mapCartesianGrid(PointCloud<PointXYZ>::Ptr elevatedCloud,
                             array<array<int, numGrid>, numGrid> & cartesianData){

    for(int i = 0; i < elevatedCloud->size(); i++){
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM/2;
        float yC = y+roiM/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
        int xI = floor(numGrid*xC/roiM);
        int yI = floor(numGrid*yC/roiM);
        cartesianData[xI][yI] = -1;
//        int a = 0;
    }
}

void search(array<array<int, numGrid>, numGrid> & cartesianData, int clusterId, int cellX, int cellY){
    cartesianData[cellX][cellY] = clusterId;
    int mean = kernelSize/2;
    for (int kX = 0; kX < kernelSize; kX++){
        int kXI = kX-mean;
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid) continue;
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
    for(int cellX = 0; cellX < numGrid; cellX++){
        for(int cellY = 0; cellY < numGrid; cellY++){
            if(cartesianData[cellX][cellY] == -1){
                clusterId ++;
                search(cartesianData, clusterId, cellX, cellY);
            }
        }
    }
}

void componentClustering(PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                         array<array<int, numGrid>, numGrid> & cartesianData,
                         int & numCluster){
    // map 120m radius data(polar grid data) into 100x100 cartesian grid,
    // parameter might need to be modified
    // in this case 30mx30m with 100x100x grid
    mapCartesianGrid(elevatedCloud, cartesianData);
    findComponent(cartesianData, numCluster);
}

void makeClusteredCloud(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        PointCloud<pcl::PointXYZRGB>::Ptr& clusterCloud){
    for(int i = 0; i < elevatedCloud->size(); i++){
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float z = elevatedCloud->points[i].z;
        float xC = x+roiM/2;
        float yC = y+roiM/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
        int xI = floor(numGrid*xC/roiM);
        int yI = floor(numGrid*yC/roiM);

        int clusterNum = cartesianData[xI][yI];
        if(clusterNum != 0){
            PointXYZRGB o;
            o.x = x;
            o.y = y;
            o.z = z;
            o.r = (500*clusterNum)%255;
            o.g = (100*clusterNum)%255;
            o.b = (150*clusterNum)%255;
            clusterCloud->push_back(o);
        }
    }
}

//void makeClusterVector(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
//                       array<array<int, numGrid>, numGrid> cartesianData,
//                       vector<PointCloud<pcl::PointXYZ>>& clusteredObjects){
//    for(int i = 0; i < elevatedCloud->size(); i++){
//        float x = elevatedCloud->points[i].x;
//        float y = elevatedCloud->points[i].y;
//        float z = elevatedCloud->points[i].z;
//        float xC = x+roiM/2;
//        float yC = y+roiM/2;
//        // exclude outside roi points
//        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
//        int xI = floor(numGrid*xC/roiM);
//        int yI = floor(numGrid*yC/roiM);
//
//        int clusterNum = cartesianData[xI][yI];
//        if(clusterNum != 0){
//            PointXYZRGB o;
//            o.x = x;
//            o.y = y;
//            o.z = z;
//            o.r = (500*clusterNum)%255;
//            o.g = (100*clusterNum)%255;
//            o.b = (150*clusterNum)%255;
////            clusterCloud->push_back(o);
//        }
//    }
//}