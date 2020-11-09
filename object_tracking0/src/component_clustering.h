//
// Created by kosuke on 11/28/17.
//

#ifndef MY_PCL_TUTORIAL_COMPONENT_CLUSTERING_H
#define MY_PCL_TUTORIAL_COMPONENT_CLUSTERING_H

#include <array>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

//const int numGrid = 100;
// (numGrid, roiM) = (200, 30) means 3000(cm)/200, 15cmx15cm grid aprart leads to different cluster
const int numGrid = 200;
extern float roiM;
extern int kernelSize;

void componentClustering(PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                         array<array<int, numGrid>, numGrid> & cartesianData,
                         int & numCluster);

void mapCartesianGrid(PointCloud<PointXYZ>::Ptr elevatedCloud,
                            array<array<int, numGrid>, numGrid> & cartesianData);

void makeClusteredCloud(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        PointCloud<pcl::PointXYZRGB>::Ptr& clusterCloud);

//void makeClusterVector(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
//                       array<array<int, numGrid>, numGrid> cartesianData,
//                       vector<PointCloud<pcl::PointXYZ>>& clusteredObjects);

#endif //TEST1_COMPONENT_CLUSTERING_H
