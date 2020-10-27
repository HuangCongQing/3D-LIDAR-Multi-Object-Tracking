
#ifndef MY_PCL_TUTORIAL_COMPONENT_CLUSTERING_H
#define MY_PCL_TUTORIAL_COMPONENT_CLUSTERING_H

#include <array>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/OccupancyGrid.h>
#include <object_tracking/Obstacle.h>
#include <object_tracking/ObstacleList.h>
using namespace std;
using namespace pcl;

const int numGrid = 250;

const float grid_size = 0.2;

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

void setOccupancyGrid(nav_msgs::OccupancyGrid *og);

std::vector<int> createCostMap(const pcl::PointCloud<pcl::PointXYZ> &scan);

void setObsMsg(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        object_tracking::ObstacleList &clu_obs);
//void makeClusterVector(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
//                       array<array<int, numGrid>, numGrid> cartesianData,
//                       vector<PointCloud<pcl::PointXYZ>>& clusteredObjects);

#endif //TEST1_COMPONENT_CLUSTERING_H
