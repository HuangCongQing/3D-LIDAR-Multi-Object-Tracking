
#include <array>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/OccupancyGrid.h>
#include "component_clustering.h"


using namespace std;
using namespace pcl;

//int numGrid = 2;
float roiM = 50;
int kernelSize = 3;

double g_resolution = 1.0;
int g_cell_width =50;
int g_cell_height=50;
double g_offset_x=0;
double g_offset_y = 25;
double g_offset_z = -2;

 double HEIGHT_LIMIT = 0.1;  // from sensor
 double CAR_LENGTH = 4.5;
 double CAR_WIDTH = 2;
//costmap paramter

// 
void mapCartesianGrid(PointCloud<PointXYZ>::Ptr elevatedCloud,
                             array<array<int, numGrid>, numGrid> & cartesianData){


    array<array<int, numGrid>, numGrid> gridNum{};
    for(int cellX = 0; cellX < numGrid; cellX++){
        for(int cellY = 0; cellY < numGrid; cellY++){
            gridNum[cellX][cellY] = 0;
        }
    }

    for(int i = 0; i < elevatedCloud->size(); i++){
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM/2;
        float yC = y+roiM/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
        int xI = floor(numGrid*xC/roiM);
        int yI = floor(numGrid*yC/roiM);
        gridNum[xI][yI] = gridNum[xI][yI] + 1;
    //    cartesianData[xI][yI] = -1;

        // if(xI == 0)
        // {
        //     if(yI == 0)
        //     {
        //         cartesianData[xI+1][yI] = -1;
        //         cartesianData[xI][yI+1] = -1;
        //         cartesianData[xI+1][yI+1] = -1;
        //     }
        //     else if(yI < numGrid - 1)
        //     {
        //         cartesianData[xI][yI-1] = -1;
        //         cartesianData[xI][yI+1] = -1;
        //         cartesianData[xI+1][yI-1] = -1;
        //         cartesianData[xI+1][yI] = -1;
        //         cartesianData[xI+1][yI+1] = -1;
        //     }
        //     else if(yI == numGrid - 1)
        //     {
        //         cartesianData[xI][yI-1] = -1;
        //         cartesianData[xI+1][yI-1] = -1;
        //         cartesianData[xI+1][yI] = -1;    
        //     }
        // }
        // else if(xI < numGrid - 1)
        // {
        //     if(yI == 0)
        //     {
        //         cartesianData[xI-1][yI] = -1;
        //         cartesianData[xI-1][yI+1] = -1;
        //         cartesianData[xI][yI+1] = -1;
        //         cartesianData[xI+1][yI] = -1;
        //         cartesianData[xI+1][yI+1] = -1;                
        //     }
        //     else if(yI < numGrid - 1)
        //     {
        //         cartesianData[xI-1][yI-1] = -1;
        //         cartesianData[xI-1][yI] = -1;
        //         cartesianData[xI-1][yI+1] = -1;
        //         cartesianData[xI][yI-1] = -1;
        //         cartesianData[xI][yI+1] = -1;
        //         cartesianData[xI+1][yI-1] = -1;
        //         cartesianData[xI+1][yI] = -1;
        //         cartesianData[xI+1][yI+1] = -1;                  
        //     }
        //     else if(yI == numGrid - 1)
        //     {
        //         cartesianData[xI-1][yI-1] = -1;
        //         cartesianData[xI-1][yI] = -1;
        //         cartesianData[xI][yI-1] = -1;
        //         cartesianData[xI+1][yI-1] = -1;
        //         cartesianData[xI+1][yI] = -1;                 
        //     } 
        // }
        // else if(xI == numGrid - 1)
        // {
        //     if(yI == 0)
        //     {
        //         cartesianData[xI-1][yI] = -1;
        //         cartesianData[xI-1][yI+1] = -1;
        //         cartesianData[xI][yI+1] = -1;
        //     }
        //     else if(yI < numGrid - 1)
        //     {
        //         cartesianData[xI-1][yI-1] = -1;
        //         cartesianData[xI-1][yI] = -1;
        //         cartesianData[xI-1][yI+1] = -1;
        //         cartesianData[xI][yI-1] = -1;
        //         cartesianData[xI][yI+1] = -1;
        //     }
        //     else if(yI == numGrid - 1)
        //     {
        //         cartesianData[xI-1][yI-1] = -1;
        //         cartesianData[xI-1][yI] = -1;
        //         cartesianData[xI][yI-1] = -1;    
        //     }            
        // }
//        int a = 0;
    }

    for(int xI = 0; xI < numGrid; xI++){
        for(int yI = 0; yI < numGrid; yI++){
            if(gridNum[xI][yI] > 1){
                cartesianData[xI][yI] = -1;

                if(xI == 0)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI+1][yI] = -1;
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

// object_tracking/src/cluster/main.cpp会引用此函数
void componentClustering(PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                         array<array<int, numGrid>, numGrid> & cartesianData,
                         int & numCluster){
    // map 120m radius data(polar grid data) into 100x100 cartesian grid,
    // parameter might need to be modified
    // in this case 30mx30m with 100x100x grid
    mapCartesianGrid(elevatedCloud, cartesianData); // 第一步
    findComponent(cartesianData, numCluster);  // 第二步
}

// void makeClusteredCloud(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
//                         array<array<int, numGrid>, numGrid> cartesianData,
//                         PointCloud<pcl::PointXYZRGB>::Ptr& clusterCloud){
    
//     array<array<int, numGrid>, numGrid> is_obs{0};

//     for(int i = 0; i < elevatedCloud->size(); i++){
//         float x = elevatedCloud->points[i].x;
//         float y = elevatedCloud->points[i].y;
//         float z = elevatedCloud->points[i].z;
//         float xC = x+roiM/2;
//         float yC = y+roiM/2;
//         // exclude outside roi points
//         if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
//         int xI = floor(numGrid*xC/roiM);
//         int yI = floor(numGrid*yC/roiM);

//         int clusterNum;

//         if(is_obs[xI][yI] == 1)continue;
//         else
//         {
//             clusterNum = cartesianData[xI][yI];
//             is_obs[xI][yI] == 1;
//         }

//         if(clusterNum != 0){
//             PointXYZRGB o;
//             o.x = floor(x) + grid_size/2;
//             o.y = floor(y) + grid_size/2;
//             o.z = -1;
//             o.r = (500*clusterNum)%255;
//             o.g = (100*clusterNum)%255;
//             o.b = (150*clusterNum)%255;
//             clusterCloud->push_back(o);
//         }
//     }
// }

// object_tracking/src/cluster/main.cpp会引用此函数
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
        int xI = floor(numGrid*xC/roiM);
        int yI = floor(numGrid*yC/roiM);

        int clusterNum = cartesianData[xI][yI];
        if(clusterNum != 0){
            PointXYZ o;
            o.x = grid_size*xI - roiM/2 + grid_size/2;
            o.y = grid_size*yI - roiM/2 + grid_size/2;
            o.z = -1;
            // o.r = (500*clusterNum)%255;
            // o.g = (100*clusterNum)%255;
            // o.b = (150*clusterNum)%255;
            clusterCloud->push_back(o);
        }
    }
}

void setObsMsg(PointCloud<pcl::PointXYZ>::Ptr& elevatedCloud,
                        array<array<int, numGrid>, numGrid>  cartesianData,
                        object_tracking::ObstacleList &clu_obs)
{

//    array<array<int, numGrid>, numGrid> is_obs{0};
    
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

//        if(is_obs[xI][yI] == 1)continue;

        int clusterNum = cartesianData[xI][yI];
//        is_obs[xI][yI] == 1;

        if(clusterNum != 0){
            object_tracking::Obstacle obs_list;
            obs_list.x = grid_size*xI - roiM/2 + grid_size/2;
            obs_list.y = grid_size*yI - roiM/2 + grid_size/2;
            obs_list.z = -1;
            obs_list.cluster = clusterNum;

            clu_obs.header.frame_id = elevatedCloud->header.frame_id;
            clu_obs.cellLength = grid_size;
            clu_obs.cellWidth = grid_size;            
            clu_obs.obstacles.push_back(obs_list);
            cartesianData[xI][yI] = 0;
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

// object_tracking/src/cluster/main.cpp会引用此函数
void setOccupancyGrid(nav_msgs::OccupancyGrid *og)
{
  og->info.resolution = g_resolution;
  og->info.width = g_cell_width;
  og->info.height = g_cell_height;
  og->info.origin.position.x = (-1) * (g_cell_width / 2.0) * g_resolution + g_offset_x;
  og->info.origin.position.y = (-1) * (g_cell_height / 2.0) * g_resolution + g_offset_y;
  og->info.origin.position.z = g_offset_z;
  og->info.origin.orientation.x = 0.0;
  og->info.origin.orientation.y = 0.0;
  og->info.origin.orientation.z = 0.0;
  og->info.origin.orientation.w = 1.0;
}

// object_tracking/src/cluster/main.cpp会引用此函数
std::vector<int> createCostMap(const pcl::PointCloud<pcl::PointXYZ> &scan)
{
  std::vector<int> cost_map(g_cell_width * g_cell_height, 0);
  double map_center_x = (g_cell_width / 2.0) * g_resolution - g_offset_x;
  double map_center_y = (g_cell_height / 2.0) * g_resolution - g_offset_y;

  // scan points are in sensor frame
  for (const auto &p : scan.points)
  {
    if (p.z > HEIGHT_LIMIT)
      continue;
    if (std::fabs(p.x) < CAR_LENGTH && std::fabs(p.y) < CAR_WIDTH)
      continue;

    // Calculate grid index
    int grid_y = (p.x + map_center_x) / g_resolution;
    int grid_x = (p.y + map_center_y) / g_resolution;
    if (grid_y < 0 || grid_y >= g_cell_width || grid_x < 0 || grid_x >= g_cell_height)
      continue;

    int index = g_cell_width * grid_x + grid_y;
    cost_map[index] += 15;

    // Max cost value is 100
    if (cost_map[index] > 100)
      cost_map[index] = 100;
  }

  return cost_map;
}

