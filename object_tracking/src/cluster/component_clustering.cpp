#include <array>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/OccupancyGrid.h>
#include "component_clustering.h"


using namespace std;
using namespace pcl;

//int numGrid = 2;
float roiM = 50;   // 限制周围距离25m
int kernelSize = 3;

// 下面什么参数？
double g_resolution = 1.0;
int g_cell_width =50;  // 宽 
int g_cell_height=50;
double g_offset_x=0;
double g_offset_y = 25;
double g_offset_z = -2;

 double HEIGHT_LIMIT = 0.1;  // from sensor
 double CAR_LENGTH = 4.5;  // 车的宽高
 double CAR_WIDTH = 2;
//costmap paramter

// 初始化网格Grid状态
void mapCartesianGrid(PointCloud<PointXYZ>::Ptr elevatedCloud,
                             array<array<int, numGrid>, numGrid> & cartesianData){


    array<array<int, numGrid>, numGrid> gridNum{};  //  gridNum 此数组专门统计落在每个grid的点云数， gridNums[250][250]是指：  elevatedCloud点的XY平面离散为m×n单元的网格（见paper图 Figure4-5）
    for(int cellX = 0; cellX < numGrid; cellX++){    // cellX
        for(int cellY = 0; cellY < numGrid; cellY++){   // cellY
            gridNum[cellX][cellY] = 0; // 全部填充初始化为0
        }
    }
    
    // elevatedCloud 映射到笛卡尔坐标系 //并 统计落在这个grid的有多少个点！！！
    for(int i = 0; i < elevatedCloud->size(); i++){  // 遍历高点数
        float x = elevatedCloud->points[i].x;   // x(-15, -5),y(-50, 50)
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM/2;   // float roiM = 50;(0~50)
        float yC = y+roiM/2; // (0~50)
        // exclude outside roi points  排除外部roi points  x,y属于(-25, 25)下面才继续执行
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue; // continue后，下面不执行。gridNum[xI][yI] 值不变 限制范围（0，50）
        int xI = floor(numGrid*xC/roiM);   //  xI .yI    const int numGrid = 250;    floor(x)返回的是小于或等于x的最大整数
        int yI = floor(numGrid*yC/roiM);   // 50x50 映射到→250x250
        gridNum[xI][yI] = gridNum[xI][yI] + 1;  // 统计落在这个grid的有多少个点！！！
        //   cout << "gridNum[xI][yI]: " << gridNum[xI][yI] << "-------------统计落在这个grid的有多少个点！！！------------" << endl;  // gridNum[xI][yI]
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
// 将x，y位置的单个单元格选作中心单元格，并且clusterID计数器加1。
// 然后所有相邻的相邻像元（即x-1，y  + 1，x，y +1，x +1，y +1 x -1，y，x +1，y，x -1，y -1，x，检查y − 1，x + 1，y +  1）的占用状态，并用当前集群ID标记。
// 对m×n网格中的每个x，y重复此过程，直到为所有非空群集分配了ID。
    for(int xI = 0; xI < numGrid; xI++){  //   const int numGrid = 250; 
        for(int yI = 0; yI < numGrid; yI++){
            if(gridNum[xI][yI] > 1){  // 一个点的直接舍弃？
                cartesianData[xI][yI] = -1;   // 网格分配有2种初始状态，分别为空（0），已占用（-1）和已分配。随后，将x，y位置的单个单元格选作中心单元格，并且clusterID计数器加1
                //下面为设置当前点的周围点数值为-1
                if(xI == 0)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI+1][yI] = -1;  // 角相邻的3个相邻像元
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI][yI-1] = -1;  // 边有5个相邻点
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI == numGrid - 1)  // 角相邻的3个相邻像元
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
                    else if(yI < numGrid - 1)  // 一般情况四周有8个相邻点
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
    // for(int xI = 0; xI < numGrid; xI++){  //   const int numGrid = 250; 
    //     for(int yI = 0; yI < numGrid; yI++){
    //          cout << " cartesianData[xI][yI] is "<< xI<<"   " << yI  <<"   " <<  cartesianData[xI][yI]<< "========设置为-1========" <<endl;  
    //     }}
}

// findComponent会引用search函数   聚类    图搜索
void search(array<array<int, numGrid>, numGrid> & cartesianData, int clusterId, int cellX, int cellY){   //  cellX(0-249), cellY(0-249)
    cartesianData[cellX][cellY] = clusterId; // 赋值，将周边邻居值赋值同样的clusterId
    int mean = kernelSize/2;   // kernelSize = 3;  mean  = 1 
    for (int kX = 0; kX < kernelSize; kX++){   // kernelSize = 3;循环3次
        int kXI = kX-mean; //    0， -1 ， 1 //   cout << "kXI  is "<<kXI<<endl; 
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid) continue;   // numGrid = 250;
        for( int kY = 0; kY < kernelSize;kY++){
            int kYI = kY-mean; // 减去均值？？？？？
            if((cellY + kYI) < 0 || (cellY + kYI) >= numGrid) continue;

            if(cartesianData[cellX + kXI][cellY + kYI] == -1){
                search(cartesianData, clusterId, cellX +kXI, cellY + kYI);  // 循环搜索
            }

        }
    }
}

//  对m×n网格中的每个x，y重复此过程，直到为所有非空cluster分配了ID。
void findComponent(array<array<int, numGrid>, numGrid> & cartesianData, int &clusterId){
    for(int cellX = 0; cellX < numGrid; cellX++){  // 循环每个点   numGrid = 250;
        for(int cellY = 0; cellY < numGrid; cellY++){
            //   cout << "cartesianData[cellX][cellY]  is "<< cellX<<"   " << cellY  <<"   "  <<  cartesianData[cellX][cellY]  <<endl;   // 大多数是0??为什么
            if(cartesianData[cellX][cellY] == -1){   // 随后，将x，y位置的单个单元格选作中心单元格，并且clusterID计数器加1   (网格分配有2种初始状态，分别为空（0），已占用（-1）)
                clusterId ++;    // 对m×n网格中的每个x，y重复此过程，直到为所有非空cluster分配了ID。  // cout << "clusterId is "<< clusterId <<endl;  //  特别少？？？
                search(cartesianData, clusterId, cellX, cellY);  // 对每一个点进行搜索  cellX(0-249), cellY(0-249)
            }
        }
    }
}

// object_tracking/src/cluster/main.cpp会引用此函数
void componentClustering(PointCloud<pcl::PointXYZ>::Ptr elevatedCloud,
                         array<array<int, numGrid>, numGrid> & cartesianData,
                         int & numCluster){
    // map 120m radius data(polar grid data) into 100x100 cartesian grid, parameter might need to be modified
    // 将120m半径数据（极坐标数据）映射到100x100笛卡尔网格中，可能需要修改参数
    // in this case 30mx30m with 100x100x grid  在这种情况下，30mx30m带有100x100x网格
    mapCartesianGrid(elevatedCloud, cartesianData); // 第一步设置网格Grid状态   网格数组：cartesianData
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
        int xI = floor(numGrid*xC/roiM);  // (0~249)
        int yI = floor(numGrid*yC/roiM);  // (0~249)

        // cout << "xI is "<< xI <<endl;
        // cout << "yI is "<< yI <<endl;
        // cout << "cartesianData is "<< cartesianData[xI][yI]<<endl;  //  各种数值
        int clusterNum = cartesianData[xI][yI]; //  数值
        if(clusterNum != 0){
            PointXYZ o;
            o.x = grid_size*xI - roiM/2 + grid_size/2;  // 网格大小？？const float grid_size = 0.2;
            o.y = grid_size*yI - roiM/2 + grid_size/2;
            o.z = -1;
            // o.r = (500*clusterNum)%255;
            // o.g = (100*clusterNum)%255;
            // o.b = (150*clusterNum)%255;
            clusterCloud->push_back(o); // 
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

// object_tracking/src/cluster/main.cpp会引用此函数  代价地图
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

