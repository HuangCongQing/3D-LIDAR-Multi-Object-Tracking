//
// Created by kosuke on 11/26/17.
//

// #include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <math.h>
//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//#include <array> // std::array
//#include"Eigen/Core"

#include "ground_removal.h"
#include "gaus_blur.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

//int numChannel = 80;
//int numBin = 120;
//const int numMedianKernel = 1;
float rMin = 3.4;
float rMax = 120;
//const float tHmin = -2.15;
 float tHmin = -2.0;
//float tHmin = -1.9;
 float tHmax = -0.4;
//float tHmax = -1.0;

float tHDiff = 0.4;
float hSeonsor = 2;//1.73

Cell::Cell(){
    minZ = 1000;
    isGround = false;
}

void Cell::updateMinZ(float z) {
    if (z < minZ) minZ = z;
}



void filterCloud(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ> & filteredCloud){
    for (int i = 0; i < cloud->size(); i++) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        float distance = sqrt(x * x + y * y);
        if(distance <= rMin || distance >= rMax) { // 判断欧氏距离，过滤去除异常点  （ rMin = 3.4; rMax = 120;）
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

void getCellIndexFromPoints(float x, float y, int& chI, int& binI){
    float distance = sqrt(x * x + y * y);
    //normalize
    float chP = (atan2(y, x) + M_PI) / (2 * M_PI);
    float binP = (distance - rMin) / (rMax - rMin);
    //index
    chI = floor(chP*numChannel);   // ???
    binI = floor(binP*numBin);     // ？？没懂
   cout << "bin ind: "<<binI << " ch ind: "<<chI <<endl;
}

void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin>, numChannel>& polarData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);  // 得到CellIndex  ： chI, binI
        // TODO; modify abobe function so that below code would not need
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue; // to prevent segentation fault
        polarData[chI][binI].updateMinZ(z);
    }
}

// update HDiff with larger value
void computeHDiffAdjacentCell(array<Cell, numBin>& channelData){
    for(int i = 0; i < channelData.size(); i++){
        // edge case
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight();
            channelData[i].updateHDiff(hD);
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

//        cout <<channelData[i].getHeight() <<" " <<channelData[i].getHDiff() << endl;
    }
}

void applyMedianFilter(array<array<Cell, numBin>, numChannel>& polarData){
    // maybe later: consider edge case
    for(int channel = 1; channel < polarData.size()-1; channel++){
        for(int bin = 1; bin < polarData[0].size()-1; bin++){
            if(!polarData[channel][bin].isThisGround()){
                // target cell is non-ground AND surrounded by ground cells
                if(polarData[channel][bin+1].isThisGround()&&
                   polarData[channel][bin-1].isThisGround()&&
                   polarData[channel+1][bin].isThisGround()&&
                   polarData[channel-1][bin].isThisGround()){
                    vector<float> sur{polarData[channel][bin+1].getHeight(),
                                      polarData[channel][bin-1].getHeight(),
                                      polarData[channel+1][bin].getHeight(),
                                      polarData[channel-1][bin].getHeight()};
                    sort(sur.begin(), sur.end());
                    float m1 = sur[1]; float m2 = sur[2];
                    float median = (m1+m2)/2;
                    polarData[channel][bin].updataHeight(median);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}

void outlierFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size() - 1; channel++) {
        for (int bin = 1; bin < polarData[0].size() - 2; bin++) {
            if(polarData[channel][bin].isThisGround()&&
               polarData[channel][bin+1].isThisGround()&&
               polarData[channel][bin-1].isThisGround()&&
               polarData[channel][bin+2].isThisGround()){
                float height1 = polarData[channel][bin-1].getHeight();
                float height2 = polarData[channel][bin].getHeight();
                float height3 = polarData[channel][bin+1].getHeight();
                float height4 = polarData[channel][bin+2].getHeight();
                if(height1 != tHmin && height2 == tHmin && height3 != tHmin){
                    float newH = (height1 + height3)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
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

// 主函数  src/groundremove/main.cpp会调用此函数
void groundRemove(PointCloud<pcl::PointXYZ>::Ptr   cloud,  // 初始点云
              PointCloud<pcl::PointXYZ>::Ptr  elevatedCloud,  // 高点
              PointCloud<pcl::PointXYZ>::Ptr  groundCloud){  // 地面点

    PointCloud<pcl::PointXYZ> filteredCloud;

    filterCloud(cloud, filteredCloud);  // 判断欧氏距离，过滤去除异常点  （ rMin = 3.4; rMax = 120;）
    array<array<Cell, numBin>, numChannel> polarData;
    createAndMapPolarGrid(filteredCloud, polarData);

    cout << "初始点云 size: "<<cloud->size() << endl;
    cout << "高点 size: "<<elevatedCloud->size() << endl;
    cout << "地面点 size: "<<groundCloud->size() << endl;
    for (int channel = 0; channel < polarData.size(); channel++){
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            float zi = polarData[channel][bin].getMinZ();
            if(zi > tHmin && zi < tHmax){polarData[channel][bin].updataHeight(zi);}
            else if(zi > tHmax){polarData[channel][bin].updataHeight(hSeonsor);}
            else {polarData[channel][bin].updataHeight(tHmin);}
        }
        //could replace gauss with gradient
//        computeGradientAdjacentCell(polarData[channel]);
        gaussSmoothen(polarData[channel], 1, 3);
//        std::cout << " finished smoothing at channel "<< channel << std::endl;
        computeHDiffAdjacentCell(polarData[channel]);

        for (int bin = 0; bin < polarData[0].size(); bin ++){
            if(polarData[channel][bin].getSmoothed() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
            else if(polarData[channel][bin].getHeight() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }
    // implement MedianFilter
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
            if (z < (hGround + 0.25)) {
                groundCloud->push_back(o);
            } else {
                elevatedCloud->push_back(o);
            }
        } else {
            elevatedCloud->push_back(o);
        }
    }
}