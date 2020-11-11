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
float rMin = 3.4;  // 见论文图
float rMax = 120;
//const float tHmin = -2.15;
 float tHmin = -2.0;  // 高度
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


// 过滤车周围半径距离（（ rMin = 3.4; rMax = 120;））
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

// xy坐标，转换为 chP binP坐标
void getCellIndexFromPoints(float x, float y, int& chI, int& binI){
    float distance = sqrt(x * x + y * y);  // 欧氏距离
    //normalize(极坐标)
    float chP = (atan2(y, x) + M_PI) / (2 * M_PI);   // 范围（0，1）atan2(y, x)是4象限反正切     M_PI==3.14
    float binP = (distance - rMin) / (rMax - rMin); //范围（0，1）  rMax - rMin ~~ (3.4, 120)  见figure4-4
    //index
    chI = floor(chP*numChannel);   // numChannel = 80（角度分为80份）
    binI = floor(binP*numBin);     //    numBin = 120（半径分为120份）
//    cout << "bin ind: "<<binI << " ch ind: "<<chI <<endl;
}

// 创造映射极坐标网格（未赋值）
void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin>, numChannel>& polarData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);  //  xy坐标得到对应栅格坐标 得到CellIndex  ： chI, binI（极坐标：角度和半径）
        // TODO; modify abobe function so that below code would not need
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue; // to prevent segentation fault 去除不合理点
        polarData[chI][binI].updateMinZ(z);  // {if (z < minZ) minZ = z;}  // 更新得到z值点的最小值，最终得到最小值 ？？好几个点都在一个栅格，这些点只更新了updateMinZ ,点的其他信息不存储？
    }
}

// update HDiff with larger value
void computeHDiffAdjacentCell(array<Cell, numBin>& channelData){
    //    std::cout << " channelData.size()   "<< channelData.size() << std::endl;  // 输出120
    for(int i = 0; i < channelData.size(); i++){   // 120
        // edge case
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight(); // 差值
            channelData[i].updateHDiff(hD); // 每个栅格高度差
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

 // 中值滤波处理缺失的地面信息（由于遮挡而常见），顾名思义，缺失单元的高度值将替换为相邻单元的中值。
void applyMedianFilter(array<array<Cell, numBin>, numChannel>& polarData){
    // maybe later: consider edge case
    for(int channel = 1; channel < polarData.size()-1; channel++){
        for(int bin = 1; bin < polarData[0].size()-1; bin++){
            if(!polarData[channel][bin].isThisGround()){  // 判断不是地面
                // target cell is non-ground AND surrounded by ground cells
                if(polarData[channel][bin+1].isThisGround()&&  // 四周是地面
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
                    sort(sur.begin(), sur.end()); // 从小到大排序
                    float m1 = sur[1]; float m2 = sur[2];// 取中间2个值
                    float median = (m1+m2)/2;  // 取中值
                    polarData[channel][bin].updataHeight(median);  //高度取中值  缺失单元的高度值将替换为相邻单元的中值
                    polarData[channel][bin].updateGround();  // 是地面
                }
            }
        }
    }
}

 // smoothen spot with outlier  用离群值平滑点?
void outlierFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size() - 1; channel++) {
        for (int bin = 1; bin < polarData[0].size() - 2; bin++) {
            if(polarData[channel][bin].isThisGround()&&  // 都是Ground
               polarData[channel][bin+1].isThisGround()&&
               polarData[channel][bin-1].isThisGround()&&
               polarData[channel][bin+2].isThisGround())
            {
                float height1 = polarData[channel][bin-1].getHeight();
                float height2 = polarData[channel][bin].getHeight();  // height2本尊
                float height3 = polarData[channel][bin+1].getHeight();
                float height4 = polarData[channel][bin+2].getHeight();
                if(height1 != tHmin && height2 == tHmin && height3 != tHmin){ // float tHmin = -2.0;  // 高度
                    float newH = (height1 + height3)/2; //取两边均值
                    polarData[channel][bin].updataHeight(newH);  // 更新高度值
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

// 主函数  src/groundremove/main.cpp会调用此函数
void groundRemove(PointCloud<pcl::PointXYZ>::Ptr   cloud,  // 初始点云
              PointCloud<pcl::PointXYZ>::Ptr  elevatedCloud,  // 高点
              PointCloud<pcl::PointXYZ>::Ptr  groundCloud){  // 地面点

    PointCloud<pcl::PointXYZ> filteredCloud;

    cout << "进入groundRemove函数中---------------------------------------" << endl;  

    filterCloud(cloud, filteredCloud);  // 判断欧氏距离，过滤去除车周围半径外异常点  （ rMin = 3.4; rMax = 120;）
    array<array<Cell, numBin>, numChannel> polarData;  // 数组定义polarData[numChannel][numBin]  ,Cell是个class
    createAndMapPolarGrid(filteredCloud, polarData);   // 极坐标网格映射

    //  polarData.size()  80   channel
    //  polarData[0].size()  120  bin
    for (int channel = 0; channel < polarData.size(); channel++){   // channel: 80
        for (int bin = 0; bin < polarData[0].size(); bin ++){   //  120
            float zi = polarData[channel][bin].getMinZ();  // 得到最小值
            if(zi > tHmin && zi < tHmax){polarData[channel][bin].updataHeight(zi);}   // 每个Cell栅格都有一个updataHeight  
            else if(zi > tHmax){polarData[channel][bin].updataHeight(hSeonsor);}  // float hSeonsor = 2;  高度     大于-0.4 ，设置为2
            else {polarData[channel][bin].updataHeight(tHmin);} //  float tHmin = -2.0;  // 高度  小于-2，设置为-2
        }
        // could replace gauss with gradient
        //  computeGradientAdjacentCell(polarData[channel]);
        gaussSmoothen(polarData[channel], 1, 3);  // 高斯平滑  来自 src/groundremove/gaus_blur.cpp
    //    std::cout << " finished smoothing at channel "<< channel << std::endl;  // 输出
        computeHDiffAdjacentCell(polarData[channel]);    // 还在大的for循环中

        // 判断是否ground
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            if(polarData[channel][bin].getSmoothed() < tHmax &&  // float tHmin = -2.0;
                    polarData[channel][bin].getHDiff() < tHDiff){   //  tHDiff = 0.4; 地面应该是相对平滑的
                polarData[channel][bin].updateGround();  // void updateGround(){isGround = true; hGround = height;}
            }
            else if(polarData[channel][bin].getHeight() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }
    // implement MedianFilter 中值滤波处理缺失的地面信息（由于遮挡而常见），顾名思义，缺失单元的高度值将替换为相邻单元的中值
    applyMedianFilter(polarData);
    // smoothen spot with outlier  用离群值平滑点?
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
            // std::cout << " hGround "<< hGround << std::endl;  // 输出 大多数都是-2
            // std::cout << " z "<< z << std::endl;  // 
            if (z < (hGround + 0.25)) {  // 判断hGround   getHGround()）最终的判断重点就是getHeight 高度值
                groundCloud->push_back(o);  // 
            } else {
                elevatedCloud->push_back(o);
            }
        } else {
            elevatedCloud->push_back(o); // 高点
        }
    }
      cout << "初始点云 size: "<<cloud->size()  << " 处理后高点 size: "<<elevatedCloud->size() << " 处理后地面点 size: "<<groundCloud->size()<< endl;
}