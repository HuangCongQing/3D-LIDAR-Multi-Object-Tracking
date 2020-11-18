//
// Created by kosuke on 11/29/17.
//
#include <array>
#include <random>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include "box_fitting.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>


using namespace std;
using namespace pcl;
using namespace cv;

// float picScale = 30;
float picScale = 900/roiM;   // ???
int ramPoints = 80;
int lSlopeDist = 1.0;
////////////////////////int lSlopeDist = 3.0;
////////////////////////int lnumPoints = 300;
int lnumPoints = 5;

//////////////////////////////float sensorHeight = 1.73;
float sensorHeight = 2;   // 激光雷达距离地面高度
// float tHeightMin = 1.2;
float tHeightMin = 0.8;
float tHeightMax = 2.6;
// float tWidthMin = 0.5;
// float tWidthMin = 0.4;
float tWidthMin = 0.2;//0.25
float tWidthMax = 3.5;
float tLenMin = 0.2;//0.5
float tLenMax = 14.0;
float tAreaMax = 20.0;
//float tRatioMin = 1.3;
//float tRatioMax = 5.0;

float tRatioMin = 1;
float tRatioMax = 8.0;

float minLenRatio = 3.0;
float tPtPerM3 = 8;

void getClusteredPoints(PointCloud<PointXYZ>::Ptr elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        vector<PointCloud<PointXYZ>>&  clusteredPoints) {
    for (int i = 0; i < elevatedCloud->size(); i++) {
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float z = elevatedCloud->points[i].z;
        float xC = x + roiM / 2;
        float yC = y + roiM / 2;
        // exclude outside roi points
        if (xC < 0 || xC >= roiM || yC < 0 || yC >= roiM) continue;
        int xI = floor(numGrid * xC / roiM);
        int yI = floor(numGrid * yC / roiM);

        int clusterNum = cartesianData[xI][yI]; //1 ~ numCluster   初始聚类ID数量numCluster
        // cout << "clusterNum is " << clusterNum << endl;  // 连续几个相同的值（聚类在一块的）
        int vectorInd = clusterNum - 1; //0 ~ (numCluster -1 )
        if (clusterNum != 0) {
            PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            clusteredPoints[vectorInd].push_back(o);  // 填充clusteredPoints   PointXYZ类型数组
            // cout << "clusteredPoints is " << clusteredPoints << endl;  //  报错？
        }
    }
}

// 
void getPointsInPcFrame(Point2f rectPoints[], vector<Point2f>& pcPoints, int offsetX, int offsetY){
    // loop 4 rect points
    for (int pointI = 0; pointI < 4; pointI++){
        float picX = rectPoints[pointI].x;
        float picY = rectPoints[pointI].y;
        // reverse offset
        float rOffsetX = picX - offsetX;
        float rOffsetY = picY - offsetY;
        // reverse from image coordinate to eucledian coordinate
        float rX = rOffsetX;
        float rY = picScale*roiM - rOffsetY;  //  float picScale = 900/roiM;
        // reverse to 30mx30m scale
        float rmX = rX/picScale;  //  float picScale = 900/roiM;
        float rmY = rY/picScale;
        // reverse from (0 < x,y < 30) to (-15 < x,y < 15)
        float pcX = rmX - roiM/2;
        float pcY = rmY - roiM/2;
        Point2f point(pcX, pcY);
        pcPoints[pointI] = point;
    }
}

bool ruleBasedFilter(vector<Point2f> pcPoints, float maxZ, int numPoints){
    bool isPromising = false; // 是否舍弃此聚类
    //minnimam points thresh
    if(numPoints < 30) return isPromising;  // 小于30个点，返回false 舍弃此聚类  cout << "numPoints is "<< numPoints <<endl;  // 差不多(3~, 800~)
    // length is longest side of the rectangle while width is the shorter side. 长度是矩形的最长边，而宽度是较短的边。
    float width, length, height, area, ratio, mass;

    float x1 = pcPoints[0].x;
    float y1 = pcPoints[0].y;
    float x2 = pcPoints[1].x;
    float y2 = pcPoints[1].y;
    float x3 = pcPoints[2].x;
    float y3 = pcPoints[2].y;

    float dist1 = sqrt((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2));
    float dist2 = sqrt((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
    if(dist1 > dist2){  // 判断长边和宽边
        length = dist1;
        width = dist2;
    }
    else{
        length = dist2;
        width = dist1;
    }
    // assuming ground = sensor height  假设地面=传感器高度
    //  cout << "maxZ is "<< maxZ <<endl;  // 差不多这个范围(0.553392,0.999263)
    height = maxZ + sensorHeight; //  float sensorHeight = 2;  
    // assuming right angle
    area = dist1*dist2;  //平面框面积
    mass = area*height;  //框 体积
    ratio = length/width;  // 长宽比

    // cout<< "height is " <<height<<endl;  //  2.3821
    // cout<< "width is " <<width<<endl; // 2.17553
    // cout<< "length is " <<length<<endl; //4.92502
    // cout<< "area is " <<area<<endl; //10.7145
    // cout<< "mass is " <<mass<<endl; //25.523
    // cout<< "ratio is " <<ratio<<endl; // 2.26383

    //start rule based filtering   // 开始 Rule-based Filter
    if(height > tHeightMin && height < tHeightMax){ //(0.8  2.6) tHeightMin = 0.8;  tHeightMax = 2.6;
        if(width > tWidthMin && width < tWidthMax){ // (0.2  3.5)
            if(length > tLenMin && length < tLenMax){ // (0.2,  14)
                if(area < tAreaMax){  // tAreaMax = 20.0;
                    if(numPoints > mass*tPtPerM3){   // tPtPerM3 = 8;
                        if(length > minLenRatio){ // minLenRatio = 3.0;
                            if(ratio > tRatioMin && ratio < tRatioMax){ //长宽比 (1.0,  8.0)
                                isPromising = true;
                                return isPromising;
                            }
                        }
                        else{
                            isPromising = true;
                            return isPromising;
                        }
                    }
                }
            }
        }
    }
    else return isPromising;
}

// getBoundingBox引用  制作实体框
visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZ> cloud_cluster) 
{ 
  Eigen::Vector4f centroid; 
  Eigen::Vector4f min; 
  Eigen::Vector4f max; 
  
  pcl::compute3DCentroid (cloud_cluster, centroid);  // 坐标 centroid- 重心
  pcl::getMinMax3D (cloud_cluster, min, max);  // 最大最小值?
  
  uint32_t shape = visualization_msgs::Marker::CUBE;  //定义立方体 ！！！！
  visualization_msgs::Marker marker;    // 定义marker
  marker.header.frame_id = "/velo_link";  //frame_id
  marker.header.stamp = ros::Time::now(); 
  
  marker.ns = "cube";  // 简单命名
  marker.id = 0; 
  marker.type = shape;  // 立方体
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = centroid[0];  // 
  marker.pose.position.y = centroid[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0; 
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0; 
  
  marker.scale.x = (max[0]-min[0]);  // 对应x,y,x坐标
  marker.scale.y = (max[1]-min[1]); 
  marker.scale.z = (max[2]-min[2]); 
  
  if (marker.scale.x ==0)   // 判断
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
 marker.color.r = 1.0f;  // 红色实体
//   marker.color.g = 1.0f;  // 绿色
//  marker.color.b = b; 
  marker.color.a = 1.0; 

  marker.lifetime = ros::Duration(1); 
//   marker.lifetime = ros::Duration(0.5); 
  return marker;   // 返回marker
}

//  将最小面积矩形（MAR）[128]应用于每个聚类对象，从而生成一个2D框，当与保留在聚类过程中的高度信息结合后，它便成为3D边界框 
void getBoundingBox(vector<PointCloud<PointXYZ>>  clusteredPoints,
                    vector<PointCloud<PointXYZ>>& bbPoints,visualization_msgs::MarkerArray& ma){
    // cout << "the number of cluster is "<< clusteredPoints.size() <<endl;   // 初始聚类ID数量numCluster is 
    
    for (int iCluster = 0; iCluster < clusteredPoints.size(); iCluster++){  //  循环的次数就是 初始聚类ID数量numCluster 
        Mat m (picScale*roiM, picScale*roiM, CV_8UC1, Scalar(0));
        float initPX = clusteredPoints[iCluster][0].x + roiM/2; // (0 ~ 50) 第0个点的x，y坐标
        float initPY = clusteredPoints[iCluster][0].y + roiM/2; // (0 ~ 50)
        int initX = floor(initPX*picScale);  //   picScale = 900/roiM, 放大倍数900
        int initY = floor(initPY*picScale); // 
        int initPicX = initX;
        int initPicY = picScale*roiM - initY;  // Y值取余 ： Y  =  900-Y
        int offsetInitX = roiM*picScale/2 - initPicX;  // 
        int offsetInitY = roiM*picScale/2 - initPicY;

        int numPoints = clusteredPoints[iCluster].size();  //  每一聚类里面有多少点 numPoints
        vector<Point> pointVec(numPoints);  // 定义
        vector<Point2f> pcPoints(4);   // ???
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -99;
        float maxdistance=-999;
        // for center of gravity重力
        float sumX = 0; float sumY = 0;

        //  cout << "the number of cluster i is "<< numPoints <<endl;

        // 循环每一聚类里面的点
        for (int iPoint = 0; iPoint < clusteredPoints[iCluster].size(); iPoint++){
            float pX = clusteredPoints[iCluster][iPoint].x;   // 聚类中每个点的x，y，z
            float pY = clusteredPoints[iCluster][iPoint].y;
            float pZ = clusteredPoints[iCluster][iPoint].z;
            // cast (-15 < x,y < 15) into (0 < x,y < 30)
            float roiX = pX + roiM/2;
            float roiY = pY + roiM/2;
            // cast 30mx30m into 900x900 scale  投射放大
            int x = floor(roiX*picScale);
            int y = floor(roiY*picScale);
            // cast into image coordinate
            int picX = x;
            int picY = picScale*roiM - y;   //  Y轴换个反方向
            // offset so that the object would be locate at the center   offset偏移量以便将object定位在中心
            int offsetX = picX + offsetInitX;
            int offsetY = picY + offsetInitY;
            
/*            std::cout << "----------------------" << std::endl;
            std::cout << offsetX << std::endl; // 举例 444
            std::cout << offsetY << std::endl; // 举例 452
            std::cout << "---------------------" << std::endl; 
*/

          //  m.at<uchar>(offsetY, offsetX) = 255;
            pointVec[iPoint] = Point(offsetX, offsetY); // 偏移量
            // calculate min and max slope 斜率
            float m = pY/pX;  // 斜率
            ////////////std::cout << "m:" << m << std::endl;

            if(m < minM) {  // float minM = 999; float maxM = -999  根据斜率判断
                minM = m;
                minMx = pX;
                minMy = pY;
                /////////std::cout << "mmmmmm" << m << std::endl;

            }
            if(m > maxM) { // float minM = 999; float maxM = -999
                maxM = m;
                maxMx = pX;
                maxMy = pY;
                /////////std::cout << "MMMMM" << std::endl;
            }

            float xydis = sqrt(pX*pX+pY*pY);  // 

            if(xydis > maxdistance)maxdistance = xydis;    // 最大距离


            //get maxZ
            if(pZ > maxZ) maxZ = pZ;  // 最大高度

            sumX += offsetX; // ???
            sumY += offsetY; 

        } // 小循环结束
        // L shape fitting parameters ===  将最小面积矩形（MAR）[128]应用于每个聚类对象，从而生成一个2D框，
        // 某些稀疏点（带红色圆圈）被认为是异常值，并且MAR过程导致不正确的框，使用L形拟合可解决此问题  可参考:https://www.yuque.com/huangzhongqing/hre6tf/pcohs1#FONbX
        float xDist = maxMx - minMx; // 选择两个最远的离群点x1和x2
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist*xDist + yDist*yDist);   // 选择两个最远的离群点x1和x2，它们位于面向LIDAR传感器的对象的相对侧。然后在两点之间绘制一条线Ld
        /////////////////////std::cout << "boxFitting  slopeDist" << slopeDist << std::endl;
        float slope = (maxMy - minMy)/(maxMx - minMx);   //最大斜率

        // random variable
        mt19937_64 mt(0); // ???
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object 开始为汽车之类的物体 l shape fitting
        // lSlopeDist = 10000, lnumPoints = 30000;
        if(slopeDist > lSlopeDist && numPoints > lnumPoints && (maxMy > 8 || maxMy < -5)){ // int lSlopeDist = 1.0;  int lnumPoints = 5;判断框的大小以及框内的点数
//        if(1){
            float maxDist = 0;
            float maxDx, maxDy;

            // 80 random points, get max distance
            for(int i = 0; i < ramPoints; i++){   // ramPoints = 80;
                int pInd = randPoints(mt); // 随机点
                assert(pInd >= 0 && pInd < clusteredPoints[iCluster].size()); // 如果其值为假（即为0），那么它先向stderr打印一条出错信息，
                float xI = clusteredPoints[iCluster][pInd].x;
                float yI = clusteredPoints[iCluster][pInd].y;

                // from equation of distance between line and point /点到直线的距离方程
                float dist = abs(slope*xI-1*yI+maxMy-slope*maxMx)/sqrt(slope*slope + 1);  //点到直线的距离
                if(dist > maxDist) {   // 寻找最远的距离  maxDist
                    maxDist = dist; // 赋值给最远的距离  maxDist
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            // for center of gravity  重心
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding  向量加法   闭合连接X1、X2和X3得到L形折线
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            float lastX = maxDx + maxMvecX + minMvecX; // X4
            float lastY = maxDy + maxMvecY + minMvecY;

            pcPoints[0] = Point2f(minMx, minMy);  // ??pcPoints
            pcPoints[1] = Point2f(maxDx, maxDy);
            pcPoints[2] = Point2f(maxMx, maxMy);
            pcPoints[3] = Point2f(lastX, lastY);  // 得到第四个点

            // std::cout << " pcPoints[0]  " << pcPoints[0].x << " " <<  pcPoints[0].y << std::endl;
            // std::cout << " pcPoints[1]  " << pcPoints[1].x << " " <<  pcPoints[1].y << std::endl;
            // std::cout << " pcPoints[2]  " << pcPoints[2].x << " " <<  pcPoints[2].y << std::endl;
            // std::cout << " pcPoints[3]  " << pcPoints[3].x << " " <<  pcPoints[3].y << std::endl;

            // std::cout << "boxFitting  maxZ " << maxZ << std::endl;   //  boxFitting  maxZ 0.525725
            //  最后一步:消除大多数无关对象，例如墙壁，灌木丛，建筑物和树木。实现尺寸阈值化以实现此目的。
            bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);    //   比如聚类33，每一类里面有多少点 numPoints(四个点,z最大值, 每个聚类中的点数)
            // std::cout << "boxFitting   isPromising:" << isPromising << std::endl; // 1, 0
            if(!isPromising) continue;  // 没有,就舍弃此聚类,到下一个循环
        }
        else{
            //MAR fitting
            RotatedRect rectInfo = minAreaRect(pointVec); //自带函数
            Point2f rectPoints[4]; rectInfo.points( rectPoints );
            // covert points back to lidar coordinate  隐蔽点返回激光雷达坐标
            getPointsInPcFrame(rectPoints, pcPoints, offsetInitX, offsetInitY);
            // rule based filter
            bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            if(!isPromising) continue;   // 没有,就舍弃此聚类,到下一个循环
        //     // for visualization
        //    for( int j = 0; j < 4; j++ )
        //        line( m, rectPoints[j], rectPoints[(j+1)%4ne], Scalar(255,255,0), 1, 8 );
        //    imshow("Display Image", m);
        //    waitKey(0);
            /////////std::cout << " else  pcPoints[0]  " << pcPoints[0].x << " " <<  pcPoints[0].y << std::endl;
            ////////std::cout << " else  pcPoints[1]  " << pcPoints[1].x << " " <<  pcPoints[1].y << std::endl;
            ///////std::cout << " else  pcPoints[2]  " << pcPoints[2].x << " " <<  pcPoints[2].y << std::endl;
            ///////std::cout << " else  pcPoints[3]  " << pcPoints[3].x << " " <<  pcPoints[3].y << std::endl;

        }

        // make pcl cloud for 3d bounding box  3D边界框
        PointCloud<PointXYZ> oneBbox;
        for(int pclH = 0; pclH < 2; pclH++){ // 上下个4个坐标点
            for(int pclP = 0; pclP < 4; pclP++){
                PointXYZ o;
                o.x = pcPoints[pclP].x;
                o.y = pcPoints[pclP].y;
                if(pclH == 0) o.z = -sensorHeight;  // -2  下面四个点坐标为-2
                else o.z = maxZ;  // 上面四个点z坐标
                oneBbox.push_back(o);   // 一个边界框
            }
        }  //  3D边界框填充循环结束

        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////

        // float xDist1 = oneBbox[0].x-oneBbox[1].x;
        // float yDist1 = oneBbox[0].y-oneBbox[1].y;
        // float slopeDist1 = sqrt(xDist1*xDist1 + yDist1*yDist1);
        // float xDist2 = oneBbox[1].x-oneBbox[2].x;
        // float yDist2 = oneBbox[1].y-oneBbox[2].y;
        // float slopeDist2 = sqrt(xDist2*xDist2 + yDist2*yDist2);
        // if(slopeDist1 < 0.2||slopeDist2 < 0.2) continue;


        ////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////


        bbPoints.push_back(oneBbox); // 实体边界框集合
//        clustered2D[iCluster] = m;

        visualization_msgs::Marker mac = mark_cluster(clusteredPoints[iCluster]);   // 边框参数设置(还在for循环里面哈)   返回marker
        ma.markers.push_back(mac);  // ma在这里 填充  实体框

    }

    
    // std::cout << "boxFitting   bbPoints:" << bbPoints.size() << " 多少个边界框" << std::endl;

}


//  候选框拟合处理
vector<PointCloud<PointXYZ>> boxFitting(PointCloud<PointXYZ>::Ptr elevatedCloud,
                array<array<int, numGrid>, numGrid> cartesianData,
                int numCluster,visualization_msgs::MarkerArray& ma)  // ma数据
{
    vector<PointCloud<PointXYZ>>  clusteredPoints(numCluster);
    getClusteredPoints(elevatedCloud, cartesianData, clusteredPoints);  // 指的是聚类点？具体什么意思呢？
    vector<PointCloud<PointXYZ>>  bbPoints;
    getBoundingBox(clusteredPoints, bbPoints,ma);  // 得到候选框

    return bbPoints;  //返回
//    vector<vector<float>>  bBoxes(numCluster,  vector<float>(6));
//
//    return bBoxes;
}

