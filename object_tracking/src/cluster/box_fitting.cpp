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
float picScale = 900/roiM;
int ramPoints = 80;
int lSlopeDist = 1.0;
////////////////////////int lSlopeDist = 3.0;
////////////////////////int lnumPoints = 300;
int lnumPoints = 5;

//////////////////////////////float sensorHeight = 1.73;
float sensorHeight = 2;
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

        int clusterNum = cartesianData[xI][yI]; //1 ~ numCluster
        int vectorInd = clusterNum - 1; //0 ~ (numCluster -1 )
        if (clusterNum != 0) {
            PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            clusteredPoints[vectorInd].push_back(o);  // 填充clusteredPoints
        }
    }
}

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
        float rY = picScale*roiM - rOffsetY;
        // reverse to 30mx30m scale
        float rmX = rX/picScale;
        float rmY = rY/picScale;
        // reverse from (0 < x,y < 30) to (-15 < x,y < 15)
        float pcX = rmX - roiM/2;
        float pcY = rmY - roiM/2;
        Point2f point(pcX, pcY);
        pcPoints[pointI] = point;
    }
}

bool ruleBasedFilter(vector<Point2f> pcPoints, float maxZ, int numPoints){
    bool isPromising = false;
    //minnimam points thresh
    if(numPoints < 30) return isPromising;
    // length is longest side of the rectangle while width is the shorter side.
    float width, length, height, area, ratio, mass;

    float x1 = pcPoints[0].x;
    float y1 = pcPoints[0].y;
    float x2 = pcPoints[1].x;
    float y2 = pcPoints[1].y;
    float x3 = pcPoints[2].x;
    float y3 = pcPoints[2].y;

    float dist1 = sqrt((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2));
    float dist2 = sqrt((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
    if(dist1 > dist2){
        length = dist1;
        width = dist2;
    }
    else{
        length = dist2;
        width = dist1;
    }
    // assuming ground = sensor height
    height = maxZ + sensorHeight;
    // assuming right angle
    area = dist1*dist2;
    mass = area*height;
    ratio = length/width;

    // cout<< "height is " <<height<<endl;
    // cout<< "width is " <<width<<endl;
    // cout<< "length is " <<length<<endl;
    // cout<< "area is " <<area<<endl;
    // cout<< "mass is " <<mass<<endl;
    // cout<< "ratio is " <<ratio<<endl;

    //start rule based filtering
    if(height > tHeightMin && height < tHeightMax){
        if(width > tWidthMin && width < tWidthMax){
            if(length > tLenMin && length < tLenMax){
                if(area < tAreaMax){
                    if(numPoints > mass*tPtPerM3){
                        if(length > minLenRatio){
                            if(ratio > tRatioMin && ratio < tRatioMax){
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

visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZ> cloud_cluster) 
{ 
  Eigen::Vector4f centroid; 
  Eigen::Vector4f min; 
  Eigen::Vector4f max; 
  
  pcl::compute3DCentroid (cloud_cluster, centroid); 
  pcl::getMinMax3D (cloud_cluster, min, max); 
  
  uint32_t shape = visualization_msgs::Marker::CUBE; 
  visualization_msgs::Marker marker; 
  marker.header.frame_id = "/velodyne"; 
  marker.header.stamp = ros::Time::now(); 
  
  marker.ns = "cube"; 
  marker.id = 0; 
  marker.type = shape; 
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = centroid[0]; 
  marker.pose.position.y = centroid[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0; 
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0; 
  
  marker.scale.x = (max[0]-min[0]); 
  marker.scale.y = (max[1]-min[1]); 
  marker.scale.z = (max[2]-min[2]); 
  
  if (marker.scale.x ==0) 
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
//  marker.color.r = r; 
  marker.color.g = 1.0f; 
//  marker.color.b = b; 
  marker.color.a = 1.0; 

  marker.lifetime = ros::Duration(1); 
//   marker.lifetime = ros::Duration(0.5); 
  return marker; 
}

void getBoundingBox(vector<PointCloud<PointXYZ>>  clusteredPoints,
                    vector<PointCloud<PointXYZ>>& bbPoints,visualization_msgs::MarkerArray& ma){
//     cout << "the number of cluster is "<< clusteredPoints.size() <<endl;
    
    for (int iCluster = 0; iCluster < clusteredPoints.size(); iCluster++){
        Mat m (picScale*roiM, picScale*roiM, CV_8UC1, Scalar(0));
        float initPX = clusteredPoints[iCluster][0].x + roiM/2;
        float initPY = clusteredPoints[iCluster][0].y + roiM/2;
        int initX = floor(initPX*picScale);
        int initY = floor(initPY*picScale);
        int initPicX = initX;
        int initPicY = picScale*roiM - initY;
        int offsetInitX = roiM*picScale/2 - initPicX;
        int offsetInitY = roiM*picScale/2 - initPicY;

        int numPoints = clusteredPoints[iCluster].size();
        vector<Point> pointVec(numPoints);
        vector<Point2f> pcPoints(4);
        float minMx, minMy, maxMx, maxMy;
        float minM = 999; float maxM = -999; float maxZ = -99;
        float maxdistance=-999;
        // for center of gravity
        float sumX = 0; float sumY = 0;

//        cout << "the number of cluster i is "<< numPoints <<endl;

        for (int iPoint = 0; iPoint < clusteredPoints[iCluster].size(); iPoint++){
            float pX = clusteredPoints[iCluster][iPoint].x;
            float pY = clusteredPoints[iCluster][iPoint].y;
            float pZ = clusteredPoints[iCluster][iPoint].z;
            // cast (-15 < x,y < 15) into (0 < x,y < 30)
            float roiX = pX + roiM/2;
            float roiY = pY + roiM/2;
            // cast 30mx30m into 900x900 scale
            int x = floor(roiX*picScale);
            int y = floor(roiY*picScale);
            // cast into image coordinate
            int picX = x;
            int picY = picScale*roiM - y;
            // offset so that the object would be locate at the center
            int offsetX = picX + offsetInitX;
            int offsetY = picY + offsetInitY;
            
/*            std::cout << "----------------------" << std::endl;
            std::cout << offsetX << std::endl;
            std::cout << offsetY << std::endl;
            std::cout << "---------------------" << std::endl;
*/
          //  m.at<uchar>(offsetY, offsetX) = 255;
            pointVec[iPoint] = Point(offsetX, offsetY);
            // calculate min and max slope
            float m = pY/pX;
            ////////////std::cout << "m:" << m << std::endl;

            if(m < minM) {
                minM = m;
                minMx = pX;
                minMy = pY;
                /////////std::cout << "mmmmmm" << m << std::endl;

            }
            if(m > maxM) {
                maxM = m;
                maxMx = pX;
                maxMy = pY;
                /////////std::cout << "MMMMM" << std::endl;
            }

            float xydis = sqrt(pX*pX+pY*pY);

            if(xydis > maxdistance)maxdistance = xydis;


            //get maxZ
            if(pZ > maxZ) maxZ = pZ;

            sumX += offsetX;
            sumY += offsetY; 

        }
        // L shape fitting parameters
        float xDist = maxMx - minMx;
        float yDist = maxMy - minMy;
        float slopeDist = sqrt(xDist*xDist + yDist*yDist);
        /////////////////////std::cout << "boxFitting  slopeDist" << slopeDist << std::endl;
        float slope = (maxMy - minMy)/(maxMx - minMx);

        // random variable
        mt19937_64 mt(0);
        uniform_int_distribution<> randPoints(0, numPoints-1);

        // start l shape fitting for car like object
        // lSlopeDist = 10000, lnumPoints = 30000;
        if(slopeDist > lSlopeDist && numPoints > lnumPoints && (maxMy > 8 || maxMy < -5)){
//        if(1){
            float maxDist = 0;
            float maxDx, maxDy;

            // 80 random points, get max distance
            for(int i = 0; i < ramPoints; i++){
                int pInd = randPoints(mt);
                assert(pInd >= 0 && pInd < clusteredPoints[iCluster].size());
                float xI = clusteredPoints[iCluster][pInd].x;
                float yI = clusteredPoints[iCluster][pInd].y;

                // from equation of distance between line and point
                float dist = abs(slope*xI-1*yI+maxMy-slope*maxMx)/sqrt(slope*slope + 1);
                if(dist > maxDist) {
                    maxDist = dist;
                    maxDx = xI;
                    maxDy = yI;
                }
            }

            // for center of gravity
            // maxDx = sumX/clusteredPoints[iCluster].size();
            // maxDy = sumY/clusteredPoints[iCluster].size();

            // vector adding
            float maxMvecX = maxMx - maxDx;
            float maxMvecY = maxMy - maxDy;
            float minMvecX = minMx - maxDx;
            float minMvecY = minMy - maxDy;
            float lastX = maxDx + maxMvecX + minMvecX;
            float lastY = maxDy + maxMvecY + minMvecY;

            pcPoints[0] = Point2f(minMx, minMy);
            pcPoints[1] = Point2f(maxDx, maxDy);
            pcPoints[2] = Point2f(maxMx, maxMy);
            pcPoints[3] = Point2f(lastX, lastY);

            ////////////std::cout << " pcPoints[0]  " << pcPoints[0].x << " " <<  pcPoints[0].y << std::endl;
            ///////////std::cout << " pcPoints[1]  " << pcPoints[1].x << " " <<  pcPoints[1].y << std::endl;
            ///////////std::cout << " pcPoints[2]  " << pcPoints[2].x << " " <<  pcPoints[2].y << std::endl;
            //////////std::cout << " pcPoints[3]  " << pcPoints[3].x << " " <<  pcPoints[3].y << std::endl;

            ///////////////std::cout << "boxFitting  maxZ" << maxZ << std::endl;
            bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            // ///////////////////////    std::cout << "boxFitting   isPromising:" << isPromising << std::endl;
            if(!isPromising) continue;
        }
        else{
            //MAR fitting
            RotatedRect rectInfo = minAreaRect(pointVec);
            Point2f rectPoints[4]; rectInfo.points( rectPoints );
            // covert points back to lidar coordinate
            getPointsInPcFrame(rectPoints, pcPoints, offsetInitX, offsetInitY);
            // rule based filter
            bool isPromising = ruleBasedFilter(pcPoints, maxZ, numPoints);
            if(!isPromising) continue;
            // for visualization
//            for( int j = 0; j < 4; j++ )
//                line( m, rectPoints[j], rectPoints[(j+1)%4ne], Scalar(255,255,0), 1, 8 );
//            imshow("Display Image", m);
//            waitKey(0);
            /////////std::cout << " else  pcPoints[0]  " << pcPoints[0].x << " " <<  pcPoints[0].y << std::endl;
            ////////std::cout << " else  pcPoints[1]  " << pcPoints[1].x << " " <<  pcPoints[1].y << std::endl;
            ///////std::cout << " else  pcPoints[2]  " << pcPoints[2].x << " " <<  pcPoints[2].y << std::endl;
            ///////std::cout << " else  pcPoints[3]  " << pcPoints[3].x << " " <<  pcPoints[3].y << std::endl;

        }

        // make pcl cloud for 3d bounding box
        PointCloud<PointXYZ> oneBbox;
        for(int pclH = 0; pclH < 2; pclH++){
            for(int pclP = 0; pclP < 4; pclP++){
                PointXYZ o;
                o.x = pcPoints[pclP].x;
                o.y = pcPoints[pclP].y;
                if(pclH == 0) o.z = -sensorHeight;
                else o.z = maxZ;
                oneBbox.push_back(o);
            }
        }

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


        bbPoints.push_back(oneBbox);
//        clustered2D[iCluster] = m;

        visualization_msgs::Marker mac = mark_cluster(clusteredPoints[iCluster]);
        ma.markers.push_back(mac);

    }

    
///////////////std::cout << "boxFitting   bbPoints:" << bbPoints.size() << std::endl;

}


//  候选框拟合处理
vector<PointCloud<PointXYZ>> boxFitting(PointCloud<PointXYZ>::Ptr elevatedCloud,
                array<array<int, numGrid>, numGrid> cartesianData,
                int numCluster,visualization_msgs::MarkerArray& ma){  // ma数据
    vector<PointCloud<PointXYZ>>  clusteredPoints(numCluster);
    getClusteredPoints(elevatedCloud, cartesianData, clusteredPoints);  // 聚类点
    vector<PointCloud<PointXYZ>>  bbPoints;
    getBoundingBox(clusteredPoints, bbPoints,ma);  // 得到候选框

    return bbPoints;
//    vector<vector<float>>  bBoxes(numCluster,  vector<float>(6));
//
//    return bBoxes;
}

