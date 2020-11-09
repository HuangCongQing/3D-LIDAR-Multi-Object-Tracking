#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"
// #include <pcl_ros/transforms.h>


#include <vector>
#include <iostream>
#include <math.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "ground_removal.h"
#include "component_clustering.h"
#include "box_fitting.h"
#include "imm_ukf_jpda.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

int counta = 0;

ros::Publisher pub;

ros::Publisher vis_pub;

tf::TransformListener* tran;
// tf::TransformListener* tran2;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr input){
  PointCloud<pcl::PointXYZ> cloud;
  PointCloud<pcl::PointXYZ>::Ptr elevatedCloud (new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr groundCloud   (new pcl::PointCloud<pcl::PointXYZ>());

  // Convert from ros msg to PCL::PointCloud data type
  fromROSMsg (*input, cloud);

  //start processing pcl::pointcloud
  // process global cloud point
  // groundRemove(globalCloud, elevatedCloud, groundCloud);
  //process local ego tf point cloud
  groundRemove(cloud, elevatedCloud, groundCloud);

  //todo: using union find tree
  // make array with initialized 0
  int numCluster = 0; // global variable?
  array<array<int, numGrid>, numGrid> cartesianData{};
  componentClustering(elevatedCloud, cartesianData, numCluster);

  // for visualization
  PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  makeClusteredCloud(elevatedCloud, cartesianData, clusteredCloud);

  // Convert from PCL::PointCloud to ROS data type
  clusteredCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  elevatedCloud->header.frame_id = cloud.header.frame_id; // add "velo_link"
  sensor_msgs::PointCloud2 output;
  // toROSMsg(*clusteredCloud, output);
  toROSMsg(*elevatedCloud, output);
  
  // Publish the data.
  pub.publish(output);

  counta ++;
  cout << "Frame: "<<counta << "----------------------------------------"<< endl;

  vector<PointCloud<PointXYZ>> bBoxes = boxFitting(elevatedCloud, cartesianData, numCluster);
  assert(bBoxes.size() != 0);

  // convert local to global-------------------------
  double timestamp = cloud.header.stamp;
  vector<vector<double>> egoPoints;
  getOriginPoints(timestamp, egoPoints);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(egoPoints[0][0], egoPoints[0][1], 0.0) );
  tf::Quaternion q;
  ros::Time input_time = input->header.stamp;
  q.setRPY(0, 0, egoPoints[0][2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, input_time, "velo_link", "global"));
  // cout << "transform "<< egoPoints[0][0] << " "<<egoPoints[0][1]<< " "<<egoPoints[0][2]<<endl;
  
  PointCloud<PointXYZ> newBox;
  for(int i = 0; i < bBoxes.size(); i++ ){
    // cout << "before converting "<<getCpFromBbox(bBoxes[i])(0) << " "<<getCpFromBbox(bBoxes[i])(1)<<endl;
    bBoxes[i].header.frame_id = "velo_link";
    tran->waitForTransform("/global", "/velo_link", input_time, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/global", bBoxes[i], newBox, *tran);
    bBoxes[i] = newBox;
  }
  //end converting----------------------------------------
  PointCloud<PointXYZ> targetPoints;
  vector<vector<double>> targetVandYaw;
  vector<int> trackManage;
  vector<bool> isStaticVec;
  vector<bool> isVisVec;
  vector<PointCloud<PointXYZ>> visBBs;
  immUkfJpdaf(bBoxes, timestamp, targetPoints, targetVandYaw, trackManage, isStaticVec, isVisVec, visBBs);

  assert(targetPoints.size() == trackManage.size());
  assert(targetPoints.size()== targetVandYaw.size());

  

  // converting from global to ego tf for visualization
  // processing targetPoints
  PointCloud<PointXYZ> egoTFPoints;
  targetPoints.header.frame_id = "global";
  pcl_ros::transformPointCloud("/velo_link", targetPoints, egoTFPoints, *tran);

  //processing visBBs
  PointCloud<PointXYZ> visEgoBB;
  for(int i = 0; i < visBBs.size(); i++){
    visBBs[i].header.frame_id = "global";
    pcl_ros::transformPointCloud("/velo_link", visBBs[i], visEgoBB, *tran);
    visBBs[i] = visEgoBB;
  }
  //end converting to ego tf-------------------------


  // origin arrows visualizing start---------------------------------------------
  // for(int i = 0; i < egoPoints.size(); i++){
  // // for(int i = 0; i < 2; i++){
  //   visualization_msgs::Marker egoArrows;
  //   egoArrows.lifetime = ros::Duration();

  //   // egoArrows.header.frame_id = "/velo_link";
  //   egoArrows.header.frame_id = "/global";
  //   egoArrows.header.stamp= ros::Time::now();
  //   egoArrows.ns = "egoArrows";
  //   egoArrows.action = visualization_msgs::Marker::ADD;
  //   egoArrows.type =  visualization_msgs::Marker::ARROW;
  //   // red
  //   egoArrows.color.r = 1.0;
  //   egoArrows.color.a = 1.0;
  //   egoArrows.id = i;
  //   geometry_msgs::Point p;

  //   // p.x = egoPoints[i][0];
  //   // p.y = egoPoints[i][1];
  //   p.x = 1;
  //   p.y = 1;
  //   p.z = 0;
  //   double tv   = 15;
  //   // double tyaw = egoPoints[i][2];
  //   double tyaw = 0;

  //   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  //   egoArrows.pose.position.x = p.x;
  //   egoArrows.pose.position.y = p.y;
  //   egoArrows.pose.position.z = p.z;


  //   // convert from 3 angles to quartenion
  //   tf::Matrix3x3 obs_mat;
  //   obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
  //   tf::Quaternion q_tf;
  //   obs_mat.getRotation(q_tf);
  //   egoArrows.pose.orientation.x = q_tf.getX();
  //   // cout << "visualize arrow tyaw " << tyaw << endl;
  //   egoArrows.pose.orientation.y = q_tf.getY();
  //   egoArrows.pose.orientation.z = q_tf.getZ();
  //   egoArrows.pose.orientation.w = q_tf.getW();

  //   // Set the scale of the egoArrows -- 1x1x1 here means 1m on a side
  //   egoArrows.scale.x = tv;
  //   egoArrows.scale.y = 0.1;
  //   egoArrows.scale.z = 0.1;

  //   vis_pub.publish(egoArrows);
  // }
  // origin arrows end -------------------------------------------------------


  // tracking arrows visualizing start---------------------------------------------
  for(int i = 0; i < targetPoints.size(); i++){
    visualization_msgs::Marker arrowsG;
    arrowsG.lifetime = ros::Duration(0.1);
    if(trackManage[i] == 0 ) {
      continue;
    }
    if(isVisVec[i] == false ) {
      continue;
    }
    if(isStaticVec[i] == true){
      continue;
    }
    arrowsG.header.frame_id = "/velo_link";
    arrowsG.header.stamp= ros::Time::now();
    arrowsG.ns = "arrows";
    arrowsG.action = visualization_msgs::Marker::ADD;
    arrowsG.type =  visualization_msgs::Marker::ARROW;
    // green
    arrowsG.color.g = 1.0f;
    arrowsG.color.a = 1.0;  
    arrowsG.id = i;
    geometry_msgs::Point p;
    // assert(targetPoints[i].size()==4);
    p.x = egoTFPoints[i].x;
    p.y = egoTFPoints[i].y;
    p.z = -1.73/2;
    double tv   = targetVandYaw[i][0];
    double tyaw = targetVandYaw[i][1];

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    arrowsG.pose.position.x = p.x;
    arrowsG.pose.position.y = p.y;
    arrowsG.pose.position.z = p.z;

    // convert from 3 angles to quartenion
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    arrowsG.pose.orientation.x = q_tf.getX();
    arrowsG.pose.orientation.y = q_tf.getY();
    arrowsG.pose.orientation.z = q_tf.getZ();
    arrowsG.pose.orientation.w = q_tf.getW();

    // Set the scale of the arrowsG -- 1x1x1 here means 1m on a side
    arrowsG.scale.x = tv;
    arrowsG.scale.y = 0.1;
    arrowsG.scale.z = 0.1;

    vis_pub.publish(arrowsG);
  }

  
  // tracking points visualizing start---------------------------------------------
  
  visualization_msgs::Marker pointsY, pointsG, pointsR, pointsB;
  pointsY.header.frame_id = pointsG.header.frame_id = pointsR.header.frame_id = pointsB.header.frame_id = "velo_link";
  pointsY.header.stamp= pointsG.header.stamp= pointsR.header.stamp =pointsB.header.stamp = ros::Time::now();
  pointsY.ns= pointsG.ns = pointsR.ns =pointsB.ns=  "points";
  pointsY.action = pointsG.action = pointsR.action = pointsB.action = visualization_msgs::Marker::ADD;
  pointsY.pose.orientation.w = pointsG.pose.orientation.w  = pointsR.pose.orientation.w =pointsB.pose.orientation.w= 1.0;

  pointsY.id = 1;
  pointsG.id = 2;
  pointsR.id = 3;
  pointsB.id = 4;
  pointsY.type = pointsG.type = pointsR.type = pointsB.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  pointsY.scale.x =pointsG.scale.x =pointsR.scale.x = pointsB.scale.x=0.5;
  pointsY.scale.y =pointsG.scale.y =pointsR.scale.y = pointsB.scale.y = 0.5;

  // yellow
  pointsY.color.r = 1.0f;
  pointsY.color.g = 1.0f;
  pointsY.color.b = 0.0f;
  pointsY.color.a = 1.0;

  // green
  pointsG.color.g = 1.0f;
  pointsG.color.a = 1.0;

  // red
  pointsR.color.r = 1.0;
  pointsR.color.a = 1.0;

  // blue 
  pointsB.color.b = 1.0;
  pointsB.color.a = 1.0;

  for(int i = 0; i < targetPoints.size(); i++){
    if(trackManage[i] == 0) continue;
    geometry_msgs::Point p;
    // p.x = targetPoints[i].x;
    // p.y = targetPoints[i].y;
    p.x = egoTFPoints[i].x;
    p.y = egoTFPoints[i].y;
    p.z = -1.73/2;
    // cout << trackManage[i] << endl;
    if(isStaticVec[i] == true){
      pointsB.points.push_back(p); 
    }
    else if(trackManage[i] < 5 ){
      pointsY.points.push_back(p);
    }
    else if(trackManage[i] == 5){
      pointsG.points.push_back(p);
    }
    else if(trackManage[i] > 5){
      pointsR.points.push_back(p); 
    }
  }
  vis_pub.publish(pointsY);
  // cout << "pointsG" << pointsG.points[0].x << " "<< pointsG.points[0].y << endl;
  vis_pub.publish(pointsG);
  vis_pub.publish(pointsR);
  vis_pub.publish(pointsB);
  // tracking poiints visualizing end---------------------------------------------

  // bounding box visualizing start---------------------------------------------
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "velo_link";
  line_list.header.stamp = ros::Time::now();
  line_list.ns =  "boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  //LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.1;
  // Points are green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  int id = 0;string ids;
  for(int objectI = 0; objectI < visBBs.size(); objectI ++){
    for(int pointI = 0; pointI < 4; pointI++){
      assert((pointI+1)%4 < visBBs[objectI].size());
      assert((pointI+4) < visBBs[objectI].size());
      assert((pointI+1)%4+4 < visBBs[objectI].size());
      id ++; ids = to_string(id);
      geometry_msgs::Point p;
      p.x = visBBs[objectI][pointI].x;
      p.y = visBBs[objectI][pointI].y;
      p.z = visBBs[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = visBBs[objectI][(pointI+1)%4].x;
      p.y = visBBs[objectI][(pointI+1)%4].y;
      p.z = visBBs[objectI][(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = visBBs[objectI][pointI].x;
      p.y = visBBs[objectI][pointI].y;
      p.z = visBBs[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = visBBs[objectI][pointI+4].x;
      p.y = visBBs[objectI][pointI+4].y;
      p.z = visBBs[objectI][pointI+4].z;
      line_list.points.push_back(p);

      p.x = visBBs[objectI][pointI+4].x;
      p.y = visBBs[objectI][pointI+4].y;
      p.z = visBBs[objectI][pointI+4].z;
      line_list.points.push_back(p);
      p.x = visBBs[objectI][(pointI+1)%4+4].x;
      p.y = visBBs[objectI][(pointI+1)%4+4].y;
      p.z = visBBs[objectI][(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }

  //line list end
  vis_pub.publish(line_list);
  // bounding box visualizing end---------------------------------------------
}

// void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
//   ROS_INFO("Imu angular valo x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
// }


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // ros::Subscriber subImu = nh.subscribe ("imu_data", 10, imu_cb);

  tf::TransformListener lr(ros::Duration(10));
  tran=&lr;

  // tf::TransformListener lr2(ros::Duration(10));
  // tran2=&lr2;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 160, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


  // //TF Broadcasterの実体化
  // tf::TransformBroadcaster global_robot_broadcaster;
   
  // //Robot位置と姿勢(x,y,yaw)の取得
  // // double x=GetRobotPositionX();
  // // double y=GetRobotPositionY();
  // // double yaw=GetRobotPositionYaw();
  // double x = 0;
  // double y = 0;
  // double yaw = M_PI/2;

 
  // //yawのデータからクォータニオンを作成
  // geometry_msgs::Quaternion robot_quat=tf::createQuaternionMsgFromYaw(yaw);
   
  // //robot座標系の元となるロボットの位置姿勢情報格納用変数の作成
  // geometry_msgs::TransformStamped robotState;
   
  // //現在の時間の格納
  // robotState.header.stamp = ros::Time::now();
   
  // //座標系globalとrobotの指定
  // robotState.header.frame_id = "global";
  // robotState.child_frame_id  = "velo_link";
   
  // //global座標系からみたrobot座標系の原点位置と方向の格納
  // robotState.transform.translation.x = x;
  // robotState.transform.translation.y = y;
  // robotState.transform.translation.z = -1.73/2;
  // robotState.transform.rotation = robot_quat;
   
  // //tf情報をbroadcast(座標系の設定)
  // global_robot_broadcaster.sendTransform(robotState);

  // Spin
  ros::spin ();
}