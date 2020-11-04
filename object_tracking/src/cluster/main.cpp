//base on pingjia3,code adjust

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"

#include <vector>
#include <iostream>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <utility>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>

#include <object_tracking/trackbox.h>  // 没有这个文件？

#include "component_clustering.h"
#include "box_fitting.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;

int counta = 0;

ros::Publisher pub;

ros::Publisher vis_pub;

ros::Publisher g_costmap_pub;

ros::Publisher obs_pub;

ros::Publisher marker_array_pub_;

ros::Publisher box_pub;

// 回调函数
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

  PointCloud<pcl::PointXYZ>::Ptr none_ground_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // Convert from ros msg to PCL::PointCloud data type
  fromROSMsg (*input, *none_ground_cloud);  // 转为PCL数据格式PCL::PointCloud

  //start processing pcl::pointcloud

  int numCluster = 0; // global variable
  array<array<int, numGrid>, numGrid> cartesianData{};
  componentClustering(none_ground_cloud, cartesianData, numCluster);
  //cout << "num is "<<numCluster<<endl;
  // for visualization
  PointCloud<pcl::PointXYZ>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
//  PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  makeClusteredCloud(none_ground_cloud, cartesianData, clusteredCloud);

  // Convert from PCL::PointCloud to ROS data type
  clusteredCloud->header.frame_id = none_ground_cloud->header.frame_id; // add "velo_link"
  sensor_msgs::PointCloud2 output;
  toROSMsg(*clusteredCloud, output);

  static int count = 0;
  static nav_msgs::OccupancyGrid og;
  if (!count)
    setOccupancyGrid(&og);

  og.header.frame_id = none_ground_cloud->header.frame_id;

  // create cost map with pointcloud
  std::vector<int> cost_map = createCostMap(*none_ground_cloud);

  /*
  bool filter = false;
  if (filter)
    cost_map = filterCostMap(cost_map);
  */

  og.data.insert(og.data.end(), cost_map.begin(), cost_map.end());
  g_costmap_pub.publish(og);
  og.data.clear();
  count++;

  object_tracking::ObstacleList clu_obs;
  setObsMsg(none_ground_cloud, cartesianData, clu_obs);
  obs_pub.publish(clu_obs); 
  pub.publish(output);
  
  counta ++;
  cout << "cluster Frame: "<<counta << "----------------------------------------"<< endl;

  visualization_msgs::MarkerArray ma;

  vector<PointCloud<PointXYZ>> bBoxes = boxFitting(none_ground_cloud, cartesianData, numCluster,ma);  // bBoxes----一个数组(候选框8个坐标)

  object_tracking::trackbox boxArray; // boxArray--候选框8个坐标数组 的 数组
    
  boxArray.header = input->header;
  boxArray.box_num = bBoxes.size();
  for(int i = 0;i < bBoxes.size();i++)
  {
    boxArray.x1.push_back(bBoxes[i][0].x);
    boxArray.x1.push_back(bBoxes[i][0].y);
    boxArray.x1.push_back(bBoxes[i][0].z);

    boxArray.x2.push_back(bBoxes[i][1].x);
    boxArray.x2.push_back(bBoxes[i][1].y);
    boxArray.x2.push_back(bBoxes[i][1].z);

    boxArray.x3.push_back(bBoxes[i][2].x);
    boxArray.x3.push_back(bBoxes[i][2].y);
    boxArray.x3.push_back(bBoxes[i][2].z);

    boxArray.x4.push_back(bBoxes[i][3].x);
    boxArray.x4.push_back(bBoxes[i][3].y);
    boxArray.x4.push_back(bBoxes[i][3].z);

    boxArray.y1.push_back(bBoxes[i][4].x);
    boxArray.y1.push_back(bBoxes[i][4].y);
    boxArray.y1.push_back(bBoxes[i][4].z);

    boxArray.y2.push_back(bBoxes[i][5].x);
    boxArray.y2.push_back(bBoxes[i][5].y);
    boxArray.y2.push_back(bBoxes[i][5].z);

    boxArray.y3.push_back(bBoxes[i][6].x);
    boxArray.y3.push_back(bBoxes[i][6].y);
    boxArray.y3.push_back(bBoxes[i][6].z);

    boxArray.y4.push_back(bBoxes[i][7].x);
    boxArray.y4.push_back(bBoxes[i][7].y);
    boxArray.y4.push_back(bBoxes[i][7].z);
  }

//************************************cube visualiaztion******************************

  box_pub.publish(boxArray);   // boxArray--候选框8个坐标数组 的 数组

  // cout << "boxArray is " << boxArray<< endl;  // bBoxes
  cout << "size of bBoxes is " << bBoxes.size() << endl;  //bBoxes数量 size of bBoxes is 2
  cout << "size of marker is " << ma.markers.size() << endl; // marker数量 size of marker is 2
  marker_array_pub_.publish(ma);


//************************************end of cube*************************************




//*********************************************bBoxes visualization***************************************

  visualization_msgs::Marker line_list; //将候选框8个点连线
  line_list.header.frame_id = "velodyne";   // 定义frame_id (rviz需要设置世界坐标系为velodyne)
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
  for(int objectI = 0; objectI < bBoxes.size(); objectI ++){
    for(int pointI = 0; pointI < 4; pointI++){
      assert((pointI+1)%4 < bBoxes[objectI].size());
      assert((pointI+4) < bBoxes[objectI].size());
      assert((pointI+1)%4+4 < bBoxes[objectI].size());
      id ++; ids = to_string(id);
      geometry_msgs::Point p;
      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4].x;
      p.y = bBoxes[objectI][(pointI+1)%4].y;
      p.z = bBoxes[objectI][(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI].x;
      p.y = bBoxes[objectI][pointI].y;
      p.z = bBoxes[objectI][pointI].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);

      p.x = bBoxes[objectI][pointI+4].x;
      p.y = bBoxes[objectI][pointI+4].y;
      p.z = bBoxes[objectI][pointI+4].z;
      line_list.points.push_back(p);
      p.x = bBoxes[objectI][(pointI+1)%4+4].x;
      p.y = bBoxes[objectI][(pointI+1)%4+4].y;
      p.z = bBoxes[objectI][(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }

  //line list end
  vis_pub.publish(line_list);   //发布者  cluster_obs -- 话题topic名
  // bounding box visualizing end---------------------------------------------

}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "cluster");  // "cluster"--节点名
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("none_ground_topic", 160, cloud_cb);  //订阅者  none_ground_topic -- 话题topic名

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);  //发布者  output -- 话题topic名

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );  //发布者  visualization_marker -- 话题topic名
  marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("cluster_ma", 10);   //发布者  cluster_ma -- 话题topic名
  g_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);    //发布者  realtime_cost_map -- 话题topic名

  obs_pub = nh.advertise<object_tracking::ObstacleList>("cluster_obs",10);   //发布者  cluster_obs -- 话题topic名

  box_pub = nh.advertise<object_tracking::trackbox>("track_box",10);   //发布者  track_box -- 话题topic名

  // Spin
  ros::spin ();
}