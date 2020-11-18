//base on pingjia3,code adjust

#include <ros/ros.h>
// ros自带的msg类型
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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include "ground_removal.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;

int counta = 0;
ros::Publisher ground_pub; // 发布者
ros::Publisher none_ground_pub;
ros::Publisher auxpoint_pub;

float filter_z_max;
float filter_z_min;
pcl::ConditionalRemoval<pcl::PointXYZ> condrem; // 一个类    /usr/include/pcl-1.8/pcl/filters/conditional_removal.h

// 设置限制：删除点云中不符合用户指定的多个条件的数据点。
void filter_mid_area_limitation()
{
  //创建条件限定下的滤波器
  // 可参考：https://www.yuque.com/huangzhongqing/pcl/ai96k5#nXJU2
  // 可参考：  https://github.com/HuangCongQing/pcl-learning/tree/master/09filters%E6%BB%A4%E6%B3%A2/6%E7%94%A8ConditionalRemoval%E6%88%96RadiusOutlinerRemoval%E7%A7%BB%E9%99%A4%E7%A6%BB%E7%BE%A4%E7%82%B9
  //添加在Z字段上大于0的比较算子
  //GT greater than
  //EQ equal
  //LT less than
  //GE greater than or equal
  //LE less than
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_1(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -15));  // 添加在x字段上大于-15的比较算子
  range_cond->addComparison(cond_1);
  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_2(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 5)); //添加在x字段上小于5的比较算子
  range_cond->addComparison(cond_2);

  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_3(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -50));
  range_cond->addComparison(cond_3);

  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_4(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 50));
  range_cond->addComparison(cond_4);

  condrem.setCondition(range_cond);  // 删除点云中不符合用户指定的多个条件的数据点。
  //创建滤波器并用条件定义对象初始化
}

// cloud_cb回调函数调用此函数
void filter_mid_area(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  condrem.setInputCloud(cloud_in);
  condrem.setKeepOrganized(false);
  condrem.filter(*cloud_in); // 输入输出名字一样，方便
}
//  cloud_cb--回调函数
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{ // input--msg消息

  PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  ;
  PointCloud<pcl::PointXYZ>::Ptr elevatedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Convert from ros msg to PCL::PointCloud data type 转化数据类型msg→pointcloud data
  fromROSMsg(*input, *cloud); // /opt/ros/melodic/include/pcl_conversions/pcl_conversions.h
  //start processing pcl::pointcloud

  // 滤除一定高度以上的点
  pcl::PointCloud<pcl::PointXYZ>::Ptr z_filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;             //设置滤波器对象
  pass.setInputCloud(cloud);                        //设置输入点云cloud（由input转化而来）
  pass.setFilterFieldName("z");                     //设置过滤时所需要点云类型的z字段
  pass.setFilterLimits(filter_z_min, filter_z_max); //设置在过滤字段上的范围（-3.0~1.0）
  //pass.setFilterLimitsNegative (true);     //设置保留范围内的还是过滤掉范围内的
  pass.filter(*z_filter_cloud); //执行滤波，保存过滤结果在cloud_filtered

  filter_mid_area(z_filter_cloud); //  ConditionalRemoval滤波 条件限定下的滤波器（x，y轴上的滤波）

  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*z_filter_cloud, ros_cloud); // 再转回ros数据格式
  ros_cloud.header = input->header;
  auxpoint_pub.publish(ros_cloud); //  发布者  ros_cloud--msg消息

  //地面去除，分出地面点和非地面点
  groundRemove(z_filter_cloud, elevatedCloud, groundCloud); // src/groundremove/ground_removal.cpp

  // Convert from PCL::PointCloud to ROS data type

  elevatedCloud->header.frame_id = cloud->header.frame_id; // 高点
  groundCloud->header.frame_id = cloud->header.frame_id;   // 地面点
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 output2;
  toROSMsg(*groundCloud, output);    // 地面点
  toROSMsg(*elevatedCloud, output2); // 高点

  none_ground_pub.publish(output2); // 发布者 output2--msg消息
  ground_pub.publish(output);       // 发布者 output--msg消息

  counta++;
  cout << "ground Frame: " << counta << "----------------------------------------" << endl;  // 输出在屏幕上：第几帧
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ground"); // 初始化节点的名称--ground
  ros::NodeHandle nh;

//订阅者 ： "velodyne_points"  "/kitti/velo/pointcloud" --话题名(可以根据不同数据集修改话题名)   cloud_cb--回调函数   
  ros::Subscriber sub = nh.subscribe("/kitti/velo/pointcloud", 160, cloud_cb);   // kitti
  // ros::Subscriber sub = nh.subscribe("velodyne_points", 160, cloud_cb);   // changshu.bag
  nh.param<float>("filter_z_max", filter_z_max, 1.0);                   // 参数服务器默认参数（z轴上的滤波）
  nh.param<float>("filter_z_min", filter_z_min, -3.0);                  // nh.param<std::string>("default_param", default_param, "default_value");
  filter_mid_area_limitation();                                         //  //创建条件限定下的滤波器

  // Create a ROS publisher for the output point cloud
  ground_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_topic", 1);
  none_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("none_ground_topic", 1);
  auxpoint_pub = nh.advertise<sensor_msgs::PointCloud2>("aux_points", 1);
  // Spin
  ros::spin();
}