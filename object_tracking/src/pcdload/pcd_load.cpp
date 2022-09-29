/*
 * @Description: 加载pcd文件，转成msg发送
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-01-04 19:30:14
 * @LastEditTime: 2022-09-30 00:12:36
 * @FilePath: /ws_object_tracking/src/object_tracking/src/pcdload/pcd_load.cpp
 */
// #include "extract_ground.h"  // 加载extract_ground h文件
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
// #include <iostream>
using namespace std;
#include <pcl/point_types.h>

// 遍历文件
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
using namespace std;
namespace fs = boost::filesystem;



// 自定义类型
struct PointXYZIL {
    PCL_ADD_POINT4D
    float intensity;
    float label;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIL,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (float, label, label))


main (int argc, char **argv)
{
  ros::init (argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/kitti/velo/pointcloud", 1000);   // topcic名字, 消息池最大容量1000。
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // 读pcd的时候用我们自己定义的XYZIL
   pcl::PointCloud<PointXYZIL> cloud;

  sensor_msgs::PointCloud2 output;


  //  ============================遍历label文件夹 1次 /home/hcq/github/ws_bagpcd/src/bagpcd/src/pcd_all ============================
  // std::string  pcd_all_path = "/home/hcq/github/ws_bagpcd/src/bagpcd/src/pcd_all";
  // std::string  pcd_all_path = "/home/hcq/下载/label_data";  // 两个文件测试
  // for (auto &it : boost::filesystem::directory_iterator(pcd_all_path)){    // 遍历文件pcd_all/1597975078.481820.rs-bpearl.pcd=============
  //     std::string strPath = it.path().string();  //遍历出来的文件名称
  //     std::cout <<"======strPath:"  << strPath<< std::endl;     // 输出/home/hcq/github/ws_bagpcd/src/bagpcd/src/pcd_all/1597975078.481820.rs-bpearl.pcd
  //     pcl::io::loadPCDFile (strPath, cloud);  //  ===================================加载pcd文件

  //   // // 创建一个pcl::PointXYZI点云，把读到的自定义点云复制到它自带类型的点
  //   pcl::PointCloud<pcl::PointXYZI>  L_cloud;
  //   int number=0;
  //   for (int i = 0; i < cloud.size(); i++) {
  //           // std::cout <<cloud.points[i].label<< std::endl;   // 输出
  //           pcl::PointXYZI o;
  //           o.x  = cloud.points[i].x;
  //           o.y = cloud.points[i].y;
  //           o.z = cloud.points[i].z;
  //           // std::cout <<"cloud.points[i].label:  "<<cloud.points[i].label<< std::endl;   // 输出   
  //           // std::cout <<"typeid(a):  "<<typeid(cloud.points[i].label).name()<< std::endl;   // 输出  typeid(a):  float类型
  //           o.label = (int)cloud.points[i].label; // 字符转数字
  //           if(o.label ){
  //             number++;
  //           }
  //           L_cloud.push_back(o);  // 导入
  //           // std::cout <<"======L_cloud  标签====   "  << L_cloud.points[i].label<< std::endl;   // 输出
  //   }
  //   std::cout <<"======number:"  << number<<" " << L_cloud.size() - number <<" " << L_cloud.size()<< std::endl;     // 输出

  //   //Convert the L_cloud to ROS message
  //   pcl::toROSMsg(L_cloud, output);   // pcl点云转成ROS message
  //   output.header.frame_id = "/base_link";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer

  //   // ROS只发布一次消息退出节点
  //   ros::Rate loop_rate(10);   // 这行表示你希望你发布信息的速度为10Hz,这个频率是指运行上一次loop.sleep()到下一次loop.sleep()之间保持的时间
  //   // 通常情况下，代码运行速度比设定的频率要快，所以如果运行到下一次loop.sleep()后未达到0.1s（1/10Hz），则会开始休眠，等到0.1s后再执行下一句程序。
  //   int count = 0;
  //   std::cout <<"发布一次topic   " << std::endl;   // 输出
  //   while (count < 1)
  //   {
  //     // std::cout <<"pcl_pub.getNumSubscribers()：   "<< pcl_pub.getNumSubscribers()  << std::endl;   // 输出, 下面不运行了？
  //     if(pcl_pub.getNumSubscribers() > 0)  // //必须判断订阅者是否连接 否则消息可能提前发送从而导致订阅者没有收到
  //     {
  //       pcl_pub.publish(output);// 发布消息
  //       // std::cout << output << std::endl;
  //       count ++;
  //       ros::spinOnce();   // 当程序运行到spin()时，程序同样去相应的topic订阅缓存区查看是否有消息。集 中 处 理 本 节 点 所 有 的 回 调 函 数 !所以，有回调函数，就必须设置 ros_spin()和ros_spinOnce（）！
  //       loop_rate.sleep();
  //       ++count;
  //     }
  //   }
  //   std::cout <<"发布结束  " << std::endl;   // 输出
  // }
  
  // ===============================单个文件label循环==================================================
  std::cout <<"======单个文件label循环======" << std::endl;     // 输出
  // ==============================加载pcd数据集=============================
  std::string pcd_path = argv[1];   ///home/hcq/data/test/label_data/label1597975062.pcd
  pcl::io::loadPCDFile (pcd_path, cloud);  // 已标注
  // pcl::io::loadPCDFile ("/home/hcq/github/ws_bagpcd/src/bagpcd/src/pcd_all/1597975062.381742.rs-bpearl.pcd", cloud);  // 原生没标注
  // pcl::io::loadPCDFile ("/home/hcq/data/test/label_data/label1597975062.pcd", cloud);  // 已标注
  // pcl::io::loadPCDFile ("/home/hcq/data/KittiRawdata/2011_09_26_drive_0005_sync/kitti_pcd/1317013472.335336923.pcd", cloud);  // kitti未标注 运行没结果
  std::cout <<"已加载数据 "<< std::endl;   // 输出   
    // // 创建一个pcl::PointXYZI点云，把读到的自定义点云复制到它自带类型的点
  pcl::PointCloud<pcl::PointXYZI>  L_cloud;
  int number=0;
  for (int i = 0; i < cloud.size(); i++) {
          if(cloud.points[i].intensity>0.0){
            std::cout <<cloud.points[i].intensity<< std::endl;   // 输出
          }
          pcl::PointXYZI o;
          o.x  = cloud.points[i].x;
          o.y = cloud.points[i].y;
          o.z = cloud.points[i].z;
          // std::cout <<"cloud.points[i].label:  "<<cloud.points[i].label<< std::endl;   // 输出   
          // std::cout <<"typeid(a):  "<<typeid(cloud.points[i].label).name()<< std::endl;   // 输出  typeid(a):  float类型
          o.intensity = cloud.points[i].intensity;
          // o.label = 0; // 字符转数字
          if(o.intensity ){
            number++;
          }
          L_cloud.push_back(o);  // 导入
          // std::cout <<"======L_cloud  标签====   "  << L_cloud.points[i].label<< std::endl;   // 输出
  }
  // ======number:26628 93372 120000
  std::cout <<"======number:"  << number<<" " << L_cloud.size() - number <<" " << L_cloud.size()<< std::endl;     // 输出

  //Convert the L_cloud to ROS message
  pcl::toROSMsg(L_cloud, output);   // pcl点云转成ROS message
  output.header.frame_id = "velo_link";;//====================注意修改frame_id
  // output.header.frame_id = "/velo_link";;//====================kitti frame_id this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer

  // 连续发布
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);  // 发布消息
    // std::cout << output << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ===============================end==================================================
  return 0;
}