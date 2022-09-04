<!--
 * @Author: HCQ
 * @Date: 2020-10-27 10:18:56
 * @LastEditTime: 2021-01-31 21:06:06
 * @LastEditors: Please set LastEditors
 * @Description: 3D-LIDAR Multi Object Tracking for Autonomous Driving（Master论文）
 * @FilePath: /3D-LIDAR-Multi-Object-Tracking/README.md
-->

# 3D-LIDAR-Multi-Object-Tracking

3D-MOT(多目标检测和追踪) 代码有详细注解 （2020 · 秋）
更多内容,请参见公众号:双愚

参考：https://github.com/k0suke-murakami/object_tracking
* 🔥pcl学习：https://github.com/HuangCongQing/pcl-learning
* 🔥ros学习：https://github.com/HuangCongQing/ROS


@[双愚](https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking) , 若fork或star请注明来源

#### 版本(建议先看kitti分支)

* [main](https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking) : 使用个人采集数据集
* [kitti](https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking/tree/kitti) : **使用kitti数据集,初学者建议切换看这个分支,无须自己配置**🎉️🎉️🎉️🎉️🎉️

### 两文件夹介绍

此仓库的两文件夹

* [object_tracking](object_tracking): **代码有详细注解,建议先看这个入门**🎉️🎉️🎉️🎉️
* [object_tracking0](object_tracking0):原始代码(包含全部代码)

目录

```shell

├── src
│   ├── groundremove
│   │   └──extract_ground.cpp // 提取地面 没有使用？！！！
│   │   └── gaus_blur.cpp           //高斯模糊  #include "ground_removal.h"
│   │   └── ground_removal.cpp   //地面去除 #include "gaus_blur.h"  各种函数的集合，没有主函数
│   │   └── main.cpp  //  #include "ground_removal.h"
│   └──  cluster
│          ├── box_fitting.cpp        //  Bounding Box Fitting 边界框拟合
│          ├── component_clustering.cpp// 利用连通组件聚类来区分提升点中的每个可能的对象。
│          ├── main.cpp          //  #include "component_clustering.h"  "box_fitting.h"
└── tracking
    ├── Eigen
    │   ├── ...
    ├── ukf.cpp        //  Unscented Kalman Filter (UKF)无损滤波器
    ├── imm_ukf_jpda.cpp//   #include "ukf.h" IMM-UK-JPDAF的“耦合”滤波器
    └── main.cpp         //  #include "imm_ukf_jpda.h"
```

### Intro

This package includes **Ground Removal, Object Clustering, Bounding Box, IMM-UKF-JPDAF, Track Management and Object Classification** for 3D-LIDAR multi object tracking.
The idea is mainly come from this [paper](https://repository.tudelft.nl/islandora/object/uuid:f536b829-42ae-41d5-968d-13bbaa4ec736?collection=education).

代码对应论文：[3D-LIDAR Multi Object Tracking for Autonomous Driving（Master论文）](https://repository.tudelft.nl/islandora/object/uuid:f536b829-42ae-41d5-968d-13bbaa4ec736?collection=education)

* **论文阅读笔记：https://www.yuque.com/docs/share/81734320-21d8-4b50-993a-faa6d22d513f?# 《3D-LIDAR Multi Object Tracking for Autonomous Driving（Master论文）》**
* 代码分析笔记：https://www.yuque.com/docs/share/c125da75-50fd-4d49-bfc9-08ba42fc3188?# 《Code（3D-LIDAR Multi Object Tracking）》

更多内容,请参见公众号:**双愚**

下面介绍用kitti数据集相关操作

### Setup

##### Frameworks and Packages

Make sure you have the following is installed:

- [ROS Kinetic](http://wiki.ros.org/kinetic)
- [PCL 1.7.2](http://pointclouds.org/downloads/)
- [Open CV 3.2](https://opencv.org/)【PCL自带opencv，不用安装】

##### Dataset

数据集已处理好，放在百度网盘上，需要自己下载

* kitti_2011_09_26_drive_0005_synced.bag
* 链接: https://pan.baidu.com/s/1sYWHzF11RpyEW25cQ_iNGA  密码: b6pd

### 编译

将本仓库下的2个文件夹移动到catkin_wp/src下，然后执行下面操作

```shell
// 创建环境变量 src中运行
mkdir -p catkin_wp/src
cd catkin_wp/src
catkin_init_workspace

// 编译（需要回到工作空间catkin_wp）
cd ..
catkin_make  // 产生build和devel文件夹


//设置环境变量，找到src里的功能包(每个新的shell窗口都要执行以下source devel/setup.bash)
source devel/setup.bash  // 不同shell，不同哦.sh  .zsh           通过设置gedit ~/.zshrc，不用每次都source
```

详情可参考：https://www.yuque.com/docs/share/e59d5c91-b46d-426a-9957-cd262f5fc241?# 《09.创建工作空间与功能包※※※》

### 修改配置文件

举例：修改输入**topic**和**对应的**`frame_id(有好几处,可以全局搜索进行修改)`

```bash
cd object_tracking/src/groundremove/main.cpp

#第8行      "/kitti/velo/pointcloud" --话题名(可以根据不同数据集修改topic话题名) 
ros::Subscriber sub = nh.subscribe("/kitti/velo/pointcloud", 160, cloud_cb); 

# 修改frame_id = "velo_link"

```

### Start

各模块代码路径：

* [src/groundremove](object_tracking/src/groundremove)
* [src/cluster](object_tracking/src/cluster)
* [tracking](object_tracking/tracking)

#### ~~PLEASE make sure you load the files, `src/ego_velo.txt` and `src/ego_yaw.txt` in `src/imm_ukf_jpda.cpp` l68, l69~~

##### Terminal 1

```
roscore
```

##### Terminal 2

`--loop`循环paly不推荐加，tracking和上一帧有关，误差越来越大

```
# kitti官方 注意修改路径path
rosbag play path/kitti_2011_09_26_drive_0005_synced.bag --loop


```

##### Terminal 3

```
rviz
```

![arch](object_tracking/pic/setting.png)

##### Terminal 4

```

#  推荐运行launch
roslaunch  object_tracking test.launch
#  复杂
rosrun object_tracking ground
rosrun object_tracking cluster
rosrun object_tracking tracking

```

### Result

![](https://cdn.nlark.com/yuque/0/2021/png/232596/1612101391954-0ff20177-dc25-4b69-8530-e76254c4dc64.png)

![arch](object_tracking//pic/result2.png)

###### Youtube [Clip](https://www.youtube.com/watch?v=zzFpTVk2Uj0)

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/zzFpTVk2Uj0/0.jpg)](https://www.youtube.com/watch?v=zzFpTVk2Uj0)

### License

Copyright (c) [双愚](https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking). All rights reserved.

Licensed under the [MIT](./LICENSE) License.





**最后，如果您想要支持我的工作，请扫描下面的二维码**

![image](https://user-images.githubusercontent.com/20675770/174442478-705129f7-ca4d-4e89-9b21-7e1b84817940.png)
