<!--
 * @Author: your name
 * @Date: 2020-10-27 10:39:37
 * @LastEditTime: 2020-11-05 16:43:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /3D-LIDAR-Multi-Object-Tracking/object_tracking/README.md
-->
# ROS Package for Object Detecton/Tracking

## Please notice this repository is still in progress.

### Intro
This package includes **Ground Removal, Object Clustering, Bounding Box, IMM-UKF-JPDAF, Track Management and Object Classification** for 3D-LIDAR multi object tracking.
The idea is mainly come from this [paper](https://repository.tudelft.nl/islandora/object/uuid:f536b829-42ae-41d5-968d-13bbaa4ec736?collection=education).

@[双愚](https://github.com/HuangCongQing) modified , 若fork或star请注明来源

### Setup
##### Frameworks and Packages
Make sure you have the following is installed:
 - [ROS Kinetic](http://wiki.ros.org/kinetic)
 - [PCL 1.7.2](http://pointclouds.org/downloads/)
 - [Open CV 3.2](https://opencv.org/)【PCL自带opencv，不用安装】

##### Dataset
* Download the [Kitti Raw data](http://www.cvlibs.net/datasets/kitti/raw_data.php).

```
// 地址已失效

wget http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0005_sync.zip
wget http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_calib.zip

// 使用下面下载
https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0005/2011_09_26_drive_0005_sync.zip
```


* Convert raw data to rosbag by using the tool made by tomas789. This is [his repository](https://github.com/tomas789/kitti2bag).


### Start

各模块代码路径：

* [src/groundremove](src/groundremove)
* [src/cluster](src/cluster)
* [tracking](tracking)

#### ~~PLEASE make sure you load the files, `src/ego_velo.txt` and `src/ego_yaw.txt` in `src/imm_ukf_jpda.cpp` l68, l69~~



##### Terminal 1
```
roscore
```

##### Terminal 2
```
# kitti官方
rosbag play ~/data/KittiRawdata/2011_09_26_drive_0005_sync/kitti_2011_09_26_drive_0005_synced.bag --loop

# changshu bag
rosbag play  /home/hcq/data/changshu_bag/2018-06-04-16-46-11.bag --loop



```
##### Terminal 3
```
rviz
```
![arch](./pic/setting.png)

##### Terminal 4
```

#  推荐运行launch
roslaunch  object_tracking test.launch
#  复杂
rosrun object_tracking ground
rosrun object_tracking cluster
rosrun object_tracking tracking input:=/kitti/velo/pointcloud

```

### Result


![arch](./pic/result2.png)

######  Youtube [Clip](https://www.youtube.com/watch?v=zzFpTVk2Uj0)

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/zzFpTVk2Uj0/0.jpg)](https://www.youtube.com/watch?v=zzFpTVk2Uj0)
