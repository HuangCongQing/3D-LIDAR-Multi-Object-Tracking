cmake_minimum_required(VERSION 2.8.3)
project(object_tracking)

# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs 
  std_msgs
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


add_message_files(
  FILES
  Obstacle.msg
  ObstacleList.msg
  Road_extract.msg
  trackbox.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_generation std_msgs message_runtime sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

#add_executable(extract_ground src/extract_ground.cpp src/extract_ground.h)
#target_link_libraries(extract_ground 
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}
#)


#set(SOURCE_FILES src/main.cpp src/gaus_blur.cpp src/gaus_blur.h src/ground_removal.cpp src/ground_removal.h src/component_clustering.cpp src/component_clustering.h src/box_fitting.cpp src/box_fitting.cpp src/ukf.cpp)

set(SOURCE_FILES1 src/groundremove/main.cpp src/groundremove/gaus_blur.cpp src/groundremove/ground_removal.cpp )
add_executable(ground ${SOURCE_FILES1})
target_link_libraries(ground ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#src/groundremove/gaus_blur.h src/groundremove/ground_removal.h src/cluster/component_clustering.h  src/cluster/box_fitting.h

set(SOURCE_FILES2 src/cluster/main.cpp src/cluster/component_clustering.cpp src/cluster/box_fitting.cpp )
add_executable(cluster ${SOURCE_FILES2})
target_link_libraries(cluster ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set(SOURCE_FILES3 tracking/main.cpp tracking/imm_ukf_jpda.cpp tracking/ukf.cpp )
add_executable(tracking ${SOURCE_FILES3})
target_link_libraries(tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


