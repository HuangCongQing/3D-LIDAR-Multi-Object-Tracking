cmake_minimum_required(VERSION 2.8.3)
project(object_tracking0)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
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
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



#set(SOURCE_FILES src/main.cpp src/gaus_blur.cpp src/gaus_blur.h src/ground_removal.cpp src/ground_removal.h src/component_clustering.cpp src/component_clustering.h src/box_fitting.cpp src/box_fitting.cpp src/ukf.cpp)

set(SOURCE_FILES src/main.cpp src/gaus_blur.cpp src/gaus_blur.h src/ground_removal.cpp src/ground_removal.h src/component_clustering.cpp src/component_clustering.h src/box_fitting.cpp src/box_fitting.cpp src/ukf.cpp src/ukf.h src/imm_ukf_jpda.cpp src/imm_ukf_jpda.h)
add_executable(main ${SOURCE_FILES})


target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
