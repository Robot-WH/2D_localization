cmake_minimum_required(VERSION 2.8.3)
project(indoor_localization)

set(CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp               
  rospy
  std_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  message_generation     # 如果自己定义了消息的话  
  cv_bridge              # 如果要使用opencv 必须要这个  
  image_transport
)

include(cmake/glog.cmake)
#find_package(glog REQUIRED)
find_package(catkin REQUIRED )
find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)

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

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 生成共享库  .so  
add_library(${PROJECT_NAME} SHARED
  src/pf_localizer.cpp
  src/motion_Model.cpp
  src/measurement_Model.cpp
  src/ScanMatchers/GaussionNewton.cpp
  src/ScanMatchers/imls_icp.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
)


add_executable(pf_localization app/pf_localization_node.cpp)
target_link_libraries(pf_localization
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(msf_odometry app/msf_odometry_node.cpp)
target_link_libraries(msf_odometry
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

