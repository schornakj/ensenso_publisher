cmake_minimum_required(VERSION 2.8.3)
project(ensenso_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  godel_msgs
)

find_package(PCL 1.8 REQUIRED QUIET)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)