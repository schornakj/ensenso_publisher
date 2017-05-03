cmake_minimum_required(VERSION 2.8.3)
project(ensenso_publisher)

add_compile_options(-std=c++1y)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(PCL 1.8 REQUIRED QUIET)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ensenso_publisher_node src/ensenso_publisher.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ensenso_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ensenso_publisher_node ${catkin_LIBRARIES})
