cmake_minimum_required(VERSION 3.0.2)
project(test_point_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  std_msgs
  my_quickhull
  moveit_ros_planning_interface
  moveit_core
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_point_cloud
#  CATKIN_DEPENDS pcl_convensions pcl_ros roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_point_cloud_node src/test_point_cloud.cpp)
add_executable(test_pcl_subscriber_node src/test_pcl_subscriber.cpp)
add_executable(pcl_subscriber_node src/pcl_subscriber.cpp)
add_executable(main_node src/main.cpp)
add_executable(test_find_center_node src/test_find_center.cpp)

target_link_libraries(test_point_cloud_node
  ${catkin_LIBRARIES}
)

target_link_libraries(test_pcl_subscriber_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pcl_subscriber_node
  ${catkin_LIBRARIES}
)

target_link_libraries(main_node
  ${catkin_LIBRARIES}
)

target_link_libraries(test_find_center_node
  ${catkin_LIBRARIES}
)
