cmake_minimum_required(VERSION 2.8.3)
project(justina_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  navig_msgs
  pcl_conversions
  point_cloud_manager
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  pcl_conversions
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES justina_tools
  CATKIN_DEPENDS geometry_msgs nav_msgs navig_msgs point_cloud_manager roscpp rospy roslib sensor_msgs std_msgs pcl_conversions std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(justina_tools
  src/JustinaHardware.cpp
  src/JustinaNavigation.cpp
  src/JustinaTools.cpp
  src/JustinaHRI.cpp
)

add_dependencies(justina_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(justina_tools
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
