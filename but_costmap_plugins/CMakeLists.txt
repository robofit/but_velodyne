cmake_minimum_required(VERSION 2.8.3)
project(but_costmap_plugins)

find_package(catkin 
  REQUIRED COMPONENTS
  roscpp
  nodelet
  nav_msgs
  costmap_2d
  roslint
  rostest
)

catkin_package(CATKIN_DEPENDS nav_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(traversability_layer src/traversability_layer.cpp)
target_link_libraries( traversability_layer ${catkin_LIBRARIES})

roslint_cpp()

