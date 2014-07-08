cmake_minimum_required(VERSION 2.8.3)
project(but_calibration_camera_velodyne)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g")

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  cv_bridge
  image_transport
  image_geometry
  pcl_ros
  velodyne_pointcloud
  roslint
)

catkin_package(CATKIN_DEPENDS std_msgs )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS}) 

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(calibration src/calibration-node.cpp src/Image.cpp src/Velodyne.cpp src/Similarity.cpp src/Calibration3DMarker.cpp ) 
target_link_libraries(calibration ${PCL_LIBRARIES}) 
target_link_libraries(calibration ${catkin_LIBRARIES})

add_executable(coloring src/coloring-node.cpp src/Image.cpp src/Velodyne.cpp )
target_link_libraries(coloring ${PCL_LIBRARIES}) 
target_link_libraries(coloring ${catkin_LIBRARIES})

roslint_cpp()

