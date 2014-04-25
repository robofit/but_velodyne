cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(but_calibration_camera_velodyne)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  cv_bridge
  image_transport
  image_geometry
  pcl_ros
)

catkin_package(CATKIN_DEPENDS std_msgs )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(calibration src/calibration-node.cpp src/Image.cpp src/Velodyne.cpp src/Similarity.cpp src/Calibration3DMarker.cpp ) 
target_link_libraries(calibration ${PCL_LIBRARIES}) 
target_link_libraries(calibration ${catkin_LIBRARIES})

add_executable(coloring src/coloring-node.cpp src/Image.cpp src/Velodyne.cpp )
target_link_libraries(coloring ${PCL_LIBRARIES}) 
target_link_libraries(coloring ${catkin_LIBRARIES})


