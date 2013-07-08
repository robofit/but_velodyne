cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories( include/rt_road_detection )

# Sample grass detector
rosbuild_add_library(sample_grass_detector src/detectors/sample_grass_detector.cpp)

rosbuild_add_executable(sample_grass_detector_node src/nodes/sample_grass_detector_node.cpp)
target_link_libraries(sample_grass_detector_node sample_grass_detector)

rosbuild_add_library(sample_grass_detector_nodelet src/nodelets/sample_grass_detector_nodelet.cpp)
target_link_libraries( sample_grass_detector_nodelet sample_grass_detector )

rosbuild_add_library(traversability_costmap src/costmap.cpp)
rosbuild_add_executable( costmap_node src/nodes/costmap_node.cpp )
target_link_libraries( costmap_node traversability_costmap )

# Parse check all the launch/*.launch files
rosbuild_add_roslaunch_check(launch)
