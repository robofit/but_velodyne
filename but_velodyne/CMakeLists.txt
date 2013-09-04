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
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

# BUT Velodyne processing library
rosbuild_add_library( butvelo 
                      src/but_velodyne_lib/laser_scan.cpp
                      src/but_velodyne_lib/ground_map.cpp )
#rosbuild_add_openmp_flags( butvelo )


# Simulated laser scan nodelet
rosbuild_add_library( butvelo_laserscan_nodelet src/laser_scan_nodelet.cpp )
target_link_libraries( butvelo_laserscan_nodelet butvelo )

# Simulated laser scan node
rosbuild_add_executable( laser_scan src/laser_scan_node.cpp )
#rosbuild_add_openmp_flags( laser_scan )
target_link_libraries( laser_scan butvelo )


# Ground map nodelet
rosbuild_add_library( butvelo_groundmap_nodelet src/ground_map_nodelet.cpp )
target_link_libraries( butvelo_groundmap_nodelet butvelo )

# Ground map node
rosbuild_add_executable( ground_map src/ground_map_node.cpp )
#rosbuild_add_openmp_flags( ground_map )
target_link_libraries( ground_map butvelo )


# Parse check all the launch/*.launch files
rosbuild_add_roslaunch_check(launch)
