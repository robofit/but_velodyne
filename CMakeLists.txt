cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_definitions (-Wno-missing-field-initializers -O4 -mfpmath=sse -msse2 -fopenmp)
# generate dynamic reconfigure stuff
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_gensrv()


include_directories( include/rt_road_detection )

# Sample hue detector
rosbuild_add_library(sample_hue_detector_ros src/detectors/sample_hue_detector.cpp src/detectors/sample_hue_detector_ros.cpp)

rosbuild_add_executable(sample_hue_detector_node src/nodes/sample_hue_detector_node.cpp)
target_link_libraries(sample_hue_detector_node sample_hue_detector_ros)

#rosbuild_add_library( detector_nodelets src/nodelets/sample_hue_detector_nodelet.cpp)
#target_link_libraries( detector_nodelets sample_hue_detector_ros)

# LBP detector
rosbuild_add_library(lbp_detector_ros src/detectors/lbp_detector.cpp src/detectors/lbp_detector_ros.cpp src/detectors/lbp/lbp.cpp  )

rosbuild_add_executable(lbp_detector_node src/nodes/lbp_detector_node.cpp)
target_link_libraries(lbp_detector_node lbp_detector_ros)


# LBP training tool
rosbuild_add_library(lbp_training_ros src/detectors/lbp_detector.cpp src/detectors/lbp_training_ros.cpp src/detectors/lbp/lbp.cpp  )

rosbuild_add_executable(lbp_training_node src/nodes/lbp_training_node.cpp)
target_link_libraries(lbp_training_node lbp_training_ros)


rosbuild_add_library(traversability_costmap src/costmap.cpp)
rosbuild_add_executable( costmap_node src/nodes/costmap_node.cpp )
target_link_libraries( costmap_node traversability_costmap )

#rosbuild_add_library(traversability_costmap_nodelet src/nodelets/costmap_nodelet.cpp)
#target_link_libraries( traversability_costmap_nodelet traversability_costmap )

rosbuild_add_library(road_detection_nodelets src/nodelets/costmap_nodelet.cpp src/nodelets/sample_hue_detector_nodelet.cpp)
target_link_libraries( road_detection_nodelets sample_hue_detector_ros traversability_costmap)

rosbuild_add_executable(wp_corrector_node src/nodes/waypoint_corrector_main.cpp src/nodes/WaypointCorrector.cpp)

# Parse check all the launch/*.launch files
rosbuild_add_roslaunch_check(launch)
