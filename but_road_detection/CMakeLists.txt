cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

cmake_minimum_required(VERSION 2.8.3)
project(but_road_detection)

find_package(catkin 
  REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
  nodelet
  message_filters
  image_geometry
  stereo_msgs
  nav_msgs
  tf
  dynamic_reconfigure
  laser_geometry
  roslint
  rostest
)

# generate service
add_service_files(
  DIRECTORY srv
  FILES
  getCorrectedWaypoint.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/hsv_hist_detector.cfg
  cfg/lbp_detector.cfg
  cfg/sample_hue_detector.cfg
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs sensor_msgs stereo_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(sample_hue_detector_ros src/detectors/sample_hue_detector.cpp src/detectors/sample_hue_detector_ros.cpp)
add_dependencies(sample_hue_detector_ros ${PROJECT_NAME}_gencfg)
target_link_libraries(sample_hue_detector_ros ${catkin_LIBRARIES})
add_executable(sample_hue_detector_node src/nodes/sample_hue_detector_node.cpp)
target_link_libraries(sample_hue_detector_node sample_hue_detector_ros)

add_library(lbp_detector_ros src/detectors/lbp_detector.cpp src/detectors/lbp_detector_ros.cpp src/detectors/lbp/lbp.cpp  )
add_dependencies(lbp_detector_ros ${PROJECT_NAME}_gencfg)
target_link_libraries(lbp_detector_ros ${catkin_LIBRARIES})
add_executable(lbp_detector_node src/nodes/lbp_detector_node.cpp)
target_link_libraries(lbp_detector_node lbp_detector_ros)

add_library(lbp_training_ros src/detectors/lbp_detector.cpp src/detectors/lbp_training_ros.cpp src/detectors/lbp/lbp.cpp  )
target_link_libraries(lbp_training_ros ${catkin_LIBRARIES})
add_executable(lbp_training_node src/nodes/lbp_training_node.cpp)
target_link_libraries(lbp_training_node lbp_training_ros)

add_library(hsv_hist_detector_ros src/detectors/hsv_hist_detector.cpp src/detectors/hsv_hist_detector_ros.cpp)
add_dependencies(hsv_hist_detector_ros ${PROJECT_NAME}_gencfg)
target_link_libraries(hsv_hist_detector_ros ${catkin_LIBRARIES})

add_executable(hsv_hist_detector_node src/nodes/hsv_hist_detector_node.cpp)
target_link_libraries(hsv_hist_detector_node hsv_hist_detector_ros)

add_executable(hsv_hist_detector_train src/detectors/hsv_hist_detector.cpp src/detectors/hsv_hist_detector_train.cpp)
add_dependencies(hsv_hist_detector_train ${PROJECT_NAME}_gencfg)
target_link_libraries(hsv_hist_detector_train ${catkin_LIBRARIES})

add_library(traversability_costmap src/costmap.cpp)
add_dependencies(traversability_costmap ${PROJECT_NAME}_gencfg)
target_link_libraries(traversability_costmap ${catkin_LIBRARIES})
add_dependencies(traversability_costmap ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable( costmap_node src/nodes/costmap_node.cpp )
target_link_libraries( costmap_node traversability_costmap )

add_executable(wp_corrector_node src/nodes/waypoint_corrector_main.cpp src/nodes/WaypointCorrector.cpp)
target_link_libraries(wp_corrector_node ${catkin_LIBRARIES})
add_dependencies(wp_corrector_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# TODO add nodeletes

roslint_cpp()

