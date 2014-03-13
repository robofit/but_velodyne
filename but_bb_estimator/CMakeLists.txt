cmake_minimum_required(VERSION 2.8.3)
project(but_bb_estimator)


find_package(catkin REQUIRED COMPONENTS
  message_generation
#  arm_navigation_msgs
  cv_bridge
  message_filters
  roscpp
  sensor_msgs
  pcl_ros
  tf
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(bullet REQUIRED bullet)

find_package( OpenCV REQUIRED )


add_service_files(
  DIRECTORY srv
  FILES
  EstimateBBAlt.srv
  EstimateRectAlt.srv
#  Estimate2DHullMesh.srv
  EstimateBB.srv
  EstimateRect.srv
  Estimate2DHullPointCloud.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS} ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
CATKIN_DEPENDS message_runtime cv_bridge message_filters opencv pcl roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable(bb_estimator_server src/service_server.cpp src/funcs.cpp )
add_executable(bb_estimator_client src/sample_client.cpp)

target_link_libraries(bb_estimator_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(bb_estimator_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_dependencies(bb_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(bb_estimator_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


