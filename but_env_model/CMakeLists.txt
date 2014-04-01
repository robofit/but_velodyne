cmake_minimum_required(VERSION 2.8.3)
project(but_env_model)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  image_geometry
  camera_calibration_parsers
  std_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  interactive_markers
  actionlib
  actionlib_msgs
  shape_msgs
  moveit_msgs
  pcl_ros
  pcl_conversions
  interactive_markers
  nodelet
  but_env_model_msgs
  but_interaction_primitives
  message_filters
  octomap_ros
  roslint
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE OGRE)
include_directories(include ${OGRE_INCLUDE_DIRS})

find_package( Eigen REQUIRED )
include_directories( ${EIGEN_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARIES})

catkin_package(CATKIN_DEPENDS std_msgs std_msgs visualization_msgs actionlib_msgs shape_msgs std_srvs nav_msgs moveit_msgs but_env_model_msgs but_interaction_primitives message_filters pcl_ros)
include_directories(include ${catkin_INCLUDE_DIRS})

# BUT env. model server
set( SERVER_SOURCES src/env_model/env_model_server.cpp
                    src/env_model/server_tools.cpp
                    src/env_model/octonode.cpp
#                    src/env_model/plugins/collision_map_plugin.cpp
                    src/env_model/plugins/octomap_plugin.cpp
                    src/env_model/plugins/point_cloud_plugin.cpp
                    src/env_model/plugins/collision_object_plugin.cpp
                    src/env_model/plugins/occupancy_grid_plugin.cpp
                    src/env_model/plugins/imarkers_plugin.cpp
                    src/env_model/plugins/marker_array_plugin.cpp
                    src/env_model/plugins/limited_point_cloud_plugin.cpp
                    src/env_model/plugins/compressed_point_cloud_plugin.cpp
                    src/env_model/plugins/objtree_plugin.cpp
                    src/env_model/plugins/octomap_plugin_tools/testing_oriented_box.cpp
                    src/env_model/plugins/octomap_plugin_tools/testing_planes.cpp
                    src/env_model/plugins/octomap_plugin_tools/testing_polymesh.cpp
                    src/env_model/plugins/octomap_plugin_tools/octomap_filter_base.cpp
                    src/env_model/plugins/octomap_plugin_tools/octomap_filter_raycast.cpp
                    src/env_model/plugins/octomap_plugin_tools/octomap_filter_ground.cpp
#                    src/env_model/registration/cpc_to_oc_registration.cpp
                    )
                    
set( CPC_SOURCES src/env_model/compressed_pc_publisher.cpp
                 src/cpc_node.cpp
                 )

set( OBJTREE_SOURCE_FILES src/env_model/objtree/bbox.cpp
                          src/env_model/objtree/gbbox.cpp
                          src/env_model/objtree/filter.cpp
                          src/env_model/objtree/node.cpp
                          src/env_model/objtree/object.cpp
                          src/env_model/objtree/octree.cpp
                          src/env_model/objtree/plane.cpp
                          src/env_model/objtree/history.cpp
                          )

# Objtree library
set( OBJTREE_LIB_NAME but_objtree )
add_library( ${OBJTREE_LIB_NAME} ${OBJTREE_SOURCE_FILES} )
target_link_libraries(${OBJTREE_LIB_NAME} ${catkin_LIBRARIES} ${SRS_ENV_MODEL_LIB_NAME} ${OBJTREE_LIB_NAME} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# Environment model library
set( SRS_ENV_MODEL_LIB_NAME but_env_model )
add_library( ${SRS_ENV_MODEL_LIB_NAME} ${SERVER_SOURCES} )
target_link_libraries(${SRS_ENV_MODEL_LIB_NAME} ${catkin_LIBRARIES} ${SRS_ENV_MODEL_LIB_NAME} ${OBJTREE_LIB_NAME} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

#add_dependencies(${SRS_ENV_MODEL_LIB_NAME} message_filters pcl_ros)

# Add environment model server executable
add_executable( but_env_model_node src/server_node.cpp )
target_link_libraries( but_env_model_node ${SRS_ENV_MODEL_LIB_NAME} ${OBJTREE_LIB_NAME})

# Environment model server nodelet
add_library( but_env_model_nodelet src/server_nodelet.cpp )
target_link_libraries( but_env_model_nodelet ${SRS_ENV_MODEL_LIB_NAME} ${OBJTREE_LIB_NAME})

# Add compressed pc subscriber-publisher executable
add_executable( but_cpc_node ${CPC_SOURCES} )
target_link_libraries( but_cpc_node ${SRS_ENV_MODEL_LIB_NAME} ${OBJTREE_LIB_NAME})

# TODO install

roslint_cpp()

