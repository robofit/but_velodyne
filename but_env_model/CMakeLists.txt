cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

find_package( Eigen REQUIRED )
include_directories( ${EIGEN_INCLUDE_DIRS} )
add_definitions( ${EIGEN_DEFINITIONS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#target_link_libraries( my_awesome_library ${OpenCV_LIBRARIES} )

find_package( octomap REQUIRED )
include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_libraries( ${OCTOMAP_LIBRARIES} )

rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# BUT env. model server
set( SERVER_SOURCES src/env_model/env_model_server.cpp
                    src/env_model/server_tools.cpp
                    src/env_model/octonode.cpp
                    src/env_model/plugins/collision_map_plugin.cpp
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
                    src/env_model/registration/cpc_to_oc_registration.cpp
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
rosbuild_add_library( ${OBJTREE_LIB_NAME} ${OBJTREE_SOURCE_FILES} )

# Environment model library
set( SRS_ENV_MODEL_LIB_NAME but_env_model )
rosbuild_add_library( ${SRS_ENV_MODEL_LIB_NAME} ${SERVER_SOURCES} )

# Add environment model server executable
rosbuild_add_executable( but_env_model_node ${SERVER_SOURCES} src/server_node.cpp )
target_link_libraries( but_env_model_node octomap_ros ${OBJTREE_LIB_NAME} )
target_link_libraries( but_env_model_node ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} )


# Add compressed pc subscriber-publisher executable
rosbuild_add_executable( but_cpc_node ${CPC_SOURCES} )
target_link_libraries( but_cpc_node ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} )

