cmake_minimum_required(VERSION 2.8.3)
project(but_env_model_msgs)

find_package(catkin 
  REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  RVIZCameraPosition.msg
  PlaneDesc.msg
  PlaneArray.msg
  OctomapUpdates.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs)
