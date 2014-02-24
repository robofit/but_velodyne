cmake_minimum_required(VERSION 2.8.3)
project(but_env_model_msgs)

find_package(catkin 
  REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_srvs
  moveit_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  RVIZCameraPosition.msg
  PlaneDesc.msg
  PlaneArray.msg
  OctomapUpdates.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  AddCube.srv
  GetObjectsInSphere.srv
  OctomapReset.srv
  AddPlanes.srv
  GetPlane.srv
  Plane.srv
  EnvModelLock.srv
  GetTreeDepth.srv
  RemoveCube.srv
  EnvModelPause.srv
  InsertAlignedBox.srv
  RemoveObject.srv
  EnvModelReset.srv
  InsertBoundingBox.srv
  SetCrawlingDepth.srv
  GetAlignedBox.srv
  InsertPlane.srv
  SetNumIncompleteFrames.srv
  GetAllPrimitivesNames.srv
  InsertPlanes.srv
  ShowObject.srv
  GetBoundingBox.srv
  IsNewCollisionMap.srv
  ShowObjtree.srv
#  GetCollisionMap.srv
  LoadSave.srv
  UseInputColor.srv
  GetObjectsInBox.srv
  LockCollisionMap.srv
  GetObjectsInHalfspace.srv
  OctomapAgingPause.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_srvs
  moveit_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs std_srvs moveit_msgs)
