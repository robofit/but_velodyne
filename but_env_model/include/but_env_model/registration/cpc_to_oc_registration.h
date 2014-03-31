/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 25/1/2012
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#ifndef CPCTOOCREGISTRATION_H_
#define CPCTOOCREGISTRATION_H_

#include <sensor_msgs/CameraInfo.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/node_handle.h>

#include <but_env_model/server_tools.h>

#include "pcl_registration_module.h"

#include <pcl/filters/voxel_grid.h>

namespace but_env_model
{
/**
 * Get visible pointcloud from octomap module
 */
class COcPatchMaker
{
public:
  //! Used cloud type
  //typedef sensor_msgs::PointCloud2 tCloud;
  typedef tPointCloud tCloud;

public:
  //! Constructor
  COcPatchMaker();

  //! Initialize plugin - called in server constructor
  virtual void init(ros::NodeHandle & node_handle);

  //! Get output pointcloud
  bool computeCloud(const SMapWithParameters & par, const ros::Time & time);

  //! Get cloud
  tCloud & getCloud()
  {
    return m_cloud;
  }

  //! Set output cloud frame id
  void setCloudFrameId(const std::string & fid)
  {
    m_pcFrameId = fid;
  }

protected:
  /// On camera position changed callback
  void onCameraChangedCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

  // main loop when spinning our own thread
  // - process callbacks in our callback queue
  // - process pending goals
  void spinThread();

  //! Test if point is in camera cone
  bool inSensorCone(const cv::Point2d& uv) const;

  //! Called when new scan was inserted and now all can be published
  void publishInternal(const ros::Time & timestamp);

  /// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
  virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapWithParameters & mp);

protected:
  /// Should camera position and orientation be transformed?
  bool m_bTransformCamera;

  /// Camera frame id
  std::string m_cameraFrameId;

  //! Output frame id
  std::string m_pcFrameId;

  // Camera position topic name
  std::string m_cameraInfoTopic;

  /// Subscriber - camera position
  ros::Subscriber m_camPosSubscriber;

  // Mutex used to lock camera position parameters
  boost::recursive_mutex m_camPosMutex;

  //! Spin out own input callback thread
  bool m_bSpinThread;

  // these are needed when spinning up a dedicated thread
  boost::scoped_ptr<boost::thread> spin_thread_;
  ros::NodeHandle node_handle_;
  ros::CallbackQueue callback_queue_;
  volatile bool need_to_terminate_;

  /// Is camera model initialized?
  bool m_bCamModelInitialized;

  /// Camera offsets
  int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

  /// Output point cloud data
  tCloud m_cloud;

  //! Should i publish pointcloud
  bool m_bPublishCloud;

  /// Camera model
  image_geometry::PinholeCameraModel m_camera_model, m_camera_model_buffer;

  //! Camera info buffer
  sensor_msgs::CameraInfo m_camera_info_buffer;

  /// Camera size
  cv::Size m_camera_size, m_camera_size_buffer;

  //! Transform listener
  tf::TransformListener m_tfListener;

  //! Cloud publishers
  ros::Publisher m_pubConstrainedPC;

  /// Crawled octomap frame id
  std::string m_ocFrameId;

  /// Time stamp
  ros::Time m_DataTimeStamp, m_time_stamp;

  /// PC to sensor transformation
  tf::Transform m_to_sensorTf;

  //! Fraction of the field of view taken from the octomap (x-direction)
  double m_fracX;

  //! Fraction of the field of view taken from the octomap (y-direction)
  double m_fracY;

  //! View fraction computation matrix
  tf::Matrix3x3 m_fracMatrix;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class COcToPcl


/**
 * Registrates incomming point cloud to the octomap
 */
class CPcToOcRegistration
{
public:
  //! Used cloud
  typedef sensor_msgs::PointCloud2 tCloud;

  //! Registration module type
  typedef CPclRegistration< tPclPoint, tPclPoint> tRegistrator;

public:
  //! Constructor
  CPcToOcRegistration();

  //! Initialize plugin - called in server constructor
  virtual void init(ros::NodeHandle & node_handle);

  //! Register cloud to the octomap
  bool registerCloud(tPointCloudPtr & cloud, const SMapWithParameters & map);

  //! Get transform
  Eigen::Matrix4f getTransform()
  {
    return m_registration.getTransform();
  }

  //! Is some registering mode set
  bool isRegistering()
  {
    return m_registration.isRegistering();
  }

protected:
  //! Patch maker
  COcPatchMaker m_patchMaker;

  //! Registration module
  tRegistrator m_registration;

  //! Voxel grid filter
  //pcl::VoxelGrid<tCloud> m_gridFilter;

  //! Cloud buffer
  tCloud::Ptr m_resampledCloud;

  //! Transform listener
  tf::TransformListener m_tfListener;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class CPcToOcRegistration


} // namespace but_env_model

#endif /* CPCTOOCREGISTRATION_H_ */
