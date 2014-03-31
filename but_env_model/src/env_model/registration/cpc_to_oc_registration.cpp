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

#include <but_env_model/registration/cpc_to_oc_registration.h>
#include <but_env_model/registration/pcl_registration_module.h>
#include <but_env_model/topics_list.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>


/**
 * Constructor
 */
but_env_model::COcPatchMaker::COcPatchMaker()
  : m_bTransformCamera(false)
  , m_pcFrameId("/map")
  , m_bSpinThread(false)
  , m_bCamModelInitialized(false)
  , m_camera_stereo_offset_left(0)
  , m_camera_stereo_offset_right(0)
  , m_bPublishCloud(false)
  , m_fracX(1.5)
  , m_fracY(1.5)
{

}

/**
 * Initialize module - called in server constructor
 */
void but_env_model::COcPatchMaker::init(ros::NodeHandle & node_handle)
{
  ROS_DEBUG("Initializing CCompressedPointCloudPlugin");

  if (m_bSpinThread)
  {
    // if we're spinning our own thread, we'll also need our own callback queue
    node_handle.setCallbackQueue(&callback_queue_);

    need_to_terminate_ = false;
    spin_thread_.reset(new boost::thread(boost::bind(&COcPatchMaker::spinThread, this)));
    node_handle_ = node_handle;
  }

  // Read parameters
  {
    // Where to get camera position information
    node_handle.param("camera_info_topic_name", m_cameraInfoTopic, CPC_CAMERA_INFO_PUBLISHER_NAME);
  }


  // Subscribe to position topic
  // Create subscriber
  m_camPosSubscriber = node_handle.subscribe<sensor_msgs::CameraInfo>(m_cameraInfoTopic, 10, &but_env_model::COcPatchMaker::onCameraChangedCB, this);

  if (!m_camPosSubscriber)
  {
    ROS_ERROR("Not subscribed...");
    ROS_ERROR("Cannot subscribe to the camera position publisher...");
  }

  // stereo cam params for sensor cone:
//  node_handle.param<int> ("compressed_pc_camera_stereo_offset_left", m_camera_stereo_offset_left, 0); // 128
//  node_handle.param<int> ("compressed_pc_camera_stereo_offset_right", m_camera_stereo_offset_right, 0);

  node_handle.param("registration_patch_view_fraction_x", m_fracX, m_fracX);
  node_handle.param("registration_patch_view_fraction_y", m_fracY, m_fracY);

  node_handle.param("registration_patch_publish_cloud", m_bPublishCloud, m_bPublishCloud);

  if (m_bPublishCloud)
    // Create publisher - simple point cloud
    m_pubConstrainedPC = node_handle.advertise<sensor_msgs::PointCloud2> (REGISTRATION_CONSTRAINED_CLOUD_PUBLISHER_NAME, 5, false);

}

/**
 * Get output pointcloud
 */
bool but_env_model::COcPatchMaker::computeCloud(const SMapWithParameters & par, const ros::Time & time)
{
  ROS_DEBUG("CCompressedPointCloudPlugin: onFrameStart");

  // Copy buffered camera normal and d parameter
  boost::recursive_mutex::scoped_lock lock(m_camPosMutex);

  // Clear data
  m_cloud.clear();
  m_ocFrameId = par.frameId;
  m_DataTimeStamp = m_time_stamp = time;

  bool bTransformOutput = m_ocFrameId != m_pcFrameId;

  // Output transform matrix
  Eigen::Matrix4f pcOutTM;

  // If different frame id
  if (bTransformOutput)
  {
    tf::StampedTransform ocToPcTf;

    // Get transform
    try
    {
      // Transformation - to, from, time, waiting time
      m_tfListener.waitForTransform(m_pcFrameId, m_ocFrameId,
                                    m_time_stamp, ros::Duration(5));

      m_tfListener.lookupTransform(m_pcFrameId, m_ocFrameId,
                                   m_time_stamp, ocToPcTf);

    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
      ROS_ERROR_STREAM("Transform error.");
      return false;
    }


    // Get transformation matrix
    pcl_ros::transformAsMatrix(ocToPcTf, pcOutTM);  // Sensor TF to defined base TF

  }

  if (m_cameraFrameId.size() == 0)
  {
    ROS_ERROR_STREAM("Wrong camera frame id...");
    m_bTransformCamera = false;
    return false;
  }

  m_bTransformCamera = m_cameraFrameId != m_ocFrameId;


  // Store camera information
  m_camera_size = m_camera_size_buffer;
  m_camera_model.fromCameraInfo(m_camera_info_buffer);

  // Transform camera model to the cloud time and frame id
  {
    tf::StampedTransform camTf;

    // Modify camera position to the current time
    if (m_bTransformCamera)
    {
      // We need camera to octomap transfor
      try
      {
        // Transformation - to, from, time, waiting time
        //  std::cerr << "T1, try to get transform from " << m_cameraFrameId << " to " << m_ocFrameId << ", time: " << par.currentTime << std::endl;
        m_tfListener.waitForTransform(m_ocFrameId, m_cameraFrameId,
                                      m_time_stamp, ros::Duration(5.0));

        m_tfListener.lookupTransform(m_ocFrameId, m_cameraFrameId,
                                     m_time_stamp, camTf);

      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error1: " << ex.what() << ", quitting callback");
        return false;
      }

    }
    else
    {
      // Just move camera position "in time"
      try
      {
        // Time transformation - target frame, target time, source frame, source time, fixed frame, time, waiting time
        m_tfListener.waitForTransform(m_cameraFrameId, m_time_stamp,
                                      m_cameraFrameId, m_camera_info_buffer.header.stamp,
                                      "/map", ros::Duration(1.0));

        m_tfListener.lookupTransform(m_cameraFrameId, m_time_stamp,
                                     m_cameraFrameId, m_camera_info_buffer.header.stamp,
                                     "/map", camTf);

      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error2: " << ex.what() << ", quitting callback");
        return false;
      }
    }

    // We have transform so convert it to the eigen matrix
    m_to_sensorTf = camTf.inverse();
  }

  // Initialize leaf iterators
  tButServerOcTree & tree(par.map->getTree());
  but_env_model::tButServerOcTree::leaf_iterator it, itEnd(tree.end_leafs());

  // Compute fraction matrix
  m_fracMatrix = tf::Matrix3x3::getIdentity().scaled(tf::Point(1.0 / m_fracX, 1.0 / m_fracY, 1.0));

  // Crawl through nodes
  for (it = tree.begin_leafs(par.treeDepth); it != itEnd; ++it)
  {
    // Node is occupied?
    if (tree.isNodeOccupied(*it))
    {
      handleOccupiedNode(it, par);
    }// Node is occupied?

  } // Iterate through octree

  if (bTransformOutput)
  {
    // transform point cloud from octomap frame to the preset frame
    pcl::transformPointCloud< tPclPoint >(m_cloud, m_cloud, pcOutTM);
  }

  if (m_bPublishCloud)
    publishInternal(m_DataTimeStamp);

  return true;
}

/**
 * hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
 */
void but_env_model::COcPatchMaker::handleOccupiedNode(but_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
{
//  PERROR("OnHandleOccupied");

  if (! m_bCamModelInitialized)
    return;

  // Test input point
  tf::Point pos(it.getX(), it.getY(), it.getZ());
//  if( m_bTransformCamera )
  pos = m_to_sensorTf(pos);

  pos = pos * m_fracMatrix;

  if (pos.z() < 0)
    return;

  cv::Point2d uv = m_camera_model.project3dToPixel(cv::Point3d(pos.x(), pos.y(), pos.z()));

  // ignore point if not in sensor cone
  if (!inSensorCone(uv))
    return;

  // Ok, add it...
  //  std::cerr << "PCP: handle occupied" << std::endl;
  tPclPoint point;

  // Set position
  point.x = it.getX();
  point.y = it.getY();
  point.z = it.getZ();

  // Set color
  point.r = it->r();
  point.g = it->g();
  point.b = it->b();

//  std::cerr << "Occupied node r " << (int)point.r << ", g " << (int)point.g << ", b " << (int)point.b << std::endl;

  m_cloud.push_back(point);
}

/**
 * On camera position changed callback
 */
void but_env_model::COcPatchMaker::onCameraChangedCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{
  boost::recursive_mutex::scoped_lock lock(m_camPosMutex);

  //PERROR("OnCameraChange.");

  //  ROS_DEBUG( "CCompressedPointCloudPlugin: onCameraChangedCB" );

  // Set camera position frame id
  m_cameraFrameId = cam_info->header.frame_id;

  ROS_DEBUG("COcToPcl: Set camera info: %d x %d\n", cam_info->height, cam_info->width);
  m_camera_model_buffer.fromCameraInfo(*cam_info);
  m_camera_size_buffer = m_camera_model_buffer.fullResolution();

  // Set flag
  m_bCamModelInitialized = true;

  m_camera_info_buffer = *cam_info;
}


/**
 * Test if point is in camera cone
 */
bool but_env_model::COcPatchMaker::inSensorCone(const cv::Point2d& uv) const
{
  const double x_offset(0.0), y_offset(0.0);

  //PERROR( uv.y << " > " << 1 << " && " << uv.y << " < " << m_camera_size.height - 2 );
  // Check if projected 2D coordinate in pixel range.
  // This check is a little more restrictive than it should be by using
  // 1 pixel less to account for rounding / discretization errors.
  // Otherwise points on the corner are accounted to be in the sensor cone.
  bool rv((uv.x > m_camera_stereo_offset_left + 1 - x_offset) &&
          (uv.x < m_camera_size.width + m_camera_stereo_offset_right - 2 + x_offset) &&
          (uv.y > 1 - y_offset) &&
          (uv.y < m_camera_size.height - 2 + y_offset));

  if (! rv)
  {
//      std::cerr << uv.x << ", "  << uv.y << " > " << m_camera_size.width << ", "  << m_camera_size.height << std::endl;
  }

  return rv;
}

/**
 * Main loop when spinning our own thread - process callbacks in our callback queue - process pending goals
 */
void but_env_model::COcPatchMaker::spinThread()
{
  while (node_handle_.ok())
  {
    if (need_to_terminate_)
    {
      break;
    }
    callback_queue_.callAvailable(ros::WallDuration(0.033f));
  }
}

/**
 * Called when new scan was inserted and now all can be published
 */
void but_env_model::COcPatchMaker::publishInternal(const ros::Time & timestamp)
{
  // Convert data
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg< tPclPoint >(m_cloud, msg);

  // Set message parameters and publish
  msg.header.frame_id = m_pcFrameId;
  msg.header.stamp = m_cloud.header.stamp;

  // Publish data
  m_pubConstrainedPC.publish(msg);
}

//=============================================================================
//  CPcToOcRegistration
//=============================================================================

/**
 * Constructor
 */
but_env_model::CPcToOcRegistration::CPcToOcRegistration()
  : m_resampledCloud(new sensor_msgs::PointCloud2())
{

}

/**
 *  Initialize plugin - called in server constructor
 */
void but_env_model::CPcToOcRegistration::init(ros::NodeHandle & node_handle)
{
  // Initialize modules
  m_patchMaker.init(node_handle);
  m_registration.init(node_handle);
}

/**
 * Register cloud to the octomap
 */
bool but_env_model::CPcToOcRegistration::registerCloud(tPointCloudPtr & cloud, const SMapWithParameters & map)
{
  if (!m_registration.isRegistering())
  {
    ROS_ERROR("No registration method selected.");
    return false;
  }

//  std::cerr << "Reg start" << std::endl;

  // Get patch
  m_patchMaker.setCloudFrameId(map.frameId);
  m_patchMaker.computeCloud(map, cloud->header.stamp);

  if (m_patchMaker.getCloud().size() == 0)
    return false;

  // Resample input cloud
  sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2());

  pcl::toROSMsg(*cloud, *cloudMsg);
//  std::cerr << "Patch size: " << m_patchMaker.getCloud().size() << ", " << cloudMsg->data.size() << std::endl;

  m_gridFilter.setInputCloud(cloudMsg);
  m_gridFilter.setLeafSize(map.resolution, map.resolution, map.resolution);
  m_gridFilter.filter(*m_resampledCloud);

//  std::cerr << "Voxel grid size: " << m_resampledCloud->data.size() << std::endl;

  // Try to register cloud
  tPointCloudPtr cloudSource(new tPointCloud());
  tPointCloudPtr cloudBuffer(new tPointCloud());
  tPointCloudPtr cloudTarget(new tPointCloud());
  pcl::fromROSMsg(*m_resampledCloud, *cloudSource);
  pcl::copyPointCloud(m_patchMaker.getCloud(), *cloudTarget);

//  std::cerr << "Calling registration: " << cloudSource->size() << ", " << cloudTarget->size() << std::endl;

  bool rv(m_registration.process(cloudSource, cloudTarget, cloudBuffer));

  if (! rv)
    std::cerr << "Registration failed" << std::endl;

  return rv;
}
