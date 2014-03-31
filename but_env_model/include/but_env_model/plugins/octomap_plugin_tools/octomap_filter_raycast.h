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
 * Date: dd/mm/2012
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
#ifndef OCTOMAP_FILTER_RAYCAST_H_
#define OCTOMAP_FILTER_RAYCAST_H_

#include "octomap_filter_base.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/filters/voxel_grid.h>

namespace but_env_model
{

class COcFilterRaycast : public COcTreeFilterBase
{
public:
  //! Constructor
  COcFilterRaycast(const std::string & octree_frame_id, ERunMode mode = FILTER_ALLWAYS);

  //! Initialize. Must be called before first filtering
  virtual void init(ros::NodeHandle & node_handle);

  //! Configure filter before each frame. Set input cloud.
  void setCloud(tPointCloudConstPtr cloud);

  //! Write some info about last filter run
  virtual void writeLastRunInfo();

protected:
  //! Filtering function implementation
  virtual void filterInternal(tButServerOcTree & tree);

  /// Camera info callback
  void cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

  /// Compute sensor origin from the header info
  octomap::point3d getSensorOrigin(const std_msgs::Header& sensor_header);

  /// Is point in sensor cone?
  bool inSensorCone(const cv::Point2d& uv) const;

  /// Return true, if occupied cell is between origin and p
  bool isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p, double resolution, tButServerOcTree & tree) const;

  /// Return true if point is occluded by pointcloud
  bool isOccludedRaw(const cv::Point2d& uv, double range);

  //! Compute boundig box
  void computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max);

protected:
  //! Sensor frame id
  std_msgs::Header m_sensor_header;

  //! Initialized
  bool m_bFilterInitialized;

  //! Transform listener
  tf::TransformListener m_tfListener;

  /// Camera offsets
  int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

  /// Camera size
  cv::Size m_camera_size;

  /// Camera model
  image_geometry::PinholeCameraModel m_camera_model;

  /// Is camera model initialized?
  bool m_bCamModelInitialized;

  /// Camera info locking
  boost::mutex m_lockCamera;

  /// Input cloud lock
  boost::mutex m_lockData;

  /// Camera info topic name
  std::string m_camera_info_topic;

  /// Camera info subscriber
  ros::Subscriber m_ciSubscriber;

  /// Point cloud
  tPointCloudConstPtr m_cloudPtr;

  /// Number of removed leafs
  long m_numLeafsRemoved;

  /// Number of leafs out of sensor cone
  long m_numLeafsOutOfCone;

  /// Number of leafs out of raw map
  long m_numLeafsOutOfMap;

  /// Num of tested leafs
  long m_numLeafsTested;

  octomap::point3d m_sensor_origin;

  //! Visualizations marker publisher
  ros::Publisher marker_pub_;

  //! Grid points publisher
  ros::Publisher grid_pub_;

  //! Use voxel filter to downsample pointcloud
  pcl::VoxelGrid<tPclPoint> m_vgfilter;

  //! Output filtered cloud
  tPointCloud::Ptr m_filtered_cloud;

  /// Speedup of integrate miss no time function
  float m_miss_speedup;

  /// Maximal sensor range
  double m_max_sensor_range;

  /// Tree resolution
  float m_tree_resolution;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class COcFilterRaycast

} // namespace but_env_model

#endif /* OCTOMAP_FILTER_RAYCAST_H_ */
