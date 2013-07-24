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
#ifndef COMPRESSED_PC_PUBLISHER_H_INCLUDED
#define COMPRESSED_PC_PUBLISHER_H_INCLUDED

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <but_env_model_msgs/OctomapUpdates.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>


namespace but_env_model
{

/**
 * publisher class
 */
class CCompressedPCPublisher
{
public:
	//! Define pcl point type
	typedef pcl::PointXYZRGB tPclPoint;

	//! Define internal point cloud type
	typedef pcl::PointCloud<tPclPoint> tPointCloudInternal;

	//! Input data type
	typedef but_env_model_msgs::OctomapUpdates tInputData;

	//! Incoming point cloud type
	typedef tInputData::_pointcloud2_type tInputCloudMsg;

	//! Output point cloud message type
	typedef sensor_msgs::PointCloud2 tOutputCloudMsg;

	//! Camera information type
	typedef tInputData::_camera_info_type tCameraInfo;

public:
	//! Constructor
	CCompressedPCPublisher(ros::NodeHandle & nh);

protected:
	//! Input message cb
	void incommingDataCB( const tInputData::ConstPtr & data );

	//! Copy point cloud from msg to internal type variable
	long copyCloud( const tInputCloudMsg & input, tPointCloudInternal & output, bool bAdd = false );

	//! Remove part of the cloud
	long removeFrustumFromCloud( const tInputData::ConstPtr & data, tPointCloudInternal & cloud );

	/// Test if incomming pointcloud2 has rgb part
    bool isRGBCloud( const tInputCloudMsg & cloud );

    /// Test if point is in sensor cone
    bool inSensorCone(const cv::Point2d& uv) const;

protected:
	//! Published topic name
	std::string m_publishedTopicName;

	//! Publisher
	ros::Publisher m_pub;

	//! Subscribed topic name
	std::string m_subscribedTopicName;

	//! Subscriber
//	ros::Subscriber m_sub;
	message_filters::Subscriber<tInputData> m_sub;
	tf::MessageFilter<tInputData> * m_tf_filter;

	//! Stored point cloud
	tPointCloudInternal m_cloud;

	/// Camera offsets
	int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

	/// Camera size
	cv::Size m_camera_size;

	/// Camera frame id
	std::string m_cameraFrameId;

	/// Camera model
	image_geometry::PinholeCameraModel m_camera_model;

	/// Transform from pointcloud to camera space
	tf::Transform m_to_sensor;

	/// Transform from incomming pc to publishing frame id
	tf::Transform m_to_pfid;

	/// Publishing frame id
	std::string m_publishFID, m_cameraFID;

	//! Transform listener
	tf::TransformListener m_tfListener, m_tfListener2;

public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace but_env_model

// COMPRESSED_PC_PUBLISHER_H_INCLUDED
#endif

