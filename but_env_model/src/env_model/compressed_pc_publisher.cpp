/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
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

#include <srs_env_model/but_server/compressed_pc_publisher.h>
#include <srs_env_model/topics_list.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>


/**
 * Constructor
 */
srs_env_model::CCompressedPCPublisher::CCompressedPCPublisher(ros::NodeHandle & nh)
: m_camera_stereo_offset_left(0)
, m_camera_stereo_offset_right(0)
, m_publishFID(CPC_WORLD_FRAME)
, m_cameraFID(CPC_CAMERA_FRAME)
, m_tfListener()
//, m_tfListener2()
, m_tfListener2(ros::Duration(30))
{
	// Read input parameters via the private node handle
	ros::NodeHandle pnh("~");

	// Point cloud subscribing topic name
//	pnh.param("octomap_updates_publisher", m_subscribedTopicName, CPC_INPUT_TOPIC_NAME );
	m_subscribedTopicName = CPC_INPUT_TOPIC_NAME;

	// Point cloud publishing topic name
//	pnh.param("output_topic_name", m_publishedTopicName, CPC_OUTPUT_TOPIC_NAME);
	m_publishedTopicName = CPC_OUTPUT_TOPIC_NAME;

	// TF frames
	pnh.param("world_frame", m_publishFID, CPC_WORLD_FRAME);
	pnh.param("camera_frame", m_cameraFID, CPC_CAMERA_FRAME);

	// Create publisher
	m_pub = nh.advertise<sensor_msgs::PointCloud2> (m_publishedTopicName, 5);
//	m_pub = nh.advertise<sensor_msgs::PointCloud2> (m_publishedTopicName, 1);

	// Create subscriber
//	m_sub  = nh.subscribe(m_subscribedTopicName, 5, &CCompressedPCPublisher::incommingDataCB, this );
//	m_sub  = nh.subscribe(m_subscribedTopicName, 1, &CCompressedPCPublisher::incommingDataCB, this );
	m_sub.subscribe(nh, m_subscribedTopicName, 5);
	m_tf_filter = new tf::MessageFilter<tInputData>(m_sub, m_tfListener, m_publishFID, 5);
	m_tf_filter->registerCallback( boost::bind(&CCompressedPCPublisher::incommingDataCB, this, _1) );

/*	if (!m_sub)
	{
		ROS_ERROR("Not subscribed...");
	}*/
}

/**
 * Input message cb
 */
void srs_env_model::CCompressedPCPublisher::incommingDataCB( const tInputData::ConstPtr & data )
{
	ROS_DEBUG_ONCE("CCompressedPCPublisher::incommingDataCB: message received");

	// Get transform
	{
		// Some transforms
		tf::StampedTransform pcToPubTf;

		// Get transforms
		try {
			// Transformation - to, from, time, waiting time
//			m_tfListener.waitForTransform(m_publishFID, data->pointcloud2.header.frame_id,
//					data->pointcloud2.header.stamp, ros::Duration(5));

			m_tfListener.lookupTransform( m_publishFID, data->pointcloud2.header.frame_id,
					data->pointcloud2.header.stamp, pcToPubTf );

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM( "CCompressedPCPublisher::incommingDataCB" << ": Transform error - " << ex.what() << ", quitting callback");
			return;
		}

		m_to_pfid = pcToPubTf;
	}

	// Input buffer
	tInputCloudMsg inbuffer;

	// Transform input point cloud to the internally used frame id
	pcl_ros::transformPointCloud( m_publishFID, m_to_pfid, data->pointcloud2, inbuffer );

	if( data->isPartial != 0 )
	{
//		long oldcount( m_cloud.size() ), copied;

		// Remove old data from cloud
//		copied =
		removeFrustumFromCloud( data, m_cloud );

		// Copy rest of the points from the actual point cloud
		copyCloud( inbuffer, m_cloud, true );

//		std::cerr << "Merging. Old size: " << oldcount << ", copied: " << copied << ", removed: " << oldcount - copied << ", current size: " << m_cloud.size() << std::endl;

		// Convert data
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg< tPclPoint >(m_cloud, cloud);

		// Set message parameters and publish
		cloud.header.frame_id = m_publishFID;
		cloud.header.stamp = data->pointcloud2.header.stamp;

		// Publish new full data
		m_pub.publish( cloud );

//		std::cerr << "Published. M_cloud: " << m_cloud.size() << std::endl;
	}
	else
	{
	//	std::cerr << "Complete cloud." << std::endl;

		// Set message parameters and publish
		inbuffer.header.frame_id = m_publishFID;
		inbuffer.header.stamp = data->pointcloud2.header.stamp;

		// Publish new full data
		m_pub.publish( inbuffer );

		// Copy data to the buffer
		m_cloud.clear();
		copyCloud( inbuffer, m_cloud );
	}
}
/**
 * Copy point cloud from msg to internal type variable
 */
long srs_env_model::CCompressedPCPublisher::copyCloud( const tInputCloudMsg & input, tPointCloudInternal & output, bool bAdd /*= false*/  )
{

	if( bAdd )
	{
		tPointCloudInternal buffer;
		output.header = input.header;

		if( ! isRGBCloud( input ) )
		{
			pcl::PointCloud< pcl::PointXYZ >::Ptr bufferCloud( new pcl::PointCloud< pcl::PointXYZ> );

			pcl::fromROSMsg(input, *bufferCloud );
			pcl::copyPointCloud< pcl::PointXYZ, tPclPoint >( *bufferCloud, buffer );

		}
		else
		{
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr bufferCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

			pcl::fromROSMsg(input, *bufferCloud);
			pcl::copyPointCloud<pcl::PointXYZRGB, tPclPoint>( *bufferCloud, buffer );
		}

		// Merge clouds
		output += buffer;

	}
	else
	{
		if( ! isRGBCloud( input ) )
		{
			pcl::PointCloud< pcl::PointXYZ >::Ptr bufferCloud( new pcl::PointCloud< pcl::PointXYZ> );

			pcl::fromROSMsg(input, *bufferCloud );
			pcl::copyPointCloud< pcl::PointXYZ, tPclPoint >( *bufferCloud, output );

		}
		else
		{
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr bufferCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

			pcl::fromROSMsg(input, *bufferCloud);
			pcl::copyPointCloud<pcl::PointXYZRGB, tPclPoint>( *bufferCloud, output );
		}

	}



	return output.size();
}

/**
 * Test if incomming pointcloud2 has rgb part - parameter driven
 */
bool srs_env_model::CCompressedPCPublisher::isRGBCloud( const tInputCloudMsg & cloud )
{
	tInputCloudMsg::_fields_type::const_iterator i, end;

	for( i = cloud.fields.begin(), end = cloud.fields.end(); i != end; ++i )
	{
		if( i->name == "rgb" )
		{
//			PERROR("HAS RGB");
			return true;
		}
	}

//	PERROR("NO RGB");

	return false;
}

/**
 * Remove part of the cloud
 */
long srs_env_model::CCompressedPCPublisher::removeFrustumFromCloud( const tInputData::ConstPtr & data, tPointCloudInternal & cloud )
{
	// Get camera information
	{
		// Set camera position frame id
		m_cameraFrameId = data->camera_info.header.frame_id;

		ROS_DEBUG("CCompressedPCPublisher::removeFrustumFromCloud: Set camera info: %d x %d\n", data->camera_info.height, data->camera_info.width);

		m_camera_model.fromCameraInfo(data->camera_info);
		m_camera_size = m_camera_model.fullResolution();
	}

	// Get transformation from m_publishFID frame id to the camera frame id
	{
		m_to_sensor = tf::StampedTransform::getIdentity();

		//std::cerr << "CamFID: " << m_cameraFrameId << ", PCFID: " << data->pointcloud2.header.frame_id << std::endl;

	    // If different frame id
	    if( m_cameraFrameId != m_publishFID )
	    {
	        // Some transforms
	        tf::StampedTransform ocToCamTf;

	        ros::Time stamp( data->header.stamp );
	        // Get transforms
	        try {
	            // Transformation - to, from, time, waiting time
	            m_tfListener2.waitForTransform(m_cameraFrameId, m_publishFID,
	                    stamp, ros::Duration(5));

	            m_tfListener2.lookupTransform( m_cameraFrameId, m_publishFID,
	            		stamp, ocToCamTf );

	        } catch (tf::TransformException& ex) {
	            ROS_ERROR_STREAM( "CCompressedPCPublisher::removeFrustumFromCloud: Transform error - " << ex.what() << ", quitting callback");
	            return 0;
	        }

	        m_to_sensor = ocToCamTf;
	    }
	}

	long count(0);

	// Copy current cloud to the buffer
	tPointCloudInternal buffer( cloud );
	tPointCloudInternal::iterator p, end(buffer.end());

	// Clear cloud
	cloud.clear();
	//std::cerr << "Buffer size: " << buffer.size() << std::endl;


	// Copy from the buffer only points out of the sensor cone
	for( p = buffer.begin(); p != end; ++p )
	{
		// Test input point
		tf::Point pos(p->x, p->y, p->z);
		tf::Point posRel = m_to_sensor(pos);
		cv::Point2d uv = m_camera_model.project3dToPixel(cv::Point3d( posRel.x(), posRel.y(), posRel.z()));

		// Add point if not in sensor cone
		if (!inSensorCone(uv))
		{
			++count;
			cloud.push_back( *p );
		}
	}

	return count;
}

/**
 * Test if point is in camera cone
 */
bool srs_env_model::CCompressedPCPublisher::inSensorCone(const cv::Point2d& uv) const
{
	//std::cerr << uv.x << " > " << m_camera_stereo_offset_left + 1 << " && " << uv.x << " < " << m_camera_size.width + m_camera_stereo_offset_right - 2 << std::endl;
	//std::cerr << uv.y <<	" > " << 1 << " && " << uv.y << " < " << m_camera_size.height - 2 << std::endl;
	// Check if projected 2D coordinate in pixel range.
		// This check is a little more restrictive than it should be by using
		// 1 pixel less to account for rounding / discretization errors.
		// Otherwise points on the corner are accounted to be in the sensor cone.
		return ((uv.x > m_camera_stereo_offset_left + 1) &&
				(uv.x < m_camera_size.width + m_camera_stereo_offset_right - 2) &&
				(uv.y > 1) &&
				(uv.y < m_camera_size.height - 2));
}

