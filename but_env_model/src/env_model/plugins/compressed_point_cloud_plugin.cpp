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

#include <srs_env_model/but_server/plugins/compressed_point_cloud_plugin.h>
#include <srs_env_model/topics_list.h>

#include <pcl_ros/transforms.h>
#include <Eigen/src/Geometry/Quaternion.h>

/**
 * Constructor
 */
srs_env_model::CCompressedPointCloudPlugin::CCompressedPointCloudPlugin( const std::string & name )
: srs_env_model::CPointCloudPlugin( name, false )
, m_bTransformCamera( false )
, m_bSpinThread( true )
, m_frame_counter(0)
, m_uncomplete_frames( 10 )
, m_bPublishComplete( false )
, m_octomap_updates_msg( new OctomapUpdates )
, m_bTransformOutput( false )
{
	m_use_every_nth = 1;
}

/**
 * Destructor - kill thread
 */
srs_env_model::CCompressedPointCloudPlugin::~CCompressedPointCloudPlugin()
{
	if (spin_thread_.get())
	{
		need_to_terminate_ = true;
		spin_thread_->join();
	}
}

/**
 * Thread body - call callbacks, if needed
 */
void srs_env_model::CCompressedPointCloudPlugin::spinThread()
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
 * Initialize plugin
 */
void srs_env_model::CCompressedPointCloudPlugin::init(ros::NodeHandle & node_handle)
{
	ROS_DEBUG("Initializing CCompressedPointCloudPlugin");

	// 2013/01/31 Majkl: I guess we should publish the map in the Octomap TF frame...
	node_handle.param("ocmap_frame_id", m_frame_id, m_frame_id);

	if ( m_bSpinThread )
	{
		// if we're spinning our own thread, we'll also need our own callback queue
		node_handle.setCallbackQueue( &callback_queue_ );

		need_to_terminate_ = false;
		spin_thread_.reset( new boost::thread(boost::bind(&CCompressedPointCloudPlugin::spinThread, this)) );
		node_handle_ = node_handle;
	}

    // Read parameters
	{
		// Where to get camera position information
		node_handle.param("compressed_pc_camera_info_topic_name", m_cameraInfoTopic, CPC_CAMERA_INFO_PUBLISHER_NAME );
		ROS_INFO("SRS_ENV_SERVER: parameter - compressed_pc_camera_info_topic_name: %s", m_cameraInfoTopic.c_str() );

		// Point cloud publishing topic name
		node_handle.param("compressed_pc_pointcloud_centers_publisher", m_pcPublisherName, CPC_SIMPLE_PC_PUBLISHING_TOPIC_NAME);
		ROS_INFO("SRS_ENV_SERVER: parameter - compressed_pc_pointcloud_centers_publisher: %s", m_pcPublisherName.c_str() );

		// Should simple pointcloud be published too?
		node_handle.param("publish_simple_cloud", m_bPublishSimpleCloud, false );

		// How many uncomplete frames should be published before complete one
		int uf;
		node_handle.param("compressed_pc_differential_frames_count", uf, CPC_NUM_DIFFERENTIAL_FRAMES );
		m_uncomplete_frames = uf;

		ROS_INFO("SRS_ENV_SERVER: parameter - compressed_pc_update_data_topic_name: %i", uf );

		// Complete data topic name
		node_handle.param( "compressed_pc_update_data_topic_name", m_ocUpdatePublisherName, CPC_COMPLETE_TOPIC_NAME );
		ROS_INFO("SRS_ENV_SERVER: parameter - compressed_pc_update_data_topic_name: %s", m_ocUpdatePublisherName.c_str() );
	}

	// Services
	{
		m_serviceSetNumIncomplete = node_handle.advertiseService( SetNumIncompleteFrames_SRV,
				&srs_env_model::CCompressedPointCloudPlugin::setNumIncompleteFramesCB, this );
	}

    // Create publisher - simple point cloud
	if( m_bPublishSimpleCloud)
		m_pcPublisher = node_handle.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);

    // Create publisher - packed info - cam position and update pointcloud
    m_ocUpdatePublisher = node_handle.advertise< srs_env_model::OctomapUpdates > ( m_ocUpdatePublisherName, 5, m_latchedTopics );

    // Subscribe to position topic
    // Create subscriber
    m_camPosSubscriber = node_handle.subscribe<sensor_msgs::CameraInfo>( m_cameraInfoTopic, 10, &srs_env_model::CCompressedPointCloudPlugin::onCameraChangedCB, this );
    // = new message_filters::Subscriber<srs_env_model_msgs::RVIZCameraPosition>(node_handle, cameraPositionTopic, 1);

    if (!m_camPosSubscriber)
    {
        ROS_ERROR("Not subscribed...");
        PERROR( "Cannot subscribe to the camera position publisher..." );
    }
/*
    // Create message filter
    m_tfCamPosSub = new tf::MessageFilter<srs_env_model_msgs::RVIZCameraPosition>( *m_camPosSubscriber, m_tfListener, "/map", 1);
    m_tfCamPosSub->registerCallback(boost::bind( &CCompressedPointCloudPlugin::onCameraPositionChangedCB, this, _1));
*/
    // Clear old pointcloud data
    m_data->clear();

    // stereo cam params for sensor cone:
	node_handle.param<int> ("compressed_pc_camera_stereo_offset_left", m_camera_stereo_offset_left, 0); // 128
	node_handle.param<int> ("compressed_pc_camera_stereo_offset_right", m_camera_stereo_offset_right, 0);

}

/**
 * Set used octomap frame id and timestamp
 */

void srs_env_model::CCompressedPointCloudPlugin::newMapDataCB( SMapWithParameters & par )
{
	ROS_DEBUG( "CCompressedPointCloudPlugin: onFrameStart" );

    // Copy buffered camera normal and d parameter
    boost::recursive_mutex::scoped_lock lock( m_camPosMutex );

	// Increase frames count
	++m_frame_counter;

	// Is this frame complete?
	m_bPublishComplete = m_frame_counter % m_uncomplete_frames == 0;

	// Create packed octomap update info?
	m_bCreatePackedInfoMsg = m_ocUpdatePublisher.getNumSubscribers() > 0;

    // Reset counters
    m_countVisible = m_countAll = 0;

    min = 10000000;
    max = -10000000;

    // Call parent frame start
    if( ! m_publishPointCloud )
    		return;

    // Just for sure
	if(m_frame_id != par.frameId)
	{
		PERROR("Map frame id has changed, this should never happen. Exiting newMapDataCB.");
		return;
	}

    // Clear data
	m_data->clear();
	m_DataTimeStamp = m_time_stamp = par.currentTime;
	counter = 0;

	// Pointcloud is used as output for octomap...
	m_bAsInput = false;

    if( m_cameraFrameId.size() == 0 )
    {
        ROS_DEBUG("CCompressedPointCloudPlugin::newMapDataCB: Wrong camera frame id...");
//        m_bTransformCamera = false;
        return;
    }

//    m_bTransformCamera = m_cameraFrameId != m_pcFrameId;
    bool m_bTransformCamera(m_cameraFrameId != m_frame_id);

    m_to_sensor = tf::StampedTransform::getIdentity();

    // If different frame id
    if( m_bTransformCamera )
    {
        // Some transforms
        tf::StampedTransform camToOcTf, ocToCamTf;

        // Get transforms
        try {
            // Transformation - to, from, time, waiting time
            m_tfListener.waitForTransform(m_cameraFrameId, m_frame_id,
                    par.currentTime, ros::Duration(5));

            m_tfListener.lookupTransform( m_cameraFrameId, m_frame_id,
            		par.currentTime, ocToCamTf );

        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM( m_name << ": Transform error - " << ex.what() << ", quitting callback");
            PERROR( "Camera FID: " << m_cameraFrameId << ", Octomap FID: " << m_frame_id );
            return;
        }

        m_to_sensor = ocToCamTf;

//        PERROR( "Camera position: " << m_camToOcTrans );
    }

    // Store camera information
    m_camera_size = m_camera_size_buffer;
    m_camera_model.fromCameraInfo( m_camera_info_buffer);
    m_octomap_updates_msg->camera_info = m_camera_info_buffer;
    m_octomap_updates_msg->pointcloud2.header.stamp = par.currentTime;

    // Majkl 2013/01/24: missing frame id in the message header
    m_octomap_updates_msg->pointcloud2.header.frame_id = par.frameId;

    // Initialize leaf iterators
	tButServerOcTree & tree( par.map->getTree() );
	srs_env_model::tButServerOcTree::leaf_iterator it, itEnd( tree.end_leafs() );

	// Crawl through nodes
	for ( it = tree.begin_leafs(m_crawlDepth); it != itEnd; ++it)
	{
		// Node is occupied?
		if (tree.isNodeOccupied(*it))
		{
			handleOccupiedNode(it, par);
		}// Node is occupied?

	} // Iterate through octree

	// 2013/01/31 Majkl
	m_data->header.frame_id = par.frameId;
	m_data->header.stamp = par.currentTime;

	m_DataTimeStamp = par.currentTime;

	lock.unlock();

	invalidate();
}

/**
 * hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
 */
void srs_env_model::CCompressedPointCloudPlugin::handleOccupiedNode(srs_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
{
//	PERROR("OnHandleOccupied");

	++m_countAll;

	if( ! m_bCamModelInitialized )
		return;

	if( ! m_bPublishComplete )
	{
		// Test input point
		tf::Point pos(it.getX(), it.getY(), it.getZ());
		tf::Point posRel = m_to_sensor(pos);
		cv::Point2d uv = m_camera_model.project3dToPixel(cv::Point3d( posRel.x(), posRel.y(), posRel.z()));

		// ignore point if not in sensor cone
		if (!inSensorCone(uv))
			return;
	}

	// Ok, add it...
	CPointCloudPlugin::handleOccupiedNode( it, mp );
	++m_countVisible;
}

/**
 * On camera position changed callback
 */
void srs_env_model::CCompressedPointCloudPlugin::onCameraChangedCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{
	boost::recursive_mutex::scoped_lock lock( m_camPosMutex );

	//PERROR("OnCameraChange.");

//	ROS_DEBUG( "CCompressedPointCloudPlugin: onCameraChangedCB" );

    // Set camera position frame id
    m_cameraFrameId = cam_info->header.frame_id;

    ROS_DEBUG("OctMapPlugin: Set camera info: %d x %d\n", cam_info->height, cam_info->width);
	m_camera_model_buffer.fromCameraInfo(*cam_info);
	m_camera_size_buffer = m_camera_model_buffer.fullResolution();

	// Set flag
	m_bCamModelInitialized = true;

    m_camera_info_buffer = *cam_info;
}

//! Called when new scan was inserted and now all can be published
void srs_env_model::CCompressedPointCloudPlugin::publishInternal(const ros::Time & timestamp)
{
//	ROS_DEBUG( "CCompressedPointCloudPlugin: onPublish" );

//    PERROR( "Visible: " << m_countVisible << ", all: " << m_countAll << ", compression: " << double(m_countVisible)/double(m_countAll) );
//    PERROR( "Num of points: " << m_data->size() );
//    srs_env_model::CPointCloudPlugin::publishInternal( timestamp );

    if( shouldPublish() )
    {
    	// Fill header information
    	m_octomap_updates_msg->header = m_data->header;

		// Majkl 2013/1/24: trying to solve empty header of the output
    	m_octomap_updates_msg->header.stamp = m_DataTimeStamp;
    	m_octomap_updates_msg->header.frame_id = m_frame_id;

    	// Convert data
		pcl::toROSMsg< tPclPoint >(*m_data, m_octomap_updates_msg->pointcloud2);

		// Set message parameters and publish
		m_octomap_updates_msg->pointcloud2.header.frame_id = m_frame_id;
	//	m_octomap_updates_msg->pointcloud2.header.stamp = timestamp;

		// Majkl 2013/1/24: trying to solve empty header of the output
		m_octomap_updates_msg->pointcloud2.header.stamp = m_DataTimeStamp;

		if( m_bPublishComplete )
		{
//			std::cerr << "Publishing complete cloud. Size: " << m_data->points.size() << std::endl;
			m_octomap_updates_msg->isPartial = 0;
		}
		else
		{
//			std::cerr << "Publishing only differential cloud. Size: " << m_data->points.size() << std::endl;
			m_octomap_updates_msg->isPartial = 1;
		}

		// Publish data
		m_ocUpdatePublisher.publish( m_octomap_updates_msg );

		if( m_bPublishSimpleCloud )
			m_pcPublisher.publish( m_octomap_updates_msg->pointcloud2 );
    }
}

/**
 *  Connect/disconnect plugin to/from all topics
 */
void srs_env_model::CCompressedPointCloudPlugin::pause( bool bPause, ros::NodeHandle & node_handle)
{
	boost::recursive_mutex::scoped_lock lock( m_camPosMutex );

	if( bPause )
	{
		m_pcPublisher.shutdown();
		m_camPosSubscriber.shutdown();
	}
	else
	{
		// Create publisher
		m_pcPublisher = node_handle.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);

		// Subscribe to position topic
		// Create subscriber
		m_camPosSubscriber = node_handle.subscribe<sensor_msgs::CameraInfo>( m_cameraInfoTopic, 10, &srs_env_model::CCompressedPointCloudPlugin::onCameraChangedCB, this );
	}
}

/**
 * Test if point is in camera cone
 */
bool srs_env_model::CCompressedPointCloudPlugin::inSensorCone(const cv::Point2d& uv) const
{
	//PERROR( uv.x << " > " << m_camera_stereo_offset_left + 1 << " && " << uv.x << " < " << m_camera_size.width + m_camera_stereo_offset_right - 2 );
	//PERROR( uv.y <<	" > " << 1 << " && " << uv.y << " < " << m_camera_size.height - 2 );
	// Check if projected 2D coordinate in pixel range.
		// This check is a little more restrictive than it should be by using
		// 1 pixel less to account for rounding / discretization errors.
		// Otherwise points on the corner are accounted to be in the sensor cone.
		return ((uv.x > m_camera_stereo_offset_left + 1) &&
				(uv.x < m_camera_size.width + m_camera_stereo_offset_right - 2) &&
				(uv.y > 1) &&
				(uv.y < m_camera_size.height - 2));
}

/**
 * Should plugin publish data?
 */
bool srs_env_model::CCompressedPointCloudPlugin::shouldPublish()
{
	ROS_DEBUG( "CCompressedPointCloudPlugin: shouldPublish" );

	bool rv( m_bCamModelInitialized && m_publishPointCloud && (m_pcPublisher.getNumSubscribers() > 0 || m_ocUpdatePublisher.getNumSubscribers() > 0 ) );

	if( !rv )
	{
//		PERROR( "Should not publish." << int(m_bCamModelInitialized) << "." << int(m_publishPointCloud) << "." << int(m_pcPublisher.getNumSubscribers() > 0));
	}

	return rv;
}

/**
 * Set number of incomplete frames callback
 */
bool srs_env_model::CCompressedPointCloudPlugin::setNumIncompleteFramesCB( srs_env_model::SetNumIncompleteFrames::Request & req, srs_env_model::SetNumIncompleteFrames::Response & res )
{
	m_use_every_nth = req.num;
	PERROR( "New number of incomplete frames set: " << m_use_every_nth );

	return true;
}
