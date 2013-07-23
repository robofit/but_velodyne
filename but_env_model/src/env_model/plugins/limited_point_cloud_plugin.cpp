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

#include <srs_env_model/but_server/plugins/limited_point_cloud_plugin.h>
#include <srs_env_model/topics_list.h>

#include <pcl_ros/transforms.h>
#include <Eigen/src/Geometry/Quaternion.h>

/**
 * Constructor
 */
srs_env_model::CLimitedPointCloudPlugin::CLimitedPointCloudPlugin( const std::string & name )
: srs_env_model::CPointCloudPlugin( name, false )
, m_bTransformCamera( false )
, m_bSpinThread( true )
{
}

/**
 * Destructor - kill thread
 */
srs_env_model::CLimitedPointCloudPlugin::~CLimitedPointCloudPlugin()
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
void srs_env_model::CLimitedPointCloudPlugin::spinThread()
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

void srs_env_model::CLimitedPointCloudPlugin::init(ros::NodeHandle & node_handle)
{
	if ( m_bSpinThread )
	{
		// if we're spinning our own thread, we'll also need our own callback queue
		node_handle.setCallbackQueue( &callback_queue_ );

		need_to_terminate_ = false;
		spin_thread_.reset( new boost::thread(boost::bind(&CLimitedPointCloudPlugin::spinThread, this)) );
		node_handle_ = node_handle;
	}

    // Read parameters
    node_handle.param("rviz_camera_position_topic_name", m_cameraPositionTopic, SUBSCRIBER_CAMERA_POSITION_NAME );

    // Point cloud publishing topic name
    node_handle.param("pointcloud_centers_publisher", m_pcPublisherName, VISIBLE_POINTCLOUD_CENTERS_PUBLISHER_NAME );

	// 2013/01/31 Majkl: I guess we should publish the map in the Octomap TF frame...
	node_handle.param("ocmap_frame_id", m_frame_id, m_frame_id);

    // Create publisher
    m_pcPublisher = node_handle.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);

    // Subscribe to position topic
    // Create subscriber
    m_camPosSubscriber = node_handle.subscribe<srs_env_model_msgs::RVIZCameraPosition>( m_cameraPositionTopic, 10, &srs_env_model::CLimitedPointCloudPlugin::onCameraPositionChangedCB, this );
    // = new message_filters::Subscriber<srs_env_model_msgs::RVIZCameraPosition>(node_handle, cameraPositionTopic, 1);

    if (!m_camPosSubscriber)
    {
        ROS_ERROR("Not subscribed...");
        PERROR( "Cannot subscribe to the camera position publisher..." );
    }
/*
    // Create message filter
    m_tfCamPosSub = new tf::MessageFilter<srs_env_model_msgs::RVIZCameraPosition>( *m_camPosSubscriber, m_tfListener, "/map", 1);
    m_tfCamPosSub->registerCallback(boost::bind( &CLimitedPointCloudPlugin::onCameraPositionChangedCB, this, _1));
*/
    // Clear old pointcloud data
    m_data->clear();
}

/**
 * Set used octomap frame id and timestamp
 */

void srs_env_model::CLimitedPointCloudPlugin::newMapDataCB( SMapWithParameters & par )
{
    // Reset counters
    m_countVisible = m_countHidden = 0;

    min = 10000000;
    max = -10000000;

    if( m_cameraFrameId.size() == 0 )
        {
            m_bTransformCamera = false;
            return;
        }

    if( ! m_publishPointCloud )
    		return;

    // Just for sure
	if(m_frame_id != par.frameId)
	{
		PERROR("Map frame id has changed, this should never happen. Exiting newMapDataCB.");
		return;
	}

    //	Clear data
	m_data->clear();
	m_DataTimeStamp = m_time_stamp = par.currentTime;
	counter = 0;

	// Pointcloud is used as output for octomap...
	m_bAsInput = false;



//    m_bTransformCamera = m_cameraFrameId != m_ocFrameId;
    bool m_bTransformCamera = m_cameraFrameId != m_frame_id;

    // If different frame id
    if( m_bTransformCamera )
    {
        // Some transforms
        tf::StampedTransform camToOcTf;

        // Get transforms
        try {
            // Transformation - from, to, time, waiting time
            m_tfListener.waitForTransform(m_frame_id, m_cameraFrameId,
                    par.currentTime, ros::Duration(5));

            m_tfListener.lookupTransform(m_frame_id, m_cameraFrameId,
                    par.currentTime, camToOcTf);

        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM( m_name << ": Transform error - " << ex.what() << ", quitting callback");
            PERROR( "Camera FID: " << m_cameraFrameId << ", Octomap FID: " << m_frame_id );
            return;
        }
 //       PERROR( "Camera FID: " << m_cameraFrameId << ", Octomap FID: " << m_ocFrameId );

        // Get transformation matrix
        Eigen::Matrix4f cameraToOcTM;
        pcl_ros::transformAsMatrix(camToOcTf, cameraToOcTM); // Sensor TF to defined base TF

        // Get translation and rotation
        m_camToOcRot  = cameraToOcTM.block<3, 3> (0, 0);
        m_camToOcTrans = cameraToOcTM.block<3, 1> (0, 3);

//        PERROR( "Camera position: " << m_camToOcTrans );
    }

    // Copy buffered camera normal and d parameter
    boost::recursive_mutex::scoped_lock lock( m_camPosMutex );

  //  PERROR( "Copy position...");
    m_d = m_dBuf;
    m_normal = m_normalBuf;

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
void srs_env_model::CLimitedPointCloudPlugin::handleOccupiedNode(srs_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
{
    Eigen::Vector3f p( it.getX(), it.getY(), it.getZ() );

     float f( p.dot( m_normal ) + m_d);
    if( f > max )
        max = f;
    if( f < min)
        min = f;

    // Test if point is in front of camera
    if( (p.dot( m_normal ) + m_d) > 0 )
    {
        // Ok, add it...
        CPointCloudPlugin::handleOccupiedNode( it, mp );
        ++m_countVisible;
    }
    else
    {
        ++m_countHidden;
  //      PERROR( "Point behind camera... ");
    }
}

/**
 * On camera position changed callback
 */
void srs_env_model::CLimitedPointCloudPlugin::onCameraPositionChangedCB(const srs_env_model_msgs::RVIZCameraPosition::ConstPtr& cameraPosition)
{
    // Set camera position frame id
    m_cameraFrameId = cameraPosition->header.frame_id;

    // Orientation shortcut
    const srs_env_model_msgs::RVIZCameraPosition::_position_type & p( cameraPosition->position );

    // Camera direction
    const srs_env_model_msgs::RVIZCameraPosition::_position_type & d( cameraPosition->direction );

    // Convert to eigen
    Eigen::Vector3f point( p.x, p.y, p.z );
    Eigen::Vector3f normal( d.x, d.y, d.z );


    if( m_bTransformCamera )
    {
 //       PERROR( "Transform camera");
        // Transform point to the octomap frame id
        point = m_camToOcRot * point + m_camToOcTrans;

        // Transform normal
        normal = m_camToOcRot * normal;
    }

    // Normalize normal vector
    normal.normalize();

    // Set parameters to the buffer
    boost::recursive_mutex::scoped_lock lock( m_camPosMutex );

    m_normalBuf = normal;

    // Compute last plane equation parameter
    m_dBuf = - point.dot( normal );

 //   PERROR( "Equation: " << normal[0] << ", " << normal[1] << ", " << normal[2] << ", " << m_d );
 //   PERROR( "Camera position: " << point[0] << ", " << point[1] << ", " << point[2] );
}

//! Called when new scan was inserted and now all can be published
void srs_env_model::CLimitedPointCloudPlugin::publishInternal(const ros::Time & timestamp)
{
//    PERROR( "Visible: " << m_countVisible << ", hidden: " << m_countHidden << ", min: " << min << ", max: " << max );
//    PERROR( "Num of points: " << m_data->size() );
    srs_env_model::CPointCloudPlugin::publishInternal( timestamp );
}

/**
 *  Connect/disconnect plugin to/from all topics
 */
void srs_env_model::CLimitedPointCloudPlugin::pause( bool bPause, ros::NodeHandle & node_handle)
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
		m_camPosSubscriber = node_handle.subscribe<srs_env_model_msgs::RVIZCameraPosition>( m_cameraPositionTopic, 10, &srs_env_model::CLimitedPointCloudPlugin::onCameraPositionChangedCB, this );
	}
}



