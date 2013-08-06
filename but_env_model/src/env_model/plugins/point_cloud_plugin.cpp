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

#include <but_env_model/plugins/point_cloud_plugin.h>
#include <but_env_model/topics_list.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include "pcl_ros/pcl_nodelet.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define DEFAULT_INPUT_CLOUD_FRAME_ID "/head_cam3d_link"

/// Constructor
but_env_model::CPointCloudPlugin::CPointCloudPlugin(const std::string & name, bool subscribe)
: but_env_model::CServerPluginBase(name)
, m_publishPointCloud(true)
, m_pcPublisherName(POINTCLOUD_CENTERS_PUBLISHER_NAME)
, m_pcSubscriberName(SUBSCRIBER_POINT_CLOUD_NAME)
, m_inputPcFrameId(DEFAULT_INPUT_CLOUD_FRAME_ID)
, m_bSubscribe( subscribe )
, m_latchedTopics( false )
, m_bFilterPC(true)
, m_bTransformPC(true)
, m_pointcloudMinZ(-std::numeric_limits<double>::max())
, m_pointcloudMaxZ(std::numeric_limits<double>::max())
, m_oldCloud( new tPointCloud )
, m_bufferCloud( new tPointCloud )
, m_frame_number( 0 )
, m_use_every_nth( 1 )
, m_bUseInputColor(true)
, m_r(128)
, m_g(128)
, m_b(128)
{
	assert( m_data != 0 );
	m_frame_id = "/map";
}

/// Destructor
but_env_model::CPointCloudPlugin::~CPointCloudPlugin()
{

}

//! Initialize plugin - called in server constructor
void but_env_model::CPointCloudPlugin::init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
{
    PINFO( "Initializing CPointCloudPlugin" );

	// Frame skipping
	int fs( m_use_every_nth );
	private_nh.param( "pointcloud_frame_skip", fs, 1 );
	m_use_every_nth = (fs >= 1) ? fs : 1;

	// 2013/01/31 Majkl: I guess we should publish the map in the Octomap TF frame...
	// We will use the same frame id as octomap plugin
	private_nh.param("ocmap_frame_id", m_frame_id, m_frame_id);

	// Point cloud limits
	private_nh.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
	private_nh.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);

	// Use input color default state
	private_nh.param("pointcloud_use_input_color", m_bUseInputColor, m_bUseInputColor);

	// Default color
	int c;
	c = m_r; private_nh.param("pointcloud_default_color_r", c, c);	m_r = c;
	c = m_g; private_nh.param("pointcloud_default_color_g", c, c);	m_g = c;
	c = m_b; private_nh.param("pointcloud_default_color_b", c, c);	m_b = c;

	// Get collision map crawling depth
	int depth(m_crawlDepth);
	private_nh.param("pointcloud_octree_depth", depth, depth);
	m_crawlDepth = depth > 0 ? depth : 0;

	PINFO( "Use input color: " << std::string(m_bUseInputColor ? "yes" : "no") );
	PINFO( "Default color: " << int(m_r) << ", " << int(m_g) << ", " << int(m_b) );

	// Create publisher
	m_pcPublisher = nh.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);

	// If should subscribe, create message filter and connect to the topic
	if( m_bSubscribe )
	{
		PINFO( "Subscribing input topic " << m_pcSubscriberName << ", frame ID " << m_inputPcFrameId );

		// Create subscriber
		m_pcSubscriber  = new message_filters::Subscriber<tIncommingPointCloud>(nh, m_pcSubscriberName, 1);
		if (!m_pcSubscriber)
		{
			PERROR( "Not subscribed to point cloud publisher..." );
		}

		// Create message filter
		m_tfPointCloudSub = new tf::MessageFilter<tIncommingPointCloud>( *m_pcSubscriber, m_tfListener, m_inputPcFrameId, 10);
		m_tfPointCloudSub->registerCallback(boost::bind( &CPointCloudPlugin::insertCloudCallback, this, _1));
	}

	// Clear old pointcloud data
	m_data->clear();

	PINFO( "CPointCloudPlugin initialized" );
}

//! Called when new scan was inserted and now all can be published
void but_env_model::CPointCloudPlugin::publishInternal(const ros::Time & timestamp)
{
    boost::mutex::scoped_lock lock(m_lockData);

    ROS_INFO_STREAM_ONCE( "CPointCloudPlugin::publishInternal(): called" );

	// No subscriber or disabled
	if( ! shouldPublish() )
		return;

	// No data...
	if( m_data->size() == 0 )
		return;

	// Convert data
	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg< tPclPoint >(*m_data, cloud);

	// Set message parameters and publish
	if( m_bTransformPC && m_data->header.frame_id != m_frame_id )
	{
		ROS_ERROR("CPointCloudPlugin::publishInternal(): Internal frame id is not compatible with the output one." );
		return;
	}
	if( m_bTransformPC )
		cloud.header.frame_id = m_frame_id;
	cloud.header.stamp = timestamp;

	ROS_INFO_STREAM_ONCE( "Publishing point cloud, size: " << m_data->size() << ", topic: " << m_pcPublisher.getTopic() );

	m_pcPublisher.publish(cloud);
}

//! Set used octomap frame id and timestamp
void but_env_model::CPointCloudPlugin::newMapDataCB( SMapWithParameters & par )
{
	if( ! m_publishPointCloud )
		return;

	boost::mutex::scoped_lock lock(m_lockData);

	m_data->clear();

	// Just for sure
	if(m_frame_id != par.frameId)
	{
		PERROR("Map frame ID has changed, this should never happen. Exiting newMapDataCB.");
		return;
	}

	m_frame_id = par.frameId;
	m_DataTimeStamp = m_time_stamp = par.currentTime;
	counter = 0;

	// Pointcloud is used as output for octomap...
	m_bAsInput = false;

	// Initialize leaf iterators
	tButServerOcTree & tree( par.map->getTree() );
	but_env_model::tButServerOcTree::leaf_iterator it, itEnd( tree.end_leafs() );

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

	lock.unlock();

	invalidate();
}

/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
void but_env_model::CPointCloudPlugin::handleOccupiedNode(but_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
{
//	std::cerr << "PCP: handle occupied" << std::endl;

    tPclPoint point;

	// Set position
	point.x = it.getX();
	point.y = it.getY();
	point.z = it.getZ();

	// Set color
	point.r = it->r();
	point.g = it->g();
	point.b = it->b();

//	std::cerr << "Occupied node r " << (int)point.r << ", g " << (int)point.g << ", b " << (int)point.b << std::endl;
	/*
	// Set color
	point.r = 255 - counter % 255;
	point.g = counter % 255;
	point.b = 128;
*/
	m_data->push_back( point );

	++counter;
}

///////////////////////////////////////////////////////////////////////////////

/**
 Cloud insertion callback
 */
void but_env_model::CPointCloudPlugin::insertCloudCallback( const  tIncommingPointCloud::ConstPtr& cloud)
{
    ROS_INFO_ONCE( "PointCloudPlugin::insertCloudCallback() called" );

	boost::mutex::scoped_lock lock(m_lockData);

	if( !useFrame() )
	{
		std::cerr << "Frame skipping" << std::endl;
		return;
	}

	m_bAsInput = true;

	// Convert input pointcloud
	m_data->clear();
	bool bIsRgbCloud(isRGBCloud( cloud ));

	if( !bIsRgbCloud  )
	{
		pcl::PointCloud< pcl::PointXYZ >::Ptr bufferCloud( new pcl::PointCloud< pcl::PointXYZ> );

		pcl::fromROSMsg(*cloud, *bufferCloud );
		pcl::copyPointCloud< pcl::PointXYZ, tPclPoint >( *bufferCloud, *m_data );
	}
	else
	{
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr bufferCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

		pcl::fromROSMsg(*cloud, *bufferCloud);
		pcl::copyPointCloud<pcl::PointXYZRGB, tPclPoint>( *bufferCloud, *m_data );
	}

	if( !(bIsRgbCloud && m_bUseInputColor) )
	{
		// Use our default color to colorize cloud
		tPointCloud::iterator it, itEnd(m_data->points.end());
		for( it = m_data->points.begin(); it != itEnd; ++it)
		{
			it->r = m_r; it->g = m_g; it->b = m_b;
		}
	}

	// If different frame id
	if( m_bTransformPC && cloud->header.frame_id != m_frame_id )
	{
	    ROS_INFO_ONCE( "CPointCloudPlugin::insertCloudCallback(): Waiting for TF transform" );

	    // Some transforms
		tf::StampedTransform sensorToPcTf;

		// Get transforms
		try {
			// Transformation - from, to, time, waiting time
			m_tfListener.waitForTransform(m_frame_id, cloud->header.frame_id,
					cloud->header.stamp, ros::Duration(5));

			m_tfListener.lookupTransform(m_frame_id, cloud->header.frame_id,
					cloud->header.stamp, sensorToPcTf);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			return;
		}

		Eigen::Matrix4f sensorToPcTM;

		// Get transformation matrix
		pcl_ros::transformAsMatrix(sensorToPcTf, sensorToPcTM);	// Sensor TF to defined base TF

		// transform pointcloud from sensor frame to the preset frame
		pcl::transformPointCloud< tPclPoint >(*m_data, *m_data, sensorToPcTM);
		m_data->header = cloud->header;
		m_data->header.frame_id = m_frame_id;
	}

	// Filter input pointcloud
	if( m_bFilterPC )		// TODO: Optimize this by removing redundant transforms
	{
//		PERROR( "Wait for filtering transform");

		// Get transforms to and from base id
		tf::StampedTransform pcToBaseTf, baseToPcTf;
		try {
			// Transformation - to, from, time, waiting time
			m_tfListener.waitForTransform(BASE_FRAME_ID, m_frame_id,
					cloud->header.stamp, ros::Duration(5));

			m_tfListener.lookupTransform(BASE_FRAME_ID, m_frame_id,
					cloud->header.stamp, pcToBaseTf);

			m_tfListener.lookupTransform(m_frame_id, BASE_FRAME_ID,
					cloud->header.stamp, baseToPcTf );

		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			return;
		}

		Eigen::Matrix4f pcToBaseTM, baseToPcTM;

		// Get transformation matrix
		pcl_ros::transformAsMatrix(pcToBaseTf, pcToBaseTM);	// Sensor TF to defined base TF
		pcl_ros::transformAsMatrix(baseToPcTf, baseToPcTM);	// Sensor TF to defined base TF

		// transform pointcloud from pc frame to the base frame
		pcl::transformPointCloud< tPclPoint >(*m_data, *m_data, pcToBaseTM);

		// filter height and range, also removes NANs:
		pcl::PassThrough<tPclPoint> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
		pass.setInputCloud(m_data->makeShared());
		pass.filter(*m_data);

		// transform pointcloud back to pc frame from the base frame
		pcl::transformPointCloud< tPclPoint >(*m_data, *m_data, baseToPcTM);
	}

	// Modify header
	m_data->header = cloud->header;
	if(m_bTransformPC)
		m_data->header.frame_id = m_frame_id;

    // Store timestamp
    m_DataTimeStamp = cloud->header.stamp;

//    PERROR("CPointCloudPlugin::insertCloudCallback(): stamp = %f", cloud->header.stamp.toSec());

    // Unlock for invalidation (it has it's own lock)
    lock.unlock();

 	invalidate();
}

//! Should plugin publish data?
bool but_env_model::CPointCloudPlugin::shouldPublish()
{
	return( (!m_bAsInput) && m_publishPointCloud && m_pcPublisher.getNumSubscribers() > 0 );
}

/**
 * Test if incomming pointcloud2 has rgb part - parameter driven
 */
bool but_env_model::CPointCloudPlugin::isRGBCloud( const tIncommingPointCloud::ConstPtr& cloud )
{
	tIncommingPointCloud::_fields_type::const_iterator i, end;

	for( i = cloud->fields.begin(), end = cloud->fields.end(); i != end; ++i )
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
 * Pause/resume plugin. All publishers and subscribers are disconnected on pause
 */
void but_env_model::CPointCloudPlugin::pause( bool bPause, ros::NodeHandle & nh )
{
	boost::mutex::scoped_lock lock(m_lockData);

	if( bPause )
	{
		m_pcPublisher.shutdown();

		if( m_bSubscribe )
		{

			m_pcSubscriber->unsubscribe();

			m_tfPointCloudSub->clear();

			delete m_tfPointCloudSub;
			delete m_pcSubscriber;
		}
	}
	else
	{
		// Create publisher
		m_pcPublisher = nh.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);

		if( m_bSubscribe )
		{
			m_pcSubscriber  = new message_filters::Subscriber<tIncommingPointCloud>(nh, m_pcSubscriberName, 1);

			// Create message filter
			m_tfPointCloudSub = new tf::MessageFilter<tIncommingPointCloud>( *m_pcSubscriber, m_tfListener, m_inputPcFrameId, 1);
			m_tfPointCloudSub->registerCallback(boost::bind( &CPointCloudPlugin::insertCloudCallback, this, _1));
		}
	}
}

/**
 * Wants plugin new map data?
 */
bool but_env_model::CPointCloudPlugin::wantsMap()
{
	return ! m_bAsInput;
}
