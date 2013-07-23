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

#include <srs_env_model/but_server/plugins/point_cloud_plugin.h>
#include <srs_env_model/topics_list.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include "pcl_ros/pcl_nodelet.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define DEFAULT_INPUT_CLOUD_FRAME_ID "/head_cam3d_link"

/// Constructor
srs_env_model::CPointCloudPlugin::CPointCloudPlugin(const std::string & name, bool subscribe)
: srs_env_model::CServerPluginBase(name)
, m_publishPointCloud(true)
, m_pcPublisherName(POINTCLOUD_CENTERS_PUBLISHER_NAME)
, m_pcSubscriberName("")
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
srs_env_model::CPointCloudPlugin::~CPointCloudPlugin()
{

}

//! Initialize plugin - called in server constructor
void srs_env_model::CPointCloudPlugin::init(ros::NodeHandle & node_handle)
{
//	PERROR( "Initializing PointCloudPlugin" );

	// Frame skipping
	int fs( m_use_every_nth );
	node_handle.param( "pointcloud_frame_skip", fs, 1 );
	m_use_every_nth = (fs >= 1) ? fs : 1;

	// Point cloud publishing topic name
	node_handle.param("pointcloud_centers_publisher", m_pcPublisherName, POINTCLOUD_CENTERS_PUBLISHER_NAME );

	// Point cloud subscribing topic name - try to get it from parameter server if not given
	if( m_pcSubscriberName.length() == 0 )
		node_handle.param("pointcloud_subscriber", m_pcSubscriberName, SUBSCRIBER_POINT_CLOUD_NAME);
	else
		m_bSubscribe = true;

	// 2013/01/31 Majkl: I guess we should publish the map in the Octomap TF frame...
	// We will use the same frame id as octomap plugin
	node_handle.param("ocmap_frame_id", m_frame_id, m_frame_id);

	// Point cloud limits
	node_handle.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
	node_handle.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);

	// Use input color default state
	node_handle.param("pointcloud_use_input_color", m_bUseInputColor, m_bUseInputColor);

	// Default color
	int c;
	c = m_r; node_handle.param("pointcloud_default_color_r", c, c);	m_r = c;
	c = m_g; node_handle.param("pointcloud_default_color_g", c, c);	m_g = c;
	c = m_b; node_handle.param("pointcloud_default_color_b", c, c);	m_b = c;

	// Get collision map crawling depth
	int depth(m_crawlDepth);
	node_handle.param("pointcloud_octree_depth", depth, depth);
	m_crawlDepth = depth > 0 ? depth : 0;

//	std::cerr << "Use input color: " << std::string(m_bUseInputColor ? "yes" : "no") << std::endl;
//	std::cerr << "Color: " << m_r << ", " << m_g << ", " << m_b << std::endl;

	// Create publisher
	m_pcPublisher = node_handle.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);


	// If should subscribe, create message filter and connect to the topic
	if( m_bSubscribe )
	{
		std::cerr << "Plugin name: " << this->m_name << std::endl;

//		PERROR("Subscribing to: " << m_pcSubscriberName );
		// Create subscriber
		m_pcSubscriber  = new message_filters::Subscriber<tIncommingPointCloud>(node_handle, m_pcSubscriberName, 1);

		if (!m_pcSubscriber)
		{
			ROS_ERROR("Not subscribed...");
			PERROR( "Not subscirbed to point clouds subscriber...");
		}

		// Create message filter
		m_tfPointCloudSub = new tf::MessageFilter<tIncommingPointCloud>( *m_pcSubscriber, m_tfListener, m_inputPcFrameId, 1);
		m_tfPointCloudSub->registerCallback(boost::bind( &CPointCloudPlugin::insertCloudCallback, this, _1));

		//std::cerr << "SUBSCRIBER NAME: " << m_pcSubscriberName << ", FRAMEID: " << m_pcFrameId << std::endl;
	}

	// Clear old pointcloud data
	m_data->clear();

//	PERROR( "PointCloudPlugin initialized..." );
}

//! Called when new scan was inserted and now all can be published
void srs_env_model::CPointCloudPlugin::publishInternal(const ros::Time & timestamp)
{
//	PERROR("Publish: Try lock");

	boost::mutex::scoped_lock lock(m_lockData);

//	PERROR("Publish: Lock");

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
		ROS_ERROR("CPointCloudPlugin::publishInternal: Internal frame id is not compatible with the output one.");
		return;
	}
	if( m_bTransformPC )
		cloud.header.frame_id = m_frame_id;
	cloud.header.stamp = timestamp;

//	PERROR( "Publishing cloud. Size: " << m_data->size() << ", topic: " << m_pcPublisher.getTopic() );
	m_pcPublisher.publish(cloud);

//	PERROR("Publish: Unlock");
}

//! Set used octomap frame id and timestamp
void srs_env_model::CPointCloudPlugin::newMapDataCB( SMapWithParameters & par )
{
	if( ! m_publishPointCloud )
		return;

//	PERROR("New map: Try lock");

	boost::mutex::scoped_lock lock(m_lockData);

//	PERROR("New map: Lock");

	m_data->clear();

	// Just for sure
	if(m_frame_id != par.frameId)
	{
		PERROR("Map frame id has changed, this should never happen. Exiting newMapDataCB.");
		return;
	}

	m_frame_id = par.frameId;
	m_DataTimeStamp = m_time_stamp = par.currentTime;
	counter = 0;

	// Pointcloud is used as output for octomap...
	m_bAsInput = false;

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

	lock.unlock();
//	PERROR( "New map: Unlocked");

	invalidate();
}

/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
void srs_env_model::CPointCloudPlugin::handleOccupiedNode(srs_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
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
void srs_env_model::CPointCloudPlugin::insertCloudCallback( const  tIncommingPointCloud::ConstPtr& cloud)
{
//	PERROR("insertCloud: Try lock");

	boost::mutex::scoped_lock lock(m_lockData);

//	PERROR("insertCloud: Locked");

	if( ! useFrame() )
	{
		std::cerr << "Frame skipping" << std::endl;
		return;
	}

//	std::cerr << "PCP.iccb start. Time: " << ros::Time::now() << std::endl;

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

//	std::cerr << this->m_name << " input cloud " << (m_data->isOrganized() ? "is " : "is not ") << "organized." << std::endl;


	//*/
//	std::cerr << "Input cloud frame id: " << cloud->header.frame_id << std::endl;

	// If different frame id
	if( m_bTransformPC && cloud->header.frame_id != m_frame_id )
	{
//		PERROR( "Wait for input transform" );

	    // Some transforms
		tf::StampedTransform sensorToPcTf;

		// Get transforms
		try {
			// Transformation - from, to, time, waiting time
			m_tfListener.waitForTransform(m_frame_id, cloud->header.frame_id,
					cloud->header.stamp, ros::Duration(5));

			m_tfListener.lookupTransform(m_frame_id, cloud->header.frame_id,
					cloud->header.stamp, sensorToPcTf);

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			PERROR("Transform error");

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
//	std::cerr << this->m_name << " input cloud 1 " << (m_data->isOrganized() ? "is " : "is not ") << "organized." << std::endl;

//	PERROR("1");

//	PERROR("2");

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

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			PERROR( "Transform error.");
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

//    ROS_DEBUG("CPointCloudPlugin::insertCloudCallback(): stamp = %f", cloud->header.stamp.toSec());

 //   PERROR("Insert cloud CB. Size: " << m_data->size() );

    // Unlock for invalidation (it has it's own lock)
    lock.unlock();
//    PERROR("insertCloud: Unlocked");

//    std::cerr << this->m_name << " output cloud " << (m_data->isOrganized() ? "is " : "is not ") << "organized." << std::endl;

 	invalidate();

// 	PERROR("Unlock");
}

//! Should plugin publish data?
bool srs_env_model::CPointCloudPlugin::shouldPublish()
{
	return( (!m_bAsInput) && m_publishPointCloud && m_pcPublisher.getNumSubscribers() > 0 );
}

/**
 * Test if incomming pointcloud2 has rgb part - parameter driven
 */
bool srs_env_model::CPointCloudPlugin::isRGBCloud( const tIncommingPointCloud::ConstPtr& cloud )
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
void srs_env_model::CPointCloudPlugin::pause( bool bPause, ros::NodeHandle & node_handle )
{
//	PERROR("Try lock");

	boost::mutex::scoped_lock lock(m_lockData);

//	PERROR("Lock");

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
		m_pcPublisher = node_handle.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 5, m_latchedTopics);

		if( m_bSubscribe )
		{
			m_pcSubscriber  = new message_filters::Subscriber<tIncommingPointCloud>(node_handle, m_pcSubscriberName, 1);

			// Create message filter
			m_tfPointCloudSub = new tf::MessageFilter<tIncommingPointCloud>( *m_pcSubscriber, m_tfListener, m_inputPcFrameId, 1);
			m_tfPointCloudSub->registerCallback(boost::bind( &CPointCloudPlugin::insertCloudCallback, this, _1));
		}
	}

//	PERROR("Unlock");

}

/**
 * Wants plugin new map data?
 */
bool srs_env_model::CPointCloudPlugin::wantsMap()
{
	return ! m_bAsInput;
}


