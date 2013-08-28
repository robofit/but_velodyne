/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz), Michal Spanel (spanel@fit.vutbr.cz)
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

#include <but_env_model/plugins/octomap_plugin.h>
#include <but_env_model/topics_list.h>
#include <but_env_model/plugins/point_cloud_plugin.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// Filtering
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Interactive marker
#include <but_interaction_primitives/AddUnknownObject.h>

#define DEFAULT_RESOLUTION 0.1

void but_env_model::COctoMapPlugin::setDefaults()
{
	// Set octomap parameters
	m_mapParameters.resolution = DEFAULT_RESOLUTION;
	m_mapParameters.treeDepth = 0;
	m_mapParameters.probHit = 0.7; // Probability of node, if node is occupied: 0.7
	m_mapParameters.probMiss = 0.4; // Probability of node, if node is free: 0.4
	m_mapParameters.thresMin = 0.12; // Clamping minimum threshold: 0.1192;
	m_mapParameters.thresMax = 0.97; // Clamping maximum threshold: 0.971;
	m_mapParameters.thresOccupancy = 0.5; // Occupied node threshold: 0.5
	m_mapParameters.maxRange = 8.0;

	// Set ground filtering parameters
	m_filterGroundPlane = false;
	m_removeSpecles = false;
	m_mapParameters.frameId = "/map";
	m_bPublishOctomap = true;

	// Filtering
	m_bRemoveOutdated = true;
	m_removeTester = 0; //new CTestingPolymesh(CTestingPolymesh::tPoint( 1.0, 1.0, 0.5 ), quat, CTestingPolymesh::tPoint( 1.0, 1.5, 2.0 ));
	m_testerLife = 1;

	// Set maximal tree depth used when crawling. Zero means maximal possible depth.
	m_crawlDepth = 0;
	m_bMapLoaded = false;
	m_bNotFirst = false;

	m_probDeleted = m_mapParameters.probMiss * 0.1;
	m_r = m_g = m_b = 128;

	// Octomap aging is disabled by default
    m_agingEnabled = false;
    m_agingProb = m_mapParameters.probMiss;
    m_agingProbLog = octomap::logodds(m_agingProb);
    m_agingPeriod = 1.0;
}

but_env_model::COctoMapPlugin::COctoMapPlugin(const std::string & name)
    : but_env_model::CServerPluginBase(name)
    , CDataHolderBase< tButServerOcMap >( new tButServerOcMap(DEFAULT_RESOLUTION) )
    , m_filecounter(0)
    , m_filterSingleSpecles("/map")
    , m_filterRaycast("/map")
    , m_filterGround("/map")
    , m_bFilterWithInput(false)
    , m_bNewDataToFilter(false)
    , m_bMapLoaded(false)
{
	setDefaults();

	assert( m_data != 0 );

	// Set octomap parameters
	m_data->getTree().setProbHit(m_mapParameters.probHit);
	m_data->getTree().setProbMiss(m_mapParameters.probMiss);
	m_data->getTree().setClampingThresMin(m_mapParameters.thresMin);
	m_data->getTree().setClampingThresMax(m_mapParameters.thresMax);
	m_data->getTree().setOccupancyThres(m_mapParameters.thresOccupancy);
	m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();
	m_mapParameters.map = m_data;
	m_mapParameters.crawlDepth = m_crawlDepth;
}

but_env_model::COctoMapPlugin::COctoMapPlugin(const std::string & name, const std::string & filename)
    : but_env_model::CServerPluginBase(name)
    , CDataHolderBase< tButServerOcMap >( new tButServerOcMap(DEFAULT_RESOLUTION) )
    , m_filterSingleSpecles("/map")
    , m_filterRaycast("/map")
    , m_filterGround("/map")
    , m_bFilterWithInput(false)
    , m_filterCloudPlugin(new CPointCloudPlugin("PCFILTER", false ))
    , m_bNewDataToFilter(false)
    , m_bMapLoaded(false)
{
	setDefaults();

	assert( m_data != 0 );

	// Set octomap parameters
	m_data->getTree().setProbHit(m_mapParameters.probHit);
	m_data->getTree().setProbMiss(m_mapParameters.probMiss);
	m_data->getTree().setClampingThresMin(m_mapParameters.thresMin);
	m_data->getTree().setClampingThresMax(m_mapParameters.thresMax);
	m_data->getTree().setOccupancyThres(m_mapParameters.thresOccupancy);
	m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();
	m_mapParameters.map = m_data;
	m_mapParameters.crawlDepth = m_crawlDepth;

	// is filename valid?
	if (filename.length() > 0)
	{
		// Try to load data
		if (m_data->getTree().readBinary(filename))
		{
			ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_data->getTree().size());

			// get tree depth
			m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();

			// get resolution
			m_mapParameters.resolution = m_data->getTree().getResolution();

			// Map was loaded
			m_bMapLoaded = true;

			// We have new data
			invalidate();
		}
		else
		{
			// Something is wrong - cannot load data...
			ROS_ERROR("Could not open requested file %s, continuing in standard mode.", filename.c_str());
			PERROR( "Transform error.");
		}
	}
}

/**
 * Destructor
 */
but_env_model::COctoMapPlugin::~COctoMapPlugin()
{
	// Remove tester
	if (m_removeTester != 0)
		delete m_removeTester;
}

//! Initialize plugin - called in server constructor
void but_env_model::COctoMapPlugin::init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
{
	PERROR( "Initializing OctoMapPlugin" );

	reset(false);

	private_nh.param("ocmap_resolution", m_mapParameters.resolution, m_mapParameters.resolution);
	int td( m_mapParameters.treeDepth );
	private_nh.param("ocmap_treedepth", td, td );
	m_mapParameters.treeDepth = ( td < 0 ) ? 0 : td;
	private_nh.param("ocmap_sensor_model/hit", m_mapParameters.probHit, m_mapParameters.probHit);
	private_nh.param("ocmap_sensor_model/miss", m_mapParameters.probMiss, m_mapParameters.probMiss);
	private_nh.param("ocmap_sensor_model/min", m_mapParameters.thresMin, m_mapParameters.thresMin);
	private_nh.param("ocmap_sensor_model/max", m_mapParameters.thresMax, m_mapParameters.thresMax);
	private_nh.param("ocmap_max_range", m_mapParameters.maxRange, m_mapParameters.maxRange);
	private_nh.param("ocmap_frame_id", m_mapParameters.frameId, m_mapParameters.frameId );

	// Set octomap parameters...
	{
		m_data->getTree().setResolution(m_mapParameters.resolution);
		m_data->getTree().setProbHit(m_mapParameters.probHit);
		m_data->getTree().setProbMiss(m_mapParameters.probMiss);
		m_data->getTree().setClampingThresMin(m_mapParameters.thresMin);
		m_data->getTree().setClampingThresMax(m_mapParameters.thresMax);
	}

	// Default color
	int c;
	c = m_r; private_nh.param("pointcloud_default_color_r", c, c);	m_r = c;
	c = m_g; private_nh.param("pointcloud_default_color_g", c, c);	m_g = c;
	c = m_b; private_nh.param("pointcloud_default_color_b", c, c);	m_b = c;

	// Should ground plane be filtered?
	private_nh.param("ocmap_filter_ground", m_filterGroundPlane, m_filterGroundPlane);

	// Should potentially free cells be filtered?
	private_nh.param("ocmap_filter_outdated", m_bRemoveOutdated, m_bRemoveOutdated );

	// Should be the input cloud used for raycast filtering?
	private_nh.param("use_input_for_filter", m_bFilterWithInput, m_bFilterWithInput );

	// Octomap publishing topic
	private_nh.param("ocmap_publishing_topic", m_ocPublisherName, OCTOMAP_PUBLISHER_NAME);

	// Octomap aging
    private_nh.param("ocmap_aging", m_agingEnabled, m_agingEnabled );
    private_nh.param("ocmap_aging_prob", m_agingProb, m_agingProb );
    m_agingProb = (m_agingProb < 0.0) ? 0.0 : m_agingProb;
    m_agingProb = (m_agingProb > 0.5) ? 0.5 : m_agingProb;
    m_agingProbLog = octomap::logodds(m_agingProb);
    private_nh.param("ocmap_aging_period", m_agingPeriod, m_agingPeriod );
    m_agingPeriod = (m_agingPeriod < 0.1) ? 0.1 : m_agingPeriod;

	// Advertise services
	m_serviceResetOctomap = nh.advertiseService(ResetOctomap_SRV,
			&but_env_model::COctoMapPlugin::resetOctomapCB, this);

	m_serviceRemoveCube = nh.advertiseService(RemoveCubeOctomap_SRV,
			&but_env_model::COctoMapPlugin::removeCubeCB, this);

	m_serviceAddCube = nh.advertiseService( AddCubeOctomap_SRV,
			&but_env_model::COctoMapPlugin::addCubeCB, this);

	m_serviceSetCrawlDepth = nh.advertiseService( SetCrawlDepth_SRV,
			&but_env_model::COctoMapPlugin::setCrawlingDepthCB, this );

	m_serviceGetTreeDepth = nh.advertiseService( GetTreeDepth_SRV,
			&but_env_model::COctoMapPlugin::getTreeDepthCB, this );

	m_serviceLoadMap = nh.advertiseService( LoadMap_SRV,
			&but_env_model::COctoMapPlugin::loadOctreeCB, this);

	m_serviceSaveMap = nh.advertiseService( SaveMap_SRV,
				&but_env_model::COctoMapPlugin::saveOctreeCB, this);

	m_serviceLoadFullMap = nh.advertiseService( LoadFullMap_SRV,
				&but_env_model::COctoMapPlugin::loadFullOctreeCB, this);

	m_serviceSaveFullMap = nh.advertiseService( SaveFullMap_SRV,
				&but_env_model::COctoMapPlugin::saveFullOctreeCB, this);

    m_serviceAgingPause = nh.advertiseService( OctomapAgingPause_SRV,
                &but_env_model::COctoMapPlugin::agingPauseCB, this);

	// Create publisher
	m_ocPublisher = nh.advertise<octomap_msgs::Octomap> (
			m_ocPublisherName, 5, m_latchedTopics);

	m_registration.init( nh );

	PERROR( "OctoMapPlugin initialized..." );

	m_bNotFirst = false;

	m_probDeleted = m_mapParameters.probMiss * 0.1;

	// Initialize filters
	m_filterSingleSpecles.setTreeFrameId(m_mapParameters.frameId);
	m_filterRaycast.setTreeFrameId(m_mapParameters.frameId);
	m_filterRaycast.init(nh);
	m_filterGround.setTreeFrameId(m_mapParameters.frameId);
	m_filterGround.init(nh);

	// Set specles filter 20 seconds delay
	m_filterSingleSpecles.setRunMode( COcTreeFilterBase::FILTER_TEST_TIME );
	m_filterSingleSpecles.setTimerLap(20);

	// Create and connect filter cloud plugin, is needed
	if( !m_bFilterWithInput )
	{
		// Initialize filter pointcloud plugin
		std::cerr << "Initializing filter-in pointcloud plugin." << std::endl;
		m_filterCloudPlugin->init(nh, private_nh, SUBSCRIBER_FILTERING_CLOUD_NAME);

		// Disable cloud filtering
		m_filterCloudPlugin->enableCloudFiltering(false);

		// Disable cloud publishing
		m_filterCloudPlugin->enable(false);

		// Disable filter cloud transformation
		m_filterCloudPlugin->enableCloudTransform(false);

		// Connect input point cloud input with octomap
		m_filterCloudPlugin->getSigDataChanged().connect( boost::bind( &COctoMapPlugin::filterCloud, this, _1 ));
	}

	// Initialize the aging timer
	if( m_agingEnabled )
	{
	    m_agingTimer = nh.createTimer( m_agingPeriod, &COctoMapPlugin::agingCallback, this );
	}
}

void but_env_model::COctoMapPlugin::insertCloud(tPointCloud::ConstPtr cloud)
{
//	PERROR("insertCloud: Try lock.");

	// Lock data
	boost::mutex::scoped_lock lock(m_lockData);
//	PERROR("insertCloud: Locked.");

	tPointCloud used_cloud;
	pcl::copyPointCloud( *cloud, used_cloud );
	//*

	Eigen::Matrix4f registration_transform( Eigen::Matrix4f::Identity() );

	// Registration
	{
		if( m_registration.isRegistering() && cloud->size() > 0 && m_bNotFirst )
		{
//			pcl::copyPointCloud( *m_data, *m_bufferCloud );

			tPointCloudPtr cloudPtr( new tPointCloud );
			pcl::copyPointCloud( *cloud, *cloudPtr );

			if( m_registration.registerCloud( cloudPtr, m_mapParameters ) )
			{
//				std::cerr << "Starting registration process " << std::endl;

				registration_transform = m_registration.getTransform();

//				pcl::transformPointCloud( cloud, used_cloud, transform );

//				std::cerr << "Registration succeeded"  << std::endl;
			}
			else
			{
//				std::cerr << "reg failed." << std::endl;
				return;
			}
		}
	}

//	ros::WallTime startTime = ros::WallTime::now();

//	std::cerr << "Input cloud frame id: " << cloud->header.frame_id << std::endl;

//	std::cerr << "Input cloud " << (cloud->isOrganized() ? "is " : "is not ") << "organized." << std::endl;

	tPointCloud pc_ground; // segmented ground plane
	tPointCloud pc_nonground; // everything else

	if (m_filterGroundPlane)
	{
		m_filterGround.setCloud(&used_cloud);
		m_filterGround.filter(m_data->getTree());
		pc_ground = *m_filterGround.getGroundPc();
		pc_nonground = *m_filterGround.getNongroundPc();
//		m_filterGround.writeLastRunInfo();

	} else {
		pc_nonground = used_cloud;
		pc_ground.clear();
		pc_ground.header = used_cloud.header;
		pc_nonground.header = used_cloud.header;
	}

	tf::StampedTransform cloudToMapTf, sensorToMapTf;

//	PERROR("Get transforms.");

	// Get transforms
	try {
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_mapParameters.frameId,
				cloud->header.frame_id, cloud->header.stamp, ros::Duration(5));

		m_tfListener.lookupTransform(m_mapParameters.frameId,
				cloud->header.frame_id, cloud->header.stamp, cloudToMapTf);

//		m_tfListener.waitForTransform(m_mapParameters.frameId,
//						m_sensor_frame_id, cloud->header.stamp, ros::Duration(5));

//		m_tfListener.lookupTransform(m_mapParameters.frameId,
//				m_sensor_frame_id, cloud->header.stamp, sensorToMapTf);

	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		PERROR( "Transform error.");
		return;
	}

	// transform clouds to world frame for insertion
	if (m_mapParameters.frameId != cloud->header.frame_id)
	{
		Eigen::Matrix4f c2mTM;

//		PERROR("Transforming.");

		pcl_ros::transformAsMatrix(cloudToMapTf, c2mTM);
		pcl::transformPointCloud(pc_ground, pc_ground, c2mTM);
		pcl::transformPointCloud(pc_nonground, pc_nonground, c2mTM);
	}

	// Compute sensor to octomap transform

	// Use registration transform
	pcl::transformPointCloud( pc_ground, pc_ground, registration_transform );

	pc_ground.header = cloud->header;
	pc_ground.header.frame_id = m_mapParameters.frameId;

	pc_nonground.header = cloud->header;
	pc_nonground.header.frame_id = m_mapParameters.frameId;

    // 2012/12/14: Majkl (trying to solve problem with missing time stamps in all message headers)
	m_DataTimeStamp = cloud->header.stamp;
	ROS_DEBUG("COctoMapPlugin::insertCloud(): Stamp = %f", cloud->header.stamp.toSec());

//	insertScan(sensorToMapTf.getOrigin(), pc_ground, pc_nonground);
    insertScan(cloudToMapTf.getOrigin(), pc_ground, pc_nonground);

	if (m_removeSpecles)
	{
		//degradeSingleSpeckles();
		m_filterSingleSpecles.filter(m_data->getTree());
//		m_filterSingleSpecles.writeLastRunInfo();
	}

//	PERROR("Outdated");
	if (m_bRemoveOutdated && m_bFilterWithInput && cloud->isOrganized())
	{
//		std::cerr << "Raycast filter call" << std::endl;

		m_filterRaycast.setCloud(cloud);
		m_filterRaycast.filter(m_data->getTree());
//		m_filterRaycast.writeLastRunInfo();

//		std::cerr << "Raycast filter call complete" << std::endl;
	}
	else
		m_bNewDataToFilter = true;

//	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Point cloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground).)", pc_ground.size(),
			pc_nonground.size());

//	PERROR("Filtered");
	if (m_removeTester != 0) {
		long removed = doObjectTesting(m_removeTester);

		PERROR( "Removed leafs: " << removed);

		if (removed > 0)
			m_data->getTree().prune();

		--m_testerLifeCounter;

		if (m_testerLifeCounter <= 0) {
			delete m_removeTester;
			m_removeTester = 0;
		}
	}

	// Release lock
	lock.unlock();

//	PERROR("insertCloud: Unlocked.");

	m_bNotFirst = true;

	// Publish new data
	invalidate();

//	PERROR("insertCloud: End");
}

/**
 * Insert pointcloud scan TODO: Modify to add ground
 */
void but_env_model::COctoMapPlugin::insertScan(const tf::Point & sensorOriginTf,
		                                       const tPointCloud & ground,
		                                       const tPointCloud & nonground)
{
	octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

	double maxRange(m_mapParameters.maxRange);
	/*
	 octomap::Pointcloud pcNonground;
	 octomap::pointcloudPCLToOctomap( nonground, pcNonground );
	 m_data->getTree().insertScan( pcNonground, sensorOrigin, maxRange, true, false );
	 */
//	std::cerr << "Sensor origin: " << sensorOriginTf.x() << ", " << sensorOriginTf.y() << ", " << sensorOriginTf.z() << std::endl;
//	std::cerr << "Max range: " << maxRange << std::endl;
//	std::cerr << "Cloud frame id: " << nonground.header.frame_id << std::endl;
//	std::cerr << "Trying to insert cloud. Ground size: " << ground.points.size() << ", Nonground size: " << nonground.points.size() << std::endl;
	m_data->getTree().insertColoredScan(nonground, sensorOrigin, maxRange, true);
}

void but_env_model::COctoMapPlugin::reset(bool clearLoaded)
{
	if( m_bMapLoaded && (!clearLoaded) )
		return;

	// Lock data
	boost::mutex::scoped_lock lock(m_lockData);
	m_data->getTree().clear();
	setDefaults();
}

/**
 * Use pointcloud to raycast filter map
 */
void but_env_model::COctoMapPlugin::filterCloud( tPointCloudConstPtr & cloud)
{
//	std::cerr << "Filter cloud in" << std::endl;

	if (m_bNewDataToFilter && m_bRemoveOutdated && (!m_bFilterWithInput) && cloud->isOrganized())
	{
		// Lock data
		boost::mutex::scoped_lock lock(m_lockData);

//		std::cerr << "Filter cloud " << (cloud->isOrganized() ? "is " : "is not ") << "organized." << std::endl;

//		std::cerr << "Raycast filter call" << std::endl;

		m_filterRaycast.setCloud(cloud);
		m_filterRaycast.filter(m_data->getTree());
//		m_filterRaycast.writeLastRunInfo();

//		std::cerr << "Raycast filter call complete" << std::endl;
	}
}

/**
 * Octomap aging that decreases probabilities in time
 */
void but_env_model::COctoMapPlugin::agingCallback(const ros::TimerEvent & event)
{
    ROS_INFO_ONCE( "COctoMapPlugin::agingCallback() called" );

    // Lock data
    boost::mutex::scoped_lock lock(m_lockData);

    // If no data, do nothing
    if( !m_bNotFirst || getSize() <= 1 )
    {
        return;
    }

    // Decrease probabilities stored in the octree
    tButServerOcTree & tree( m_data->getTree() );
    but_env_model::tButServerOcTree::leaf_iterator it, itEnd( tree.end_leafs() );
    for( it = tree.begin_leafs(); it != itEnd; ++it )
    {
        if( m_data->getTree().isNodeOccupied(*it) )
        {
            m_data->getTree().updateNodeLogOdds(&*it, m_agingProbLog);
        }
    }

    // Compress the octree
    m_data->getTree().prune();

    // If the point cloud was not published within a required period, try to publish it again
    // and update the timestamp
    ros::Time ctime = ros::Time::now();
    ros::Duration delay = ctime - m_mapParameters.currentTime;
    bool bPublish = (delay.toSec() > 2 * m_agingPeriod) ? true : false;
    if( bPublish )
    {
        ROS_INFO_STREAM( "Delay = " << delay.toSec() );
        m_DataTimeStamp = ctime;
//        fillMapParameters(ctime);
    }

    // Release lock
    lock.unlock();

    // Publish new data
    if( bPublish )
    {
        invalidate();
    }
}


///////////////////////////////////////////////////////////////////////////////
//  OCTOMAP CRAWLING
///////////////////////////////////////////////////////////////////////////////

/// Crawl octomap
void but_env_model::COctoMapPlugin::crawl(const ros::Time & currentTime)
{
	// Fill needed structures
	fillMapParameters(currentTime);

	// Call new data signal
	m_sigOnNewData( m_mapParameters );
}

//! Should plugin publish data?
bool but_env_model::COctoMapPlugin::shouldPublish()
{
	return (m_bPublishOctomap && m_ocPublisher.getNumSubscribers() > 0);
}

/**
 * Publishing function
 */
void but_env_model::COctoMapPlugin::publishInternal(const ros::Time & timestamp)
{
    ROS_INFO_ONCE( "COctoMapPlugin::publishInternal() called" );

	if( !shouldPublish() )
		return;

	// Lock data
	boost::mutex::scoped_lock lock(m_lockData);

//	octomap_ros::OctomapBinary map;
//	map.header.frame_id = m_mapParameters.frameId;
//	map.header.stamp = timestamp;
//	octomap::octomapMapToMsgData(m_data->getTree(), map.data);

    octomap_msgs::Octomap map;
    octomap_msgs::binaryMapToMsg(m_data->getTree(), map);
    map.header.frame_id = m_mapParameters.frameId;
    map.header.stamp = timestamp;

	m_ocPublisher.publish(map);

//	PERROR( "publish: Unlocked");
}

/// Fill map parameters
void but_env_model::COctoMapPlugin::fillMapParameters(const ros::Time & time)
{
	m_mapParameters.currentTime = time;
	m_mapParameters.mapSize = m_data->getTree().size();
	m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Reset octomap - service callback
 *
 */
bool but_env_model::COctoMapPlugin::resetOctomapCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO( "Reset octomap service called..." );

	// When reseting, loaded octomap should be cleared
	reset(true);

    ROS_INFO( "Reset done..." );

	invalidate();

	return true;
}

///////////////////////////////////////////////////////////////////////////////
// Filtering

/**
 * Do octomap testing by object
 */
long int but_env_model::COctoMapPlugin::doObjectTesting(but_env_model::CTestingObjectBase * object)
{
	if (object == 0) {
		PERROR( "Wrong testing object - NULL. ");
		return 0;
	}

	// Create removed nodes counter
	long int counter(0);

	float probMiss(m_data->getTree().getProbMissLog());

	// For all leafs
	for (but_env_model::tButServerOcTree::leaf_iterator it =
			m_data->getTree().begin_leafs(), end = m_data->getTree().end_leafs(); it
			!= end; ++it) {
		// Node is occupied?
		if (m_data->getTree().isNodeOccupied(*it))
		{
			// Node is in testing object
			if (object->isIn(it.getX(), it.getY(), it.getZ())) {
				// "Remove" node
//				m_data->getTree().setNodeColor(it.getKey(), 255, 0, 0, 255);
				(*it).setValue(probMiss);
				//				m_data->getTree().updateNodeLogOdds(&*it, -0.8);
				++counter;
			}

		}
	}

	return counter;
}

/**
 * Remove cube as a service - callback
 */
#define G2EPOINT( gp ) (Eigen::Vector3f( gp.x, gp.y, gp.z ))
#define G2EQUAT( gp ) (Eigen::Quaternionf( gp.x, gp.y, gp.z, gp.w ))
bool but_env_model::COctoMapPlugin::removeCubeCB(
		but_env_model::RemoveCube::Request & req,
		but_env_model::RemoveCube::Response & res) {

		PERROR( "Remove cube from octomap: " << req.pose << " --- " << req.size );

	// Debug - show cube position
	//addCubeGizmo( req.pose, req.size );

	if (m_removeTester != 0)
		delete m_removeTester;

	// Test frame id
	if (req.frame_id != m_mapParameters.frameId) {
		// Transform pose
		geometry_msgs::PoseStamped ps, psout;
		ps.header.frame_id = req.frame_id;
		ps.header.stamp = m_mapParameters.currentTime;
		ps.pose = req.pose;

		m_tfListener.transformPose(m_mapParameters.frameId, ps, psout);
		req.pose = psout.pose;

		// Transform size
		geometry_msgs::PointStamped vs, vsout;
		vs.header.frame_id = req.frame_id;
		vs.header.stamp = m_mapParameters.currentTime;
		vs.point = req.size;

		m_tfListener.transformPoint(m_mapParameters.frameId, vs, vsout);
		req.size = vsout.point;

				PERROR( "Transformed cube from octomap: " << req.pose << " --- " << req.size );
	}

	// Add bit of size
	double d(m_mapParameters.resolution);

	// Create new tester
	m_removeTester = new but_env_model::CTestingPolymesh(
			G2EPOINT( req.pose.position ), G2EQUAT( req.pose.orientation ),
			G2EPOINT( req.size ) + Eigen::Vector3f(d, d, d));

	// Set it to life
	m_testerLifeCounter = m_testerLife;

	return true;
}

bool but_env_model::COctoMapPlugin::addCubeCB(
		but_env_model::AddCube::Request & req,
		but_env_model::AddCube::Response & res) {

		PERROR( "Add cube to octomap: " << req.pose << " --- \n size: \n"  << req.size );

	// Debug - show cube position
	//addCubeGizmo( req.pose, req.size );

	// Test frame id
	if (req.frame_id != m_mapParameters.frameId) {
		// Transform pose
		geometry_msgs::PoseStamped ps, psout;
		ps.header.frame_id = req.frame_id;
		ps.header.stamp = m_mapParameters.currentTime;
		ps.pose = req.pose;

		m_tfListener.transformPose(m_mapParameters.frameId, ps, psout);
		req.pose = psout.pose;

		// Transform size
		geometry_msgs::PointStamped vs, vsout;
		vs.header.frame_id = req.frame_id;
		vs.header.stamp = m_mapParameters.currentTime;
		vs.point = req.size;

		m_tfListener.transformPoint(m_mapParameters.frameId, vs, vsout);
		req.size = vsout.point;

		// PERROR( "Transformed cube from octomap: " << req.pose << " --- " << req.size );
	}

	PERROR( "Computing sizes..." );

	// Compute minimal and maximal value
	octomap::point3d pmin, pmax;
	pmin(0) = req.pose.position.x - req.size.x * 0.5;
	pmin(1) = req.pose.position.y - req.size.y * 0.5;
	pmin(2) = req.pose.position.z - req.size.z * 0.5;

	pmax(0) = req.pose.position.x + req.size.x * 0.5;
	pmax(1) = req.pose.position.y + req.size.y * 0.5;
	pmax(2) = req.pose.position.z + req.size.z * 0.5;

	PERROR( "Sizes computed. Computing steps ");

	float diff[3];
	unsigned int steps[3];
	for (int i=0;i<3;++i)
	{
	  diff[i] = pmax(i) - pmin(i);
	  steps[i] = floor(diff[i] / m_mapParameters.resolution);

	  std::cerr << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
	}

	PERROR("Rendering... Resolution: " << m_mapParameters.resolution );

	long counter(0);
	octomap::point3d p = pmin;
	octomap::OcTreeKey key;

	for (unsigned int x = 0; x < steps[0]; ++x)
	{
		p.x() += m_mapParameters.resolution;
		p.y() = pmin.y();

		for (unsigned int y = 0; y < steps[1]; ++y)
		{
			p.y() += m_mapParameters.resolution;
			p.z() = pmin.z();

			for (unsigned int z = 0; z < steps[2]; ++z)
			{
				//          std::cout << "querying p=" << p << std::endl;
				p.z() += m_mapParameters.resolution;
				++counter;

				// Try to generate octree key
//				if (!m_data->getTree().genKey(p, key)) continue;
                if (!m_data->getTree().coordToKeyChecked(p, key)) continue;

				// Set node value
				m_data->getTree().updateNode(key, true, true);
				// Set node color
				m_data->getTree().setNodeColor(key, m_r, m_g, m_b, 255);
			}
		}
	}

	PERROR( "Added. Changed nodes: " << counter );
	return true;
}

/**
 * For debugging purpouses - add cubical interactive marker to the scene
 */
void but_env_model::COctoMapPlugin::addCubeGizmo(const geometry_msgs::Pose & pose, const geometry_msgs::Point & size)
{
	but_interaction_primitives::AddUnknownObject gizmo;
	gizmo.request.pose = pose;
	gizmo.request.scale.x = size.x;
	gizmo.request.scale.y = size.y;
	gizmo.request.scale.z = size.z;
	gizmo.request.frame_id = m_mapParameters.frameId;
	gizmo.request.name = "OctomapGizmo";

	ros::service::call("but_interaction_primitives/add_unknown_object", gizmo);
}

/**
 * Pause/resume plugin. All publishers and subscribers are disconnected on pause
 */
void but_env_model::COctoMapPlugin::pause( bool bPause, ros::NodeHandle & nh )
{
	boost::mutex::scoped_lock lock(m_lockData);

	if( bPause )
	{
		m_ocPublisher.shutdown();
//		m_ciSubscriber.shutdown();
//		m_markerPublisher.shutdown();
	}
	else
	{
		m_ocPublisher = nh.advertise<octomap_msgs::Octomap> (m_ocPublisherName, 5, m_latchedTopics);

		// Add camera info subscriber
//		m_ciSubscriber = nh.subscribe(m_camera_info_topic, 10, &but_env_model::COctoMapPlugin::cameraInfoCB, this);

		// If should publish, create markers publisher
//		m_markerPublisher = nh.advertise<visualization_msgs::Marker> (	m_markers_topic_name, 10);
	}
}

/**
 * Set crawling depth - service callback
 */
bool but_env_model::COctoMapPlugin::setCrawlingDepthCB( but_env_model::SetCrawlingDepth::Request & req, but_env_model::SetCrawlingDepth::Response & res )
{
	boost::mutex::scoped_lock lock(m_lockData);
	m_crawlDepth = req.depth;

	// Test maximal value
	unsigned char td( m_data->getTree().getTreeDepth() );
	if( m_crawlDepth > td)
		m_crawlDepth = td;

	m_mapParameters.crawlDepth = m_crawlDepth;

	return true;
}

/**
 * Get octomap tree depth - service callback
 */
bool but_env_model::COctoMapPlugin::getTreeDepthCB( but_env_model::GetTreeDepth::Request & req, but_env_model::GetTreeDepth::Response & res )
{
	res.depth = m_data->getTree().getTreeDepth();

	return true;
}

/**
 * Load map service callback
 */
bool but_env_model::COctoMapPlugin::loadOctreeCB( but_env_model::LoadSaveRequest & req, but_env_model::LoadSaveResponse & res )
{
	// reset data
	reset(true);

	boost::mutex::scoped_lock lock(m_lockData);

	setDefaults();

	assert( m_data != 0 );

	// Set octomap parameters
	m_data->getTree().setProbHit(m_mapParameters.probHit);
	m_data->getTree().setProbMiss(m_mapParameters.probMiss);
	m_data->getTree().setClampingThresMin(m_mapParameters.thresMin);
	m_data->getTree().setClampingThresMax(m_mapParameters.thresMax);
	m_data->getTree().setOccupancyThres(m_mapParameters.thresOccupancy);
	m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();
	m_mapParameters.map = m_data;
	m_mapParameters.crawlDepth = m_crawlDepth;

	// is filename valid?
	if (req.filename.length() > 0) {
		// Try to load data
		if (m_data->getTree().readBinary(req.filename)) {
			ROS_INFO("Octomap file %s loaded (%zu nodes).", req.filename.c_str(), m_data->getTree().size());

			// get tree depth
			m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();

			// get resolution
			m_mapParameters.resolution = m_data->getTree().getResolution();

			// Map was loaded
			m_bMapLoaded = true;

			// Unlock data before invalidation
			lock.unlock();

			// We have new data
			invalidate();

			res.all_ok = true;

			m_bNotFirst = m_data->getTree().getNumLeafNodes() > 0;

		} else {

			// Something is wrong - cannot load data...
			ROS_ERROR("Could not open requested file %s, continuing in standard mode.", req.filename.c_str());
			PERROR( "Transform error.");

			res.all_ok = false;
		}
	}

	return true;
}

/**
 * Load map service callback
 */
bool but_env_model::COctoMapPlugin::saveOctreeCB( but_env_model::LoadSaveRequest & req, but_env_model::LoadSaveResponse & res )
{
	if(req.filename.length() == 0 )
	{
		ROS_ERROR("Wrong filename: Zero length string.");
		res.all_ok = false;
		return false;
	}

	bool rv(m_data->getTree().writeBinaryConst(req.filename));

	if( !rv )
	{
		ROS_ERROR("Could not save file: %s", req.filename.c_str());
		res.all_ok = false;
		return false;
	}

	res.all_ok = true;
	return true;
}

/**
 * Load map service callback
 */
bool but_env_model::COctoMapPlugin::loadFullOctreeCB( but_env_model::LoadSaveRequest & req, but_env_model::LoadSaveResponse & res )
{
	// reset data
	reset(true);

	boost::mutex::scoped_lock lock(m_lockData);

	setDefaults();

	assert( m_data != 0 );

	// Set octomap parameters
	m_data->getTree().setProbHit(m_mapParameters.probHit);
	m_data->getTree().setProbMiss(m_mapParameters.probMiss);
	m_data->getTree().setClampingThresMin(m_mapParameters.thresMin);
	m_data->getTree().setClampingThresMax(m_mapParameters.thresMax);
	m_data->getTree().setOccupancyThres(m_mapParameters.thresOccupancy);
	m_mapParameters.treeDepth = m_data->getTree().getTreeDepth();
	m_mapParameters.map = m_data;
	m_mapParameters.crawlDepth = m_crawlDepth;

	// is filename valid?
	if (req.filename.length() > 0) {
		// Try to load data
		but_env_model::tButServerOcTree * tree = dynamic_cast<tButServerOcTree *>( tButServerOcTree::read( req.filename ) );
		if (tree != 0 )
		{
			// Remove old, create new
			m_data->setTree( tree );
			tree->prune();

			ROS_INFO("Octomap file %s loaded (%zu nodes).", req.filename.c_str(), m_data->getTree().size());

			// get tree depth
			m_mapParameters.treeDepth = tree->getTreeDepth();

			// get resolution
			m_mapParameters.resolution = tree->getResolution();

			// Get hit probability
			m_mapParameters.probHit = tree->getProbHit();

			// Get miss probability
			m_mapParameters.probMiss = tree->getProbMiss();

			// Clamping minimum threshold
			m_mapParameters.thresMin = tree->getClampingThresMin();

			// Clamping threshold maximum
			m_mapParameters.thresMax = tree->getClampingThresMax();

			// Occupancy threshold
			m_mapParameters.thresOccupancy = tree->getOccupancyThres();


			// Map was loaded
			m_bMapLoaded = true;

			// Unlock data before invalidation
			lock.unlock();

			// We have new data
			invalidate();

			res.all_ok = true;

			m_bNotFirst = m_data->getTree().getNumLeafNodes() > 0;

		} else {

			// Something is wrong - cannot load data...
			ROS_ERROR("Could not open requested file %s, continuing in standard mode.", req.filename.c_str());
			PERROR( "Transform error.");

			res.all_ok = false;
		}
	}

	return true;
}

/**
 * Load map service callback
 */
bool but_env_model::COctoMapPlugin::saveFullOctreeCB( but_env_model::LoadSaveRequest & req, but_env_model::LoadSaveResponse & res )
{
	if(req.filename.length() == 0 )
	{
		ROS_ERROR("Wrong filename: Zero length string.");
		res.all_ok = false;
		return false;
	}

	bool rv(m_data->getTree().write(req.filename));

	if( !rv )
	{
		ROS_ERROR("Could not save file: %s", req.filename.c_str());
		res.all_ok = false;
		return false;
	}

	res.all_ok = true;
	return true;
}


/**
 * Starts/stops octomap crowling - service callback
 */
bool but_env_model::COctoMapPlugin::agingPauseCB( but_env_model::OctomapAgingPause::Request & req, but_env_model::OctomapAgingPause::Response & res )
{
    if( req.pause )
    {
        m_agingTimer.stop();
    }
    else
    {
        m_agingTimer.start();
    }

    return true;
}
