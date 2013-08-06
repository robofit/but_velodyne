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

#include <but_env_model/plugins/occupancy_grid_plugin.h>
#include <but_env_model/topics_list.h>

#include <pcl_ros/transforms.h>


but_env_model::COccupancyGridPlugin::COccupancyGridPlugin(const std::string & name)
: but_env_model::CServerPluginBase(name)
, m_publishGrid(true)
, m_gridPublisherName(OCCUPANCYGRID_PUBLISHER_NAME)
, m_latchedTopics(false)
, m_minSizeX(0.0)
, m_minSizeY(0.0)
{
	assert( m_data != 0 );
}



but_env_model::COccupancyGridPlugin::~COccupancyGridPlugin()
{
}



bool but_env_model::COccupancyGridPlugin::shouldPublish()
{
	return( m_publishGrid && m_gridPublisher.getNumSubscribers() > 0 );
}



void but_env_model::COccupancyGridPlugin::init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
{
	private_nh.param("map2d_min_x_size", m_minSizeX, m_minSizeX);
	private_nh.param("map2d_min_y_size", m_minSizeY, m_minSizeY);

	// Get collision map crawling depth
	int depth(m_crawlDepth);
	private_nh.param("map2d_treedepth", depth, depth);
	m_crawlDepth = depth > 0 ? depth : 0;

	// Create publisher
	m_gridPublisher = nh.advertise<nav_msgs::OccupancyGrid> (m_gridPublisherName, 5, m_latchedTopics);
}



void but_env_model::COccupancyGridPlugin::publishInternal(const ros::Time & timestamp)
{
	boost::mutex::scoped_lock lock(m_lockData);

	if( shouldPublish() )
	{
	    ROS_INFO_ONCE( "COccupancyGridPlugin published a 2D map" );

	    m_gridPublisher.publish(*m_data);
	}
}



void but_env_model::COccupancyGridPlugin::newMapDataCB(SMapWithParameters & par)
{
    ROS_INFO_ONCE( "COccupancyGridPlugin::newMapDataCB() called" );

	// init projected 2D map:
	m_data->header.frame_id = par.frameId;
	m_data->header.stamp = par.currentTime;
	m_crawlDepth = par.treeDepth;
	bool sizeChanged(false);

	// TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
	double minX, minY, minZ, maxX, maxY, maxZ;
	par.map->getTree().getMetricMin(minX, minY, minZ);
	par.map->getTree().getMetricMax(maxX, maxY, maxZ);

	octomap::point3d minPt(minX, minY, minZ);
	octomap::point3d maxPt(maxX, maxY, maxZ);
	octomap::OcTreeKey minKey, maxKey, curKey;
//	if (!par.map->getTree().genKey(minPt, minKey)){
    if (!par.map->getTree().coordToKeyChecked(minPt, minKey)){
	  ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
	  return;
	}

//	if (!par.map->getTree().genKey(maxPt, maxKey)){
    if (!par.map->getTree().coordToKeyChecked(maxPt, maxKey)){
	  ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
	  return;
	}
//	par.map->getTree().genKeyAtDepth(minKey, par.treeDepth, minKey);
//	par.map->getTree().genKeyAtDepth(maxKey, par.treeDepth, maxKey);
    minKey = par.map->getTree().adjustKeyAtDepth(minKey, par.treeDepth);
    maxKey = par.map->getTree().adjustKeyAtDepth(maxKey, par.treeDepth);

	ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

	// add padding if requested (= new min/maxPts in x&y):
	double halfPaddedX = 0.5*m_minSizeX;
	double halfPaddedY = 0.5*m_minSizeY;
	minX = std::min(minX, -halfPaddedX);
	maxX = std::max(maxX, halfPaddedX);
	minY = std::min(minY, -halfPaddedY);
	maxY = std::max(maxY, halfPaddedY);
	minPt = octomap::point3d(minX, minY, minZ);
	maxPt = octomap::point3d(maxX, maxY, maxZ);

	octomap::OcTreeKey paddedMaxKey;
//	if (!par.map->getTree().genKey(minPt, m_paddedMinKey)){
    if (!par.map->getTree().coordToKeyChecked(minPt, m_paddedMinKey)){
	  ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
	  return;
	}
//    if (!par.map->getTree().genKey(maxPt, paddedMaxKey)){
	if (!par.map->getTree().coordToKeyChecked(maxPt, paddedMaxKey)){
	  ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
	  return;
	}
//	par.map->getTree().genKeyAtDepth(m_paddedMinKey, par.treeDepth, m_paddedMinKey);
//	par.map->getTree().genKeyAtDepth(paddedMaxKey, par.treeDepth, paddedMaxKey);
	m_paddedMinKey = par.map->getTree().adjustKeyAtDepth(m_paddedMinKey, par.treeDepth);
	paddedMaxKey = par.map->getTree().adjustKeyAtDepth(paddedMaxKey, par.treeDepth);

	ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
	assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

	m_multires2DScale = 1 << (par.treeDepth - m_crawlDepth);
	unsigned int newWidth = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
	unsigned int newHeight = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

	if (newWidth != m_data->info.width || newHeight != m_data->info.height)
	  sizeChanged = true;

	m_data->info.width = newWidth;
	m_data->info.height = newHeight;
	int mapOriginX = minKey[0] - m_paddedMinKey[0];
	int mapOriginY = minKey[1] - m_paddedMinKey[1];
	assert(mapOriginX >= 0 && mapOriginY >= 0);

	// might not exactly be min / max of octree:
	octomap::point3d origin;
//	par.map->getTree().genCoords(m_paddedMinKey, m_crawlDepth, origin);
	origin = par.map->getTree().keyToCoord(m_paddedMinKey, m_crawlDepth);
	double gridRes = par.map->getTree().getNodeSize(par.treeDepth);
	m_data->info.resolution = gridRes;
	m_data->info.origin.position.x = origin.x() - gridRes*0.5;
	m_data->info.origin.position.y = origin.y() - gridRes*0.5;

//	std::cerr << "Origin: " << origin << ", grid resolution: " << gridRes << ", computed origin: "<< mapOriginX << ".." << mapOriginY << std::endl;

	if (par.treeDepth != m_crawlDepth){
		m_data->info.origin.position.x -= par.resolution/2.0;
		m_data->info.origin.position.y -= par.resolution/2.0;
	}

	if (sizeChanged){
	  ROS_INFO("2D grid map size changed to %d x %d", m_data->info.width, m_data->info.height);
	  m_data->data.clear();
	  // init to unknown:
	  m_data->data.resize(m_data->info.width * m_data->info.height, -1);
	}

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
		else
		{
			handleFreeNode( it, par );
		}

	} // Iterate through octree

	m_DataTimeStamp = par.currentTime;

	invalidate();
}

/**
 * Occupied node handler
 */
void but_env_model::COccupancyGridPlugin::handleOccupiedNode(but_env_model::tButServerOcTree::iterator & it, const SMapWithParameters & mp)
{
	if (it.getDepth() == m_crawlDepth)
	{

		octomap::OcTreeKey nKey = it.getKey(); // TODO: remove intermedate obj (1.4)
		int i = (nKey[0] - m_paddedMinKey[0])/m_multires2DScale;;
		int j = (nKey[1] - m_paddedMinKey[1])/m_multires2DScale;;
		m_data->data[m_data->info.width * j + i] = 100;

	} else {

		int intSize = 1 << (m_crawlDepth - it.getDepth());
		octomap::OcTreeKey minKey = it.getIndexKey();
		for (int dx = 0; dx < intSize; dx++) {
			int i = (minKey[0] + dx - m_paddedMinKey[0])/m_multires2DScale;;
			for (int dy = 0; dy < intSize; dy++) {
				int j = (minKey[1] + dy - m_paddedMinKey[1])/m_multires2DScale;;
				m_data->data[m_data->info.width * j + i] = 100;
			}
		}
	}
}

/**
 * Free node handler
 */
void but_env_model::COccupancyGridPlugin::handleFreeNode(but_env_model::tButServerOcTree::iterator & it, const SMapWithParameters & mp )
{
	if (it.getDepth() == m_crawlDepth) {
		octomap::OcTreeKey nKey = it.getKey(); //TODO: remove intermedate obj (1.4)
		int i = (nKey[0] - m_paddedMinKey[0])/m_multires2DScale;;
		int j = (nKey[1] - m_paddedMinKey[1])/m_multires2DScale;
		if (m_data->data[m_data->info.width * j + i] == -1) {
			m_data->data[m_data->info.width * j + i] = 0;
		}
	} else {
		int intSize = 1 << (m_crawlDepth - it.getDepth());
		octomap::OcTreeKey minKey = it.getIndexKey();
		for (int dx = 0; dx < intSize; dx++) {
			int i = (minKey[0] + dx - m_paddedMinKey[0])/m_multires2DScale;
			for (int dy = 0; dy < intSize; dy++) {
				int j = (minKey[1] + dy - m_paddedMinKey[1])/m_multires2DScale;
				if (m_data->data[m_data->info.width * j + i] == -1) {
					m_data->data[m_data->info.width * j + i] = 0;
				}
			}
		}
	}
}

/**
 * Pause/resume plugin. All publishers and subscribers are disconnected on pause
 */
void but_env_model::COccupancyGridPlugin::pause( bool bPause, ros::NodeHandle & nh )
{
	if( bPause )
	{
		m_gridPublisher.shutdown();
	}
	else
	{
		// Create publisher
		m_gridPublisher = nh.advertise<nav_msgs::OccupancyGrid> (m_gridPublisherName, 5, m_latchedTopics);
	}
}
