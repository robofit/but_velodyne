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

#include <but_env_model/plugins/map2d_plugin.h>
#include <but_env_model/topics_list.h>

#include <pcl_ros/transforms.h>


but_env_model::CMap2DPlugin::CMap2DPlugin(const std::string & name)
: but_env_model::CServerPluginBase(name)
, m_publishMap2D(true)
, m_map2DPublisherName(MAP2D_PUBLISHER_NAME)
, m_latchedTopics(false)
, m_map2DFrameId(MAP2D_FRAME_ID)
, m_minSizeX(0.0)
, m_minSizeY(0.0)
{
	assert( m_data != 0 );
}



but_env_model::CMap2DPlugin::~CMap2DPlugin()
{
}



bool but_env_model::CMap2DPlugin::shouldPublish()
{
	return( m_publishMap2D && m_map2DPublisher.getNumSubscribers() > 0 );
}



void but_env_model::CMap2DPlugin::init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
{
	private_nh.param("collision_object_publisher", m_map2DPublisherName, MAP2D_PUBLISHER_NAME );
	private_nh.param("collision_object_frame_id", m_map2DFrameId, MAP2D_FRAME_ID );
	private_nh.param("min_x_size", m_minSizeX, m_minSizeX);
	private_nh.param("min_y_size", m_minSizeY, m_minSizeY);

	// Get collision map crawling depth
	int depth(m_crawlDepth);
	private_nh.param("collision_object_octree_depth", depth, depth);
	m_crawlDepth = depth > 0 ? depth : 0;

	// Create publisher
	m_map2DPublisher = nh.advertise<nav_msgs::OccupancyGrid> (m_map2DPublisherName, 5, m_latchedTopics);
}



void but_env_model::CMap2DPlugin::publishInternal(const ros::Time & timestamp)
{
	if( shouldPublish() )
		m_map2DPublisher.publish(*m_data);
}



void but_env_model::CMap2DPlugin::newMapDataCB(SMapWithParameters & par)
{
	m_data->header.frame_id = m_map2DFrameId;
	m_data->header.stamp = par.currentTime;
	m_data->info.resolution = par.resolution;

	m_ocFrameId = par.frameId;
	ros::Time timestamp( par.currentTime );

	tf::StampedTransform ocToMap2DTf;

	m_bConvert = m_ocFrameId != m_map2DFrameId;

	// Get transform
	try {
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_map2DFrameId, m_ocFrameId,
				timestamp, ros::Duration(5));

		m_tfListener.lookupTransform(m_map2DFrameId, m_ocFrameId,
				timestamp, ocToMap2DTf);

	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		PERROR( "Transform error.");
		return;
	}


	Eigen::Matrix4f ocToMap2DTM;

	// Get transformation matrix
	pcl_ros::transformAsMatrix(ocToMap2DTf, ocToMap2DTM);

	const tButServerOcMap &map(*par.map);

	// Disassemble translation and rotation
	m_ocToMap2DRot  = ocToMap2DTM.block<3, 3> (0, 0);
	m_ocToMap2DTrans = ocToMap2DTM.block<3, 1> (0, 3);

	double minX, minY, minZ, maxX, maxY, maxZ;
	map.getTree().getMetricMin(minX, minY, minZ);
	map.getTree().getMetricMax(maxX, maxY, maxZ);

	octomap::point3d minPt(minX, minY, minZ);
	octomap::point3d maxPt(maxX, maxY, maxZ);

	octomap::OcTreeKey minKey, maxKey, curKey;

	// Try to create key
	if (!map.getTree().genKey(minPt, minKey)) {
		ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
		return;
	}

	if (!map.getTree().genKey(maxPt, maxKey)) {
		ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
		return;
	}

	ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

	// add padding if requested (= new min/maxPts in x&y):
	double halfPaddedX = 0.5 * m_minSizeX;
	double halfPaddedY = 0.5 * m_minSizeY;

	minX = std::min(minX, -halfPaddedX);
	maxX = std::max(maxX, halfPaddedX);

	minY = std::min(minY, -halfPaddedY);
	maxY = std::max(maxY, halfPaddedY);

	minPt = octomap::point3d(minX, minY, minZ);
	maxPt = octomap::point3d(maxX, maxY, maxZ);

	octomap::OcTreeKey paddedMaxKey;

	if (!map.getTree().genKey(minPt, m_paddedMinKey)) {
		ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
		return;
	}

	if (!map.getTree().genKey(maxPt, paddedMaxKey)) {
		ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
		return;
	}

	ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1],
			m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);

	assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

	m_data->info.width = paddedMaxKey[0] - m_paddedMinKey[0] + 1;
	m_data->info.height = paddedMaxKey[1] - m_paddedMinKey[1] + 1;

	int mapOriginX = minKey[0] - m_paddedMinKey[0];
	int mapOriginY = minKey[1] - m_paddedMinKey[1];

	assert(mapOriginX >= 0 && mapOriginY >= 0);

	// might not exactly be min / max of octree:
	octomap::point3d origin;
	map.getTree().genCoords(m_paddedMinKey, par.treeDepth, origin);

	m_data->info.origin.position.x = origin.x() - par.resolution* 0.5;
	m_data->info.origin.position.y = origin.y() - par.resolution * 0.5;

	// Allocate space to hold the data (init to unknown)
	m_data->data.resize(m_data->info.width * m_data->info.height, -1);

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

	invalidate();
}

/**
 * Occupied node handler
 */
void but_env_model::CMap2DPlugin::handleOccupiedNode(but_env_model::tButServerOcTree::iterator & it, const SMapWithParameters & mp)
{
	if (it.getDepth() == mp.treeDepth)
	{

		octomap::OcTreeKey nKey = it.getKey(); // TODO: remove intermedate obj (1.4)
		int i = nKey[0] - m_paddedMinKey[0];
		int j = nKey[1] - m_paddedMinKey[1];
		m_data->data[m_data->info.width * j + i] = 100;

	} else {

		int intSize = 1 << (mp.treeDepth - it.getDepth());
		octomap::OcTreeKey minKey = it.getIndexKey();
		for (int dx = 0; dx < intSize; dx++) {
			int i = minKey[0] + dx - m_paddedMinKey[0];
			for (int dy = 0; dy < intSize; dy++) {
				int j = minKey[1] + dy - m_paddedMinKey[1];
				m_data->data[m_data->info.width * j + i] = 100;
			}
		}
	}
}

/**
 * Free node handler
 */
void but_env_model::CMap2DPlugin::handleFreeNode(but_env_model::tButServerOcTree::iterator & it, const SMapWithParameters & mp )
{
	if (it.getDepth() == mp.treeDepth) {
		octomap::OcTreeKey nKey = it.getKey(); //TODO: remove intermedate obj (1.4)
		int i = nKey[0] - m_paddedMinKey[0];
		int j = nKey[1] - m_paddedMinKey[1];
		if (m_data->data[m_data->info.width * j + i] == -1) {
			m_data->data[m_data->info.width * j + i] = 0;
		}
	} else {
		int intSize = 1 << (mp.treeDepth - it.getDepth());
		octomap::OcTreeKey minKey = it.getIndexKey();
		for (int dx = 0; dx < intSize; dx++) {
			int i = minKey[0] + dx - m_paddedMinKey[0];
			for (int dy = 0; dy < intSize; dy++) {
				int j = minKey[1] + dy - m_paddedMinKey[1];
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
void but_env_model::CMap2DPlugin::pause( bool bPause, ros::NodeHandle & nh )
{
	if( bPause )
	{
		m_map2DPublisher.shutdown();
	}
	else
	{
		// Create publisher
		m_map2DPublisher = nh.advertise<nav_msgs::OccupancyGrid> (m_map2DPublisherName, 5, m_latchedTopics);
	}
}
