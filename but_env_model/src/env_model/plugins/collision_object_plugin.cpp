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

#include <srs_env_model/but_server/plugins/collision_object_plugin.h>
#include <srs_env_model/topics_list.h>

#include <pcl_ros/transforms.h>


srs_env_model::CCollisionObjectPlugin::CCollisionObjectPlugin(const std::string & name)
: srs_env_model::CServerPluginBase(name)
, m_publishCollisionObject(true)
, m_coPublisherName(COLLISION_OBJECT_PUBLISHER_NAME)
, m_latchedTopics(false)
, m_coFrameId(COLLISION_OBJECT_FRAME_ID)
, m_bConvert( false )
{
	assert( m_data != 0 );
}



srs_env_model::CCollisionObjectPlugin::~CCollisionObjectPlugin()
{
}



bool srs_env_model::CCollisionObjectPlugin::shouldPublish()
{
	return( m_publishCollisionObject && m_coPublisher.getNumSubscribers() > 0 );
}



void srs_env_model::CCollisionObjectPlugin::init(ros::NodeHandle & node_handle)
{
	node_handle.param("collision_object_publisher", m_coPublisherName, COLLISION_OBJECT_PUBLISHER_NAME );
	node_handle.param("collision_object_frame_id", m_coFrameId, COLLISION_OBJECT_FRAME_ID );
	// Get collision map crawling depth
	int depth(m_crawlDepth);
	node_handle.param("collision_object_octree_depth", depth, depth);
	m_crawlDepth = depth > 0 ? depth : 0;

	// Create publisher
	m_coPublisher = node_handle.advertise<arm_navigation_msgs::CollisionObject> (m_coPublisherName, 5, m_latchedTopics);
}



void srs_env_model::CCollisionObjectPlugin::publishInternal(const ros::Time & timestamp)
{
	if( m_coPublisher.getNumSubscribers() > 0 )
		m_coPublisher.publish(*m_data);
}



void srs_env_model::CCollisionObjectPlugin::newMapDataCB( SMapWithParameters & par )
{
	m_data->header.frame_id = m_coFrameId;
	m_data->header.stamp = par.currentTime;
	m_data->id = "map";

	m_ocFrameId = par.frameId;

	tf::StampedTransform ocToCoTf;

	m_bConvert = m_ocFrameId != m_coFrameId;

	/// We need no transformation - frames are the same...
	if( ! m_bConvert )
	    return;

	// Get transform
	try {
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_coFrameId, m_ocFrameId,
				par.currentTime, ros::Duration(5));

		m_tfListener.lookupTransform(m_coFrameId, m_ocFrameId,
				par.currentTime, ocToCoTf);

	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		PERROR("Transform error.");
		return;
	}

	Eigen::Matrix4f ocToPcTM;

	// Get transformation matrix
	pcl_ros::transformAsMatrix(ocToCoTf, ocToPcTM);

	// Disassemble translation and rotation
	m_ocToCoRot  = ocToPcTM.block<3, 3> (0, 0);
	m_ocToCoTrans = ocToPcTM.block<3, 1> (0, 3);

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

	m_DataTimeStamp = par.currentTime;

	invalidate();
}



void srs_env_model::CCollisionObjectPlugin::handleOccupiedNode(srs_env_model::tButServerOcTree::iterator & it, const SMapWithParameters & mp)
{
	// Transform input point
	Eigen::Vector3f point( it.getX(), it.getY(), it.getZ() );

	// Convert point if needed...
	if( m_bConvert )
	    point = m_ocToCoRot * point + m_ocToCoTrans;

	// Add shape
	arm_navigation_msgs::Shape shape;
	shape.type = arm_navigation_msgs::Shape::BOX;
	shape.dimensions.resize(3);
	shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = it.getSize();
	m_data->shapes.push_back(shape);

	// Add pose
	geometry_msgs::Pose pose;
	pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	pose.position.x = point.x();
	pose.position.y = point.y();
	pose.position.z = point.z();
	m_data->poses.push_back(pose);
}


//! Connect/disconnect plugin to/from all topics
void srs_env_model::CCollisionObjectPlugin::pause( bool bPause, ros::NodeHandle & node_handle)
{
	if( bPause )
		m_coPublisher.shutdown();
	else
		m_coPublisher = node_handle.advertise<arm_navigation_msgs::CollisionObject> (m_coPublisherName, 5, m_latchedTopics);
}
