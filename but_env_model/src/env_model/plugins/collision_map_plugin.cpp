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

#include <but_env_model/plugins/collision_map_plugin.h>
#include <but_env_model/topics_list.h>

#include <pcl_ros/transforms.h>

#define FIXED_FRAME "/map"

but_env_model::CCollisionMapPlugin::CCollisionMapPlugin(const std::string & name)
: but_env_model::CServerPluginBase(name)
, m_cmapPublisherName(COLLISION_MAP_PUBLISHER_NAME)
, m_collisionMapLimitRadius(2.0)
, m_collisionMapVersion(0)
, m_cmapFrameId(COLLISION_MAP_FRAME_ID)
, m_dataBuffer( new tData )
, m_publishCollisionMap( true )
, m_tfListener( ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true )
, m_latchedTopics(false)
, m_bConvertPoint( false )
, m_mapTime(0)
{
}

//! Initialize plugin - called in server constructor
void but_env_model::CCollisionMapPlugin::init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
{
	m_collisionMapVersion = 0;
	m_dataBuffer->boxes.clear();
	m_data->boxes.clear();

	// Read parameters

	// Get collision map radius
	private_nh.param("collision_map_radius", m_collisionMapLimitRadius, COLLISION_MAP_RADIUS_LIMIT );

	// Get collision map topic name
	private_nh.param("collision_map_publisher", m_cmapPublisherName, COLLISION_MAP_PUBLISHER_NAME );

	// Get FID to which will be points transformed when publishing collision map
	private_nh.param("collision_map_frame_id", m_cmapFrameId, COLLISION_MAP_FRAME_ID ); //

	// Get collision map crawling depth
	int depth(m_crawlDepth);
	private_nh.param("collision_map_octree_depth", depth, depth);
	m_crawlDepth = depth > 0 ? depth : 0;

	// Create and publish service - get collision map
	m_serviceGetCollisionMap = nh.advertiseService( GetCollisionMap_SRV, 	&CCollisionMapPlugin::getCollisionMapSrvCallback, this);

	// Create and publish service - is new collision map
	m_serviceIsNewCMap = nh.advertiseService( IsNewCMap_SRV, &CCollisionMapPlugin::isNewCmapSrvCallback, this );

	// Create and publish service - lock map
	m_serviceLockCMap = nh.advertiseService( LockCMap_SRV, &CCollisionMapPlugin::lockCmapSrvCallback, this );

	// Create and publish service - remove cube from map
	m_serviceRemoveCube = nh.advertiseService( RemoveCubeCMP_SRV, &CCollisionMapPlugin::removeBoxCallback, this );

	// Create and publish service - add cube to map
	m_serviceAddCube = nh.advertiseService( AddCubeCMP_SRV, &CCollisionMapPlugin::addBoxCallback, this );

	// Connect publishing services
	pause( false, nh );
}

//! Called when new scan was inserted and now all can be published
void but_env_model::CCollisionMapPlugin::publishInternal(const ros::Time & timestamp)
{
	// Lock data
	boost::mutex::scoped_lock lock(m_lockData);

	// Should map be published?
	bool publishCollisionMap = m_publishCollisionMap && (m_latchedTopics || m_cmapPublisher.getNumSubscribers() > 0);

	// Test collision maps and swap them, if needed
	if( ! sameCMaps( m_data.get(), m_dataBuffer.get() ) )
	{
		// CMaps differs, increase version index and swap them
		++m_collisionMapVersion;
		m_mapTime = timestamp;
		swap( m_data, m_dataBuffer );

		lock.unlock();

		// Call invalidation
		invalidate();
	}

	if( m_bLocked )
	{
		if( m_mapTime != timestamp )
		{
			retransformTF(timestamp);
		}
	}

	// Publish collision map
	if (publishCollisionMap) {
//		std::cerr << "Publishing cmap. Frame id: " << m_cmapFrameId << ", Size: " << m_data->boxes.size() << std::endl;
		m_data->header.frame_id = m_cmapFrameId;
		m_data->header.stamp = timestamp;
		m_cmapPublisher.publish(*m_data);
	}
}

//! Set used octomap frame id and timestamp
void but_env_model::CCollisionMapPlugin::newMapDataCB( SMapWithParameters & par )
{
	if( m_bLocked )
	{
		return;
	}
	// Lock data
	boost::mutex::scoped_lock lock(m_lockData);

	// Reset collision map buffer
	m_dataBuffer->boxes.clear();

	std::string robotBaseFrameId("/base_footprint");

	// Get octomap to collision map transform matrix
	tf::StampedTransform omapToCmapTf,	// Octomap to collision map
						 baseToOmapTf,  // Robot baselink to octomap
						 cmapToWorldTF; // Collision map to world
	try
	{
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_cmapFrameId, par.frameId, par.currentTime, ros::Duration(5));
		m_tfListener.lookupTransform(m_cmapFrameId, par.frameId, par.currentTime, omapToCmapTf);

		m_tfListener.waitForTransform(par.frameId, robotBaseFrameId, par.currentTime, ros::Duration(5));
		m_tfListener.lookupTransform(par.frameId, robotBaseFrameId, par.currentTime, baseToOmapTf);
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		PERROR( "Transform error.");
		return;
	}


	// World TF to the collision map TF
	pcl_ros::transformAsMatrix(omapToCmapTf, m_worldToCMapTM );

	// Disassemble world to cmap translation and rotation
	m_worldToCMapRot  = m_worldToCMapTM.block<3, 3> (0, 0);
	m_worldToCMapTrans = m_worldToCMapTM.block<3, 1> (0, 3);

	m_bConvertPoint = m_cmapFrameId != par.frameId;

	// Compute robot position in the collision map coordinate system
	geometry_msgs::TransformStamped msg;
	tf::transformStampedTFToMsg(baseToOmapTf, msg);
	m_robotBasePosition.setX( msg.transform.translation.x );
	m_robotBasePosition.setY( msg.transform.translation.y );
	m_robotBasePosition.setZ( msg.transform.translation.z );

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

	m_DataTimeStamp = par.currentTime;

	lock.unlock();

	invalidate();
}

/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
void but_env_model::CCollisionMapPlugin::handleOccupiedNode(but_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
{
	if( m_bLocked )
			return;

	// Should we publish something?
	if (! m_publishCollisionMap)
		return;

	// Is this point near enough to the robot?
	if( ! isNearRobot( tf::Vector3( it.getX(), it.getY(), it.getZ() ), it.getSize() ) )
		return;

	Eigen::Vector3f point( it.getX(), it.getY(), it.getZ() );

	if( m_bConvertPoint )
	{
	    // Transform point from the world to the CMap TF
	    point = m_worldToCMapRot * point + m_worldToCMapTrans;
	}

	// Add point to the collision map
	tBox box;

	double size = it.getSize();
	box.extents.x = box.extents.y = box.extents.z = size;
	box.pose.position.x = point[0];
    box.pose.position.y = point[1];
    box.pose.position.z = point[2];
    //box.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, tf::inRadians(-90), 0.0);
    tf::quaternionTFToMsg(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0.0), box.pose.orientation);
//	box.axis.x = box.axis.y = 0.0;
//	box.axis.z = 1.0;
//	box.angle = 0.0;
//	box.center.x = point[0];
//	box.center.y = point[1];
//	box.center.z = point[2];

	m_dataBuffer->boxes.push_back(box);
}

/**
 * @brief Compare two collision maps
 *
 * @param map1 First map to compare
 * @param map2 Second map to compare
 * @return true if maps are the same
 */
bool but_env_model::CCollisionMapPlugin::sameCMaps( moveit_msgs::CollisionMap * map1, moveit_msgs::CollisionMap * map2 )
{
	// Do not swap maps, if in the locked mode
	if( m_bLocked )
		return true;

	// Wrong input
	if( map1 == 0 || map2 == 0 )
	{
//		std::cerr << "Some null. map1: " << map1 << ", map2: " << map2 << std::endl;
		return false;
	}

	// Two maps cannot be the same, if differs in the size
	if( map1->boxes.size() != map2->boxes.size() )
	{
//		std::cerr << "Maps size different. map1: " << map1->boxes.size() << ", map2: " << map2->boxes.size() << std::endl;
		return false;
	}

	// Compare maps box by box
	moveit_msgs::CollisionMap::_boxes_type::iterator icm1, icm2;
	moveit_msgs::CollisionMap::_boxes_type::iterator end1( map1->boxes.end());

	long num( 0 );
	for( icm1 = map1->boxes.begin(), icm2 = map2->boxes.begin(); icm1 != end1; ++icm1, ++icm2 )
	{
//		if( isGreat( icm1->center.x - icm2->center.x ) ||
//				isGreat( icm1->center.y - icm2->center.y ) ||
//				isGreat( icm1->center.z - icm2->center.z ) ||
//				isGreat( icm1->extents.x - icm2->extents.x ) )
        if( isGreat( icm1->pose.position.x - icm2->pose.position.x ) ||
                isGreat( icm1->pose.position.y - icm2->pose.position.y ) ||
                isGreat( icm1->pose.position.z - icm2->pose.position.z ) ||
                isGreat( icm1->extents.x - icm2->extents.x ) )
		{
//			std::cerr << "Point number " << num << " different." << std::endl;
			return false;
		}
		++num;
	}

	return true;
}



/**
 * @brief Test collision object if it is in the collision distance from the robot
 *
 * @param point Position of the object center - in the world coordinates
 * @param extent Bounding sphere of the collision object
 * @return
 */
bool but_env_model::CCollisionMapPlugin::isNearRobot( const tf::Vector3 & point, double extent )
{
	tfScalar s( point.distance( m_robotBasePosition ) );

	return s - extent < m_collisionMapLimitRadius;
}

/**
 * @brief Get collision map service call
 *
 * @param req request - caller's map version
 * @param res response - current map and current version
 */
bool but_env_model::CCollisionMapPlugin::getCollisionMapSrvCallback( but_env_model_msgs::GetCollisionMap::Request & req, but_env_model_msgs::GetCollisionMap::Response & res )
{

	PERROR( "Get collision map service called" );

	// Response map version should be current version number
	res.current_version = m_collisionMapVersion;

	// Get callers map version and compare to the current version number
	if( req.my_version != m_collisionMapVersion )
	{
		res.map = *m_data;
	}else{
		// No new data needed
		res.map = m_dataEmpty;
	}

	return true;
}

/**
 * @brief Returns true, if should be data published and are some subscribers.
 */
bool but_env_model::CCollisionMapPlugin::shouldPublish(  )
{
	return(m_publishCollisionMap && (m_latchedTopics || m_cmapPublisher.getNumSubscribers() > 0));
}

/**
 * @brief Get true if given timestamp is older then current map time
 * @param req request - caller's map timestamp
 * @param res response - true, if new map and current timestamp
 */
bool but_env_model::CCollisionMapPlugin::isNewCmapSrvCallback( but_env_model_msgs::IsNewCollisionMap::Request & req, but_env_model_msgs::IsNewCollisionMap::Response & res )
{
	PERROR( "Is new cmap service called ");

	res.is_newer = req.my_time < m_mapTime;
	res.current_time = m_mapTime;

	return true;
}

/**
 * @brief Lock collision map - disable its updates from new point cloud data
 * @param req request - bool - lock/unlock
 * @param res response -
 */
bool but_env_model::CCollisionMapPlugin::lockCmapSrvCallback( but_env_model_msgs::LockCollisionMap::Request & req, but_env_model_msgs::LockCollisionMap::Response & res )
{
	boost::mutex::scoped_lock lock( m_lockData );

	bool locked( req.lock != 0 );

	m_bLocked = locked;
/*
	if( locked )
		std::cerr << "Locking called. Lock set." << std::endl;
	else
		std::cerr << "Locking called. Lock removed." << std::endl;
*/
	return true;
}

/**
 *  Disconnect plugin from all topics
 */
void but_env_model::CCollisionMapPlugin::pause( bool bPause, ros::NodeHandle & nh )
{
	if( bPause )
		m_cmapPublisher.shutdown();
	else
		m_cmapPublisher = nh.advertise<moveit_msgs::CollisionMap> ( m_cmapPublisherName, 5, m_latchedTopics);
}

/**
 * @brief Remove all collision boxes within given box. Box is aligned with axes and uses the same frame id.
 * @param center Clearing box center
 * @param size Clearing box sizes
 * @return Number of removed boxes.
 */
long but_env_model::CCollisionMapPlugin::removeInsideBox( const tBoxPoint & center, const tBoxPoint & size, tBoxVec & boxes )
{
	boost::mutex::scoped_lock lock( m_lockData );

	// Create data copy
	tBoxVec buffer( boxes.begin(), boxes.end() );

	// Clear output
	boxes.clear();

	// Compute testing box extents
	tBoxPoint extents;
	extents.x = size.x / 2.0; extents.y = size.y / 2.0; extents.z = size.z / 2.0;

	long counter(0);

	// Add excluded boxes to the data
	tBoxVec::iterator it, itEnd( buffer.end() );
	for( it = buffer.begin(); it != itEnd; ++it )
	{
//        if( abs( it->center.x - center.x ) > (it->extents.x + extents.x ) ||
//            abs( it->center.y - center.y ) > (it->extents.y + extents.y ) ||
//            abs( it->center.z - center.z ) > (it->extents.z + extents.z ) )
		if( abs( it->pose.position.x - center.x ) > (it->extents.x + extents.x ) ||
			abs( it->pose.position.y - center.y ) > (it->extents.y + extents.y ) ||
			abs( it->pose.position.z - center.z ) > (it->extents.z + extents.z ) )
		{
			boxes.push_back( *it );
			++counter;
		}

	}

	// Return number of removed boxes
	return buffer.size() - counter;
}

#define POINT_ADD( p0, p1, p2 ) {p0.x = p1.x + p2.x; p0.y = p1.y + p2.y; p0.z = p1.z + p2.z;}
#define POINT_SUB( p0, p1, p2 ) {p0.x = p1.x - p2.x; p0.y = p1.y - p2.y; p0.z = p1.z - p2.z;}
#define POINT_ABS( p0, p1 ) {p0.x = abs(p1.x); p0.y = abs(p1.y); p0.z = abs(p1.z); }

/**
 * @brief Remove all boxes from cubical volume - service callback function
 * @param req Request
 * @param res Response
 */
bool but_env_model::CCollisionMapPlugin::removeBoxCallback( but_env_model_msgs::RemoveCube::Request & req, but_env_model_msgs::RemoveCube::Response & res )
{
	// Test frame id
	if (req.frame_id != m_cmapFrameId)
	{
		// Transform size
		geometry_msgs::PointStamped vs, vsout;
		vs.header.frame_id = req.frame_id;
		vs.header.stamp = m_mapTime;
		POINT_ADD( vs.point, req.size, req.pose.position );
		m_tfListener.transformPoint(m_cmapFrameId, vs, vsout);

		// Transform pose
		geometry_msgs::PoseStamped ps, psout;
		ps.header.frame_id = req.frame_id;
		ps.header.stamp = m_mapTime;
		ps.pose = req.pose;

		m_tfListener.transformPose(m_cmapFrameId, ps, psout);
		req.pose = psout.pose;

		// Finalize size transform
		POINT_SUB(req.size, vsout.point, psout.pose.position);
		POINT_ABS(req.size, req.size);

	}

	tBoxPoint center, size;

	// Convert point type
	center.x = req.pose.position.x; center.y = req.pose.position.y; center.z = req.pose.position.z;
	size.x = req.size.x; size.y = req.size.y; size.z = req.size.z;

	// Remove boxes
	long count =
	removeInsideBox( center, size, m_data->boxes );

	std::cerr << "Removed " << count << " boxes..." << std::endl;

	invalidate();

	return true;
}

/**
 * @brief Adds box to the collision map
 * @param center Box center
 * @param size Box size
 */
void but_env_model::CCollisionMapPlugin::addBox( const tBoxPoint & center, const tBoxPoint & size, tBoxVec & boxes )
{
	boost::mutex::scoped_lock lock( m_lockData );
	tBox box;

//	PERROR( "Adding box: " << std::endl << center << std::endl << size << std::endl )

	box.extents.x = size.x / 2.0;
	box.extents.y = size.y / 2.0;
	box.extents.z = size.z / 2.0;
	box.pose.position.x = center.x;
    box.pose.position.y = center.y;
    box.pose.position.z = center.z;
    tf::quaternionTFToMsg(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0.0), box.pose.orientation);
//	box.axis.x = box.axis.y = 0.0;
//	box.axis.z = 1.0;
//	box.angle = 0.0;
//	box.center.x = center.x;
//	box.center.y = center.y;
//	box.center.z = center.z;

	boxes.push_back(box);
}

/**
 * @brief Remove all boxes from cubical volume - service callback function
 * @param req Request
 * @param res Response
 */
bool but_env_model::CCollisionMapPlugin::addBoxCallback( but_env_model_msgs::RemoveCube::Request & req, but_env_model_msgs::RemoveCube::Response & res )
{
	// Test frame id
	if (req.frame_id != m_cmapFrameId)
	{
		// Transform size
		geometry_msgs::PointStamped vs, vsout;
		vs.header.frame_id = req.frame_id;
		vs.header.stamp = m_mapTime;
		POINT_ADD( vs.point, req.size, req.pose.position );
		m_tfListener.transformPoint(m_cmapFrameId, vs, vsout);

		// Transform pose
		geometry_msgs::PoseStamped ps, psout;
		ps.header.frame_id = req.frame_id;
		ps.header.stamp = m_mapTime;
		ps.pose = req.pose;

		m_tfListener.transformPose(m_cmapFrameId, ps, psout);
		req.pose = psout.pose;

		// Finalize size transform
		POINT_SUB(req.size, vsout.point, psout.pose.position);
		POINT_ABS(req.size, req.size);

	}


	tBoxPoint center, size;

	// Convert point type
	center.x = req.pose.position.x; center.y = req.pose.position.y; center.z = req.pose.position.z;
	size.x = req.size.x; size.y = req.size.y; size.z = req.size.z;

	// Add box
//	std::cerr << "Adding box. " << m_data->boxes.size() << " -> ";
	addBox( center, size, m_data->boxes );
//	std::cerr << m_data->boxes.size() << std::endl;

	invalidate();

	return true;
}

/**
 * @brief retransform map to the new time frame
 * @param currentTime time frame to transform to
 */
void but_env_model::CCollisionMapPlugin::retransformTF( const ros::Time & currentTime )
{
	// Listen and store current collision map transform matrix
	tf::StampedTransform lockedToCurrentTF;

	try
	{
		m_tfListener.waitForTransform( m_cmapFrameId, currentTime, m_cmapFrameId, m_mapTime, FIXED_FRAME, ros::Duration(5) );
		m_tfListener.lookupTransform( m_cmapFrameId, currentTime, m_cmapFrameId, m_mapTime, FIXED_FRAME, lockedToCurrentTF );
	}
	catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			PERROR( "Transform error.");
			return;
	}

	Eigen::Matrix4f m;

	// Get transform matrix
	pcl_ros::transformAsMatrix( lockedToCurrentTF, m );

	// Transform data front buffer
	tBoxVec::iterator it, itEnd( m_data->boxes.end() );
	for( it = m_data->boxes.begin(); it != itEnd; ++it )
	{
		// Copy values to the eigen vectors
//		Eigen::Vector4f position( it->center.x, it->center.y, it->center.z, 1.0f );
        Eigen::Vector4f position( it->pose.position.x, it->pose.position.y, it->pose.position.z, 1.0f );
		Eigen::Vector4f extents( it->extents.x, it->extents.y, it->extents.z, 1.0f );
		extents += position;

		// Retransform
		position = m * position;
		extents = m * extents;

		// Copy values back to the box
//        it->center.x = position[0];
//        it->center.y = position[1];
//        it->center.z = position[2];
		it->pose.position.x = position[0];
		it->pose.position.y = position[1];
		it->pose.position.z = position[2];

		// Recompute extents
		extents = extents - position;
		it->extents.x = abs(extents[0]);
		it->extents.y = abs(extents[1]);
		it->extents.z = abs(extents[2]);
	}

	// Transform back buffer
	itEnd = m_dataBuffer->boxes.end();
	for( it = m_dataBuffer->boxes.begin(); it != itEnd; ++it )
	{
		// Copy values to the eigen vectors
//		Eigen::Vector4f position( it->center.x, it->center.y, it->center.z, 1.0f );
        Eigen::Vector4f position( it->pose.position.x, it->pose.position.y, it->pose.position.z, 1.0f );
		Eigen::Vector4f extents( it->extents.x, it->extents.y, it->extents.z, 1.0f );
		extents += position;

		// Retransform
		position = m * position;
		extents = m * extents;

		// Copy values back to the box
//        it->center.x = position[0];
//        it->center.y = position[1];
//        it->center.z = position[2];
        it->pose.position.x = position[0];
        it->pose.position.y = position[1];
        it->pose.position.z = position[2];

		// Recompute extents
		extents -= position;
		it->extents.x = abs(extents[0]);
		it->extents.y = abs(extents[1]);
		it->extents.z = abs(extents[2]);
	}

	// Store timestamp
	m_mapTime = currentTime;
}
