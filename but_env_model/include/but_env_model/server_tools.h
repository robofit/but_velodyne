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
#pragma once
#ifndef SERVER_TOOLS_H_INCLUDED
#define SERVER_TOOLS_H_INCLUDED

#include <but_env_model/octonode.h>

#include <boost/signal.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>

// Small double number
#define SMALL_DOUBLE double(0.00000001);

//! Absolute value method
template <typename tpType> tpType abs( tpType x ) { return x < 0.0 ? -x : x; }

//! Is absolute value of the given number smaller than SMALL_DOUBLE?
template <typename tpType> bool isSmall( tpType x ) { return abs( x ) < SMALL_DOUBLE; }

//! Is absolute value of the given number greater than SMALL_DOUBLE?
template <typename tpType> bool isGreat( tpType x ) { return abs( x ) > SMALL_DOUBLE; }

namespace but_env_model
{
	//! ROS octomap type
	typedef octomap::ButOctomapROS< but_env_model::EMOcTree > tButServerOcMap;

	//! ROS octomap pointer type
	typedef boost::shared_ptr<tButServerOcMap> tButServerOcMapPtr;

	//! ROS octomap const pointer type
	typedef boost::shared_ptr< const tButServerOcMap > tButServerOcMapConstPtr;

	//! Define octree type
	typedef tButServerOcMap::OcTreeType tButServerOcTree;

	//! Define node type
	typedef tButServerOcTree::NodeType tButServerOcNode;

	//! Define pcl point type
	typedef pcl::PointXYZRGB tPclPoint;

	//! Define pcl point cloud type
	typedef pcl::PointCloud<tPclPoint> tPointCloud;

	//! Point cloud shared pointer
	typedef tPointCloud::Ptr tPointCloudPtr;

	//! Point cloud constant pointer
	typedef tPointCloud::ConstPtr tPointCloudConstPtr;

	/// All needed octo map parameters and something more...
	struct SMapWithParameters
	{
	public:
		/// Octomap resolution
		double resolution;

		/// Octomap tree depth
		unsigned treeDepth;

		/// Maximal tree depth used for crawling
		unsigned char crawlDepth;

		/// Hit probability
		double probHit;

		/// Miss (free cell) probability
		double probMiss;

		/// Clamping minimum
		double thresMin;

		/// Clamping maximum
		double thresMax;

		/// Occupancy threshold
		double thresOccupancy;

		/// Maximal range of valid sensor data
		double maxRange;

		/// Current octomap size
		unsigned mapSize;

		/// Current stamp time
		ros::Time currentTime;

		/// Map frame id
		std::string frameId;

		/// Map pointer
		boost::shared_ptr<tButServerOcMap> map;

	}; // struct SMapParameters.

	///////////////////////////////////////////////////////////////////////////

	//! Server plugin base class
	class CServerPluginBase
	{
	#define PERROR( x ) std::cerr << "Plugin "<< this->m_name << ": " << x << std::endl;

	public:
		//! Constructor
		CServerPluginBase( const std::string & name )
			: m_name(name)
		{ }

		//! Virtual destructor.
		virtual ~CServerPluginBase() {}

		//! Initialize plugin - called in server constructor
		virtual void init(ros::NodeHandle & node_handle)
		{
		}

		//! Called when new scan was inserted and now all can be published
		void publish(const ros::Time & timestamp)
		{
		    if( shouldPublish() ) publishInternal( timestamp );
		}

		//! Reset plugin content when reseting whole server
		virtual void reset() {}

		//! Pause/resume plugin. All publishers and subscribers are disconnected on pause
		virtual void pause( bool bPause, ros::NodeHandle & node_handle ){}

		//! Get plugin name
		std::string getName( ) { return m_name; }


	protected:
		//! Should data be published
		virtual bool shouldPublish() = 0;

		//! Publish data - virtual function
		virtual void publishInternal( const ros::Time & timestamp ) = 0;

	protected:
		//! Plugin name
		std::string m_name;

		//! Locking mutex
		boost::mutex m_lockMutex;
	};


	//! Octomap node crawler policy interface -
	template< class tpNodeType >
	class COctomapCrawlerBase
	{
	public:
		//! Constructor
		COctomapCrawlerBase() : m_crawlDepth(0) {}

		//! Virtual destructor
		virtual ~COctomapCrawlerBase() {}

		//! Handle new octomap data
		void handleNewMapData( SMapWithParameters & par )
		{
		    if( wantsMap() ) newMapDataCB( par );
		}

		//! Wants plugin new map data?
		virtual bool wantsMap() { return true; }

	protected:
		//! New octomap data callback
		virtual void newMapDataCB( SMapWithParameters & par ) = 0;

	protected:
		//! Octomap frame_id
		std::string m_frame_id;

		//! Current timestamp
		ros::Time m_time_stamp;

		/// Maximal depth of tree used when crawling
		unsigned char m_crawlDepth;
	};


	/**
	 * @brief Data holder policy
	 */
	template< class tpDataType >
	class CDataHolderBase
	{
	public:
		/// Data type
		typedef tpDataType tData;

		/// Data pointer type
		typedef boost::shared_ptr< tData > tDataPtr;

		/// Const data pointer type
		typedef boost::shared_ptr< const tData > tDataConstPtr;

		/// Data has changed signal type
		typedef boost::signal< void (tDataConstPtr, const ros::Time & ) > tSigDataHasChanged;

		/// Constructor
		// 2012/12/14 Majkl: Trying to solve problem with missing time stamps
//        CDataHolderBase() : m_data(new tData) {}
//        CDataHolderBase() : m_data(new tData), m_DataTimeStamp(ros::Time(0)) {}
        CDataHolderBase() : m_DataTimeStamp(ros::Time::now()) {m_data = boost::make_shared<tData>();}

		/// Constructor
//        CDataHolderBase( tData * data ) : m_data(data) {}
//        CDataHolderBase( tData * data ) : m_data(data), m_DataTimeStamp(ros::Time(0)) {}
        CDataHolderBase( tData * data ) : m_data(data), m_DataTimeStamp(ros::Time::now()) {}

	public:
		//! Virtual destructor
		virtual ~CDataHolderBase() {}

		//! Get data reference
		tData & getData(){ return *m_data; }

		//! Get constant data reference
		const tData & getData() const { return *m_data; }

		/// Get data has changed signal
		tSigDataHasChanged & getSigDataChanged() { return m_sigDataChanged; }

		/// Data valid
		virtual bool hasValidData() { return m_data != 0; }

		/// Invalidate data - calls invalid signal
		void invalidate()
		{
			if( hasValidData() )
			{
	//			std::cerr << "invalidate: Locked." << std::endl;
				boost::mutex::scoped_lock lock(m_lockData);
				m_sigDataChanged( m_data, m_DataTimeStamp );
	//			std::cerr << "invalidate: Unlocked." << std::endl;
			}
		}

	protected:
		/// Data
		tDataPtr m_data;

		/// Time stamp
		ros::Time m_DataTimeStamp;

		/// Data changed signal
		tSigDataHasChanged m_sigDataChanged;

		/// You can use this mutex to lock data
		boost::mutex m_lockData;
	};

} // namespace but_env_model

/*
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGB,
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(uint32_t, rgb, rgb)
)
*/

// SERVER_TOOLS_H_INCLUDED
#endif
