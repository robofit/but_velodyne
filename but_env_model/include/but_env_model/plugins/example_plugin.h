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
#pragma once
#ifndef _ExamplePlugin_H_included_
#define _ExamplePlugin_H_included_

#include <srs_env_model/but_server/server_tools.h>

namespace srs_env_model
{

/**
 * Simple plugin - just explaining what methods do...
 */
class CExamplePlugin : public CServerPluginBase
{
public:
	//! Constructor. Plugin name can be used for debug string output.
	CExamplePlugin( const std::string & name = "ExamplePlugin ")
	: CServerPluginBase( name )
	{}

	//! This method is called by server on start. Use node_handle to do
	//! parameters reading and all initializations here (subscriber
	//! connection, publisher creation, etc.)
	virtual void init(ros::NodeHandle & node_handle){}

	//! This method is called when user wants to reset server.
	virtual void reset() {}

protected:
	//!	New scan has been inserted in the octomap. After that
	//! publishing stage starts and if this node has something to
	//! publish (see last method), this method is called.
	virtual void publishInternal(const ros::Time & timestamp){}

	//! Return true, if you want to publish something now. Called
	//! before onPublish method.
	virtual bool shouldPublish(){ return false; }

}; // class CExamplePlugin

/**
 * Simple crawler plugin example.
 * As a crawler template parameter use octomap node type. Additionally this
 * plugin uses data interface to store color used in handleOccupiedNode method.
 */
class CExampleCrawlerPlugin
: public CServerPluginBase
, public COctomapCrawlerBase<tButServerOcTree::NodeType>
, public CDataHolderBase<std::vector<unsigned char> >
{
public:
	//! Constructor. Plugin name can be used for debug string output.
	CExampleCrawlerPlugin( const std::string & name = "ExamplePlugin ")
	: CServerPluginBase( name )
	{
		m_data->resize(3);

		// Store some data
		(*m_data)[0] = 255; (*m_data)[0] = 0; (*m_data)[0] = 0;

	}

	//! Destructor - do cleanup.
	~CExampleCrawlerPlugin() { }

	/// Basic plugin methods are the same
	virtual void init(ros::NodeHandle & node_handle){}

	//! Reset plugin
	virtual void reset() {}

protected:

	//! Set used octomap frame id and timestamp.
	virtual void newMapDataCB( SMapWithParameters & par )
	{
		m_frame_id = par.frameId;
		m_time_stamp = par.currentTime;

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
			else
			{
				handleFreeNode(it, par);
			}

		} // Iterate through octree

		invalidate();
	}


	virtual void publishInternal(const ros::Time & timestamp){}

	/**
	 * We publish no data
	 */
	virtual bool shouldPublish(){ return false; }


	/// Hook that is called when traversing occupied nodes of the updated Octree.
	/// We set node color to the stored one.
	virtual void handleOccupiedNode(srs_env_model::tButServerOcTree::iterator& it, const SMapWithParameters & mp)
	{
		it->r() = (*m_data)[0];
		it->g() = (*m_data)[1];
		it->b() = (*m_data)[2];
	}

	//! Handle free node - set its color to the green.
	virtual void handleFreeNode(tButServerOcTree::iterator & it, const SMapWithParameters & mp )
	{
		it->r() = 0;
		it->g() = 255;
		it->b() = 0;
	}

	/// Called when all nodes was visited.
	virtual void handlePostNodeTraversal(const SMapWithParameters & mp){}


}; // class CExampleCrawlerPlugin




} // namespace srs_env_model


// _ExamplePlugin_H_included_
#endif

