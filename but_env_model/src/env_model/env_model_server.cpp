/******************************************************************************
 * \file
 * $Id: but_server.cpp 809 2012-05-19 21:47:48Z stancl $
 *
 * Modified by Robo@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * This code is derived from the OctoMap server provided by A. Hornung.
 * Please, see the original comments below.
 */

/**
 * octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
 * (inspired by the ROS map_saver)
 * @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
 * @see http://octomap.sourceforge.net/
 * License: BSD
 */

/**
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <but_env_model/env_model_server.h>
#include <but_env_model/topics_list.h>
//#include <but_env_model/server_tools.h>

#include <sstream>

//! Swap function template
//template <typename tpType> void swap( tpType & x, tpType & y){ tpType b(x); x = y; y = b; }

///////////////////////////////////////////////////////////////////////////////
/**
 Constructor
 */
but_env_model::CButServer::CButServer(ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string& filename) :
  m_bIsPaused(false),
  m_nh(nh),
  m_privnh(private_nh),
  m_latchedTopics(false),
  m_numPCFramesProcessed(1.0),
  m_frameCounter(0),
  m_plugCMap(),
  m_plugInputPointCloud(),
  m_plugOcMapPointCloud(),
//    m_plugVisiblePointCloud(),
  m_plugOctoMap(),
  m_plugCollisionObject(),
  m_plugMap2D(),
  m_plugIMarkers(),
  m_plugMarkerArray(),
  m_plugObjTree(),
  m_plugCompressedPointCloud()
#ifdef _EXAMPLES_
  , m_plugExample()
  , m_plugExampleCrawler()
#endif
{
  m_latchedTopics = false;
  m_privnh.param("latch", m_latchedTopics, m_latchedTopics);

  //=========================================================================
  // Create plugins

  ROS_INFO("EnvModelSrv: Initializing plugins...");

//    m_plugCMap = new CCollisionMapPlugin( "CMAP" );
  m_plugInputPointCloud = boost::shared_ptr<CPointCloudPlugin>(new CPointCloudPlugin("PCIN", true));
  m_plugOcMapPointCloud = boost::shared_ptr<CPointCloudPlugin>(new CPointCloudPlugin("PCOC", false));
//    m_plugVisiblePointCloud = boost::shared_ptr<CLimitedPointCloudPlugin>( new CLimitedPointCloudPlugin( "PCVIS") );
  m_plugOctoMap = boost::shared_ptr<COctoMapPlugin>(new COctoMapPlugin("OCM", filename));
//    m_plugCollisionObject = boost::shared_ptr<CCollisionObjectPlugin>( new CCollisionObjectPlugin( "COB") );
  m_plugMap2D = boost::shared_ptr<COccupancyGridPlugin>(new COccupancyGridPlugin("MAP2D"));
//    m_plugMarkerArray = boost::shared_ptr<CMarkerArrayPlugin>( new CMarkerArrayPlugin( "MA") );
//    m_plugObjTree = boost::shared_ptr<CObjTreePlugin>( new CObjTreePlugin( "OT") );
//    m_plugCompressedPointCloud = boost::shared_ptr<CCompressedPointCloudPlugin>( new CCompressedPointCloudPlugin( "CPC") );
//    m_plugIMarkers = boost::shared_ptr<CIMarkersPlugin>( new CIMarkersPlugin( "IM") );

#ifdef _EXAMPLES_
  m_plugExample = boost::shared_ptr<CExamplePlugin>(new CExamplePlugin("Example1"));
  m_plugExampleCrawler = boost::shared_ptr<CExampleCrawlerPlugin>(new CExampleCrawlerPlugin("Example2"));
#endif

  // Store all plugins pointers for easier access
//    m_plugins.push_back( m_plugCMap.get() );
  m_plugins.push_back(m_plugInputPointCloud.get());
  m_plugins.push_back(m_plugOcMapPointCloud.get());
//    m_plugins.push_back( m_plugVisiblePointCloud.get() );
  m_plugins.push_back(m_plugOctoMap.get());
//    m_plugins.push_back( m_plugCollisionObject.get() );
  m_plugins.push_back(m_plugMap2D.get());
//    m_plugins.push_back( m_plugMarkerArray.get() );
//    m_plugins.push_back( m_plugObjTree.get() );
//    m_plugins.push_back( m_plugCompressedPointCloud.get() );
//    m_plugins.push_back( m_plugIMarkers.get() );

#ifdef _EXAMPLES_
  m_plugins.push_back(m_plugExample.get());
  m_plugins.push_back(m_plugExampleCrawler.get());
#endif

  //=========================================================================
  // Initialize plugins

  FOR_ALL_PLUGINS_PARAM2(init, m_nh, m_privnh)

  ROS_INFO("EnvModelSrv: All plugins initialized. Starting server... ");

  // Connect input point cloud with octomap
  m_plugInputPointCloud->getSigDataChanged().connect(boost::bind(&COctoMapPlugin::insertCloud, m_plugOctoMap, _1));

  // Connect all crawlers
//  m_plugOctoMap->getSigOnNewData().connect( boost::bind( &CCollisionMapPlugin::handleNewMapData, m_plugCMap, _1 ) );
  m_plugOctoMap->getSigOnNewData().connect(boost::bind(&CPointCloudPlugin::handleNewMapData, m_plugOcMapPointCloud, _1));
  m_plugOctoMap->getSigOnNewData().connect(boost::bind(&COccupancyGridPlugin::handleNewMapData, m_plugMap2D, _1));
//  m_plugOctoMap->getSigOnNewData().connect( boost::bind( &CMarkerArrayPlugin::handleNewMapData, m_plugMarkerArray, _1 ) );
//  m_plugOctoMap->getSigOnNewData().connect( boost::bind( &CCompressedPointCloudPlugin::handleNewMapData, m_plugCompressedPointCloud, _1 ) );
//  m_plugOctoMap->getSigOnNewData().connect( boost::bind( &CCollisionObjectPlugin::handleNewMapData, m_plugCollisionObject, _1 ) );
//  m_plugOctoMap->getSigOnNewData().connect( boost::bind( &CLimitedPointCloudPlugin::handleNewMapData, m_plugVisiblePointCloud, _1 ) );

  // Connect octomap data changed signal with server publish
  m_plugOctoMap->getSigDataChanged().connect(boost::bind(&CButServer::onOcMapDataChanged, *this, _1, _2));

  //=========================================================================
  // Advertise services

  m_serviceReset = m_privnh.advertiseService(EnvModelReset_SRV, &CButServer::onReset, this);
  m_servicePause = m_privnh.advertiseService(EnvModelPause_SRV, &CButServer::onPause, this);
  m_serviceUseInputColor = m_privnh.advertiseService(EnvModelUseInputColor_SRV, &CButServer::onUseInputColor, this);

} // Constructor


///////////////////////////////////////////////////////////////////////////////
/**
 Destructor
 */
but_env_model::CButServer::~CButServer()
{

}


///////////////////////////////////////////////////////////////////////////////

/**
 Publish all data
 */
void but_env_model::CButServer::publishAll(const ros::Time& rostime)
{
  // Store start time
//  ros::WallTime startTime = ros::WallTime::now();

  // If no data, do nothing
  if (m_plugOctoMap->getSize() <= 1)
  {
    ROS_INFO("Nothing to publish, octree is empty.");
    return;
  }

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;

  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_plugOctoMap->getTreeDepth() + 1);

  //=========================================================================
  // Plugins frame start

  // Crawl octomap
  m_plugOctoMap->crawl(rostime);

  publishPlugins(rostime);

  // Compute and show elapsed time
//  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
//  ROS_DEBUG("Map publishing in CButServer took %f sec", total_elapsed);
}


///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Reset server and all plugins.
 */
void but_env_model::CButServer::reset()
{
  ROS_INFO("Reseting environment server...");

  FOR_ALL_PLUGINS(reset());

  ROS_INFO("Environment server reset finished.");
}

///////////////////////////////////////////////////////////////////////////////
/**
 * On pause service call
 */
bool but_env_model::CButServer::onPause(but_env_model_msgs::EnvModelPause::Request & request, but_env_model_msgs::EnvModelPause::Response & response)
{
  if (request.pause == 0)
  {
    pause(false);
  }
  else
  {
    pause(true);
  }

  return m_bIsPaused;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * Pause-resume server
 */
void but_env_model::CButServer::pause(bool bPause)
{
  if (m_bIsPaused == bPause)
    return;

  tVecPlugins::iterator p, end(m_plugins.end());

  for (p = m_plugins.begin(); p != end; ++p)
  {
    // Majkl 2013/07
//        (*p)->pause( bPause, m_privnh );
    (*p)->pause(bPause, m_nh);
  }

  if (bPause)
  {
    ROS_INFO("Environment server paused.");
  }
  else
  {
    ROS_INFO("Environment server resumed.");
  }

  m_bIsPaused = bPause;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * Publish data
 */
void but_env_model::CButServer::publishPlugins(const ros::Time& rostime)
{
  FOR_ALL_PLUGINS_PARAM(publish, rostime);

}

///////////////////////////////////////////////////////////////////////////////
/**
 * Use/do not use color service callback
 */
bool but_env_model::CButServer::onUseInputColor(but_env_model_msgs::UseInputColor::Request & req, but_env_model_msgs::UseInputColor::Response & res)
{
  // Set value to the pc input plugin
  m_plugInputPointCloud->setUseInputColor(req.use_color);

  return true;
}

