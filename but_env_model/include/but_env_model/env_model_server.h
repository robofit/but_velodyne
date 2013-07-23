/******************************************************************************
 * \file
 *
 * $Id: but_server.h 2537 2013-02-21 15:57:57Z stancl $
 *
 * Modified by dcgm-robotics@FIT group
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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

#pragma once
#ifndef SERVER_H
#define SERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

//#include <sensor_msgs/CameraInfo.h>
//#include <std_srvs/Empty.h>

//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>

#include <srs_env_model/but_server/plugins/cmap_plugin.h>
#include <srs_env_model/but_server/plugins/point_cloud_plugin.h>
#include <srs_env_model/but_server/plugins/octomap_plugin.h>
#include <srs_env_model/but_server/plugins/collision_object_plugin.h>
#include <srs_env_model/but_server/plugins/imarkers_plugin.h>
#include <srs_env_model/but_server/plugins/marker_array_plugin.h>
#include <srs_env_model/but_server/plugins/limited_point_cloud_plugin.h>
#include <srs_env_model/but_server/plugins/compressed_point_cloud_plugin.h>
#include <srs_env_model/but_server/plugins/objtree_plugin.h>
#include <srs_env_model/but_server/plugins/collision_grid_plugin.h>

#include <srs_env_model/ButServerPause.h>
#include <srs_env_model/UseInputColor.h>

// Old interactive markers plugin used for testing
#include <srs_env_model/but_server/plugins/old_imarkers_plugin.h>

#define _EXAMPLES_
#ifdef _EXAMPLES_
#	include <srs_env_model/but_server/plugins/example_plugin.h>
#endif


namespace srs_env_model
{

/**
  BUT dynamic scene server class.
  */
class CButServer
{
public:
    //! Type of the used pointcloud
    typedef pcl::PointCloud<pcl::PointXYZ> tPCLPointCloud;

    //! Constructor - load file
    CButServer(const std::string& filename= "");

    //! Destructor
    virtual ~CButServer();

    /// Reset server and all plugins
    void reset();

    /// Pause-resume server
    void pause( bool bPause );


protected:
    //! Publish all
    void publishAll(const ros::Time& rostime );

    //! On octomap data changed
    void onOcMapDataChanged( tButServerOcMapConstPtr mapdata, const ros::Time & stamp )
    {
        publishAll(stamp);
    }

    /// On reset service call
    bool onReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        reset();
        return true;
    }

    /// On pause service call
    bool onPause( ButServerPause::Request & request, ButServerPause::Response & response );

    //! Publish all
    void publishPlugins(const ros::Time& rostime);

    /// Use/do not use color service callback
	bool onUseInputColor(srs_env_model::UseInputColor::Request & req, srs_env_model::UseInputColor::Response & res );


protected:
    //! Is server running now
    bool m_bIsPaused;

    /// Node handle
    ros::NodeHandle m_nh;

    bool m_latchedTopics;

    //! Every N-th frame should be processed when including point-cloud.
    int m_numPCFramesProcessed;

    //! Current frame counter
    int m_frameCounter;

    //======================================================================================================
    // Services

    /// Reset service
    ros::ServiceServer m_serviceReset;

    /// Pause service
    ros::ServiceServer m_servicePause;

	/// Use input color service
	ros::ServiceServer m_serviceUseInputColor;

    //======================================================================================================
    // Plugins

    /// All plugins vector type
    typedef std::vector<CServerPluginBase * > tVecPlugins;

    /// All plugins
    tVecPlugins m_plugins;

    /// Call all plugins function
#define FOR_ALL_PLUGINS( X ) { for( tVecPlugins::iterator p = m_plugins.begin(); p != m_plugins.end(); ++p ){ (*p)->X; } }
#define FOR_ALL_PLUGINS_PARAM( X, Y ) { for( tVecPlugins::iterator p = m_plugins.begin(); p != m_plugins.end(); ++p ){ (*p)->X(Y); } }

    /// Collision map
    boost::shared_ptr< CCMapPlugin > m_plugCMap;

    /// Incoming depth points cloud
    boost::shared_ptr< CPointCloudPlugin > m_plugInputPointCloud,
                                    /// Output depth points cloud - whole octomap
                                                        m_plugOcMapPointCloud;
    /// Visible points point cloud
    boost::shared_ptr< CPointCloudPlugin > m_plugVisiblePointCloud;

    /// Octo map plugin
    boost::shared_ptr< COctoMapPlugin > m_plugOctoMap;

    /// Collision object plugin
    boost::shared_ptr< CCollisionObjectPlugin > m_plugCollisionObject;

    /// 2D map plugin
    boost::shared_ptr< CCollisionGridPlugin > m_plugMap2D;

    /// Interactive markers server plugin
    boost::shared_ptr< CIMarkersPlugin > m_plugIMarkers;

    /// Marker array publisher plugin
    boost::shared_ptr< CMarkerArrayPlugin > m_plugMarkerArray;
    
    /// ObjTree plugin
    boost::shared_ptr< CObjTreePlugin > m_plugObjTree;

    /// Old interactive markers plugin
    boost::shared_ptr< COldIMarkersPlugin > m_plugOldIMarkers;

    /// Compressed pointcloud plugin
    boost::shared_ptr< CCompressedPointCloudPlugin > m_plugCompressedPointCloud;

    /// Use old interactive server plugin?				TODO: Remov this when new is will be finished
    bool m_bUseOldIMP;

#ifdef _EXAMPLES_
    /// Create example plugin
    boost::shared_ptr< CExamplePlugin > m_plugExample;

    /// Create crawler plugin holder
    boost::shared_ptr< CExampleCrawlerPlugin > m_plugExampleCrawler;
#endif
};


} // namespace srs_env_model

#endif // SERVER_H
