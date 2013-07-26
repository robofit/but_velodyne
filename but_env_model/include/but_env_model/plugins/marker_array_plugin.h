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
#ifndef MarkerArrayPlugin_H_included
#define MarkerArrayPlugin_H_included

#include <but_env_model/server_tools.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace but_env_model
{

class CMarkerArrayPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< visualization_msgs::MarkerArray >
{
public:
    /// Constructor
    CMarkerArrayPlugin(const std::string & name);

    /// Destructor
    virtual ~CMarkerArrayPlugin();

    //! Enable or disable publishing
    void enable( bool enabled ){ m_publishMarkerArray = enabled; }

    //! Initialize plugin - called in server constructor
    virtual void init(ros::NodeHandle & nh, ros::NodeHandle & private_nh);

     //! Pause/resume plugin. All publishers and subscribers are disconnected on pause
    virtual void pause( bool bPause, ros::NodeHandle & node_handle );

protected:
    //! Called when new scan was inserted and now all can be published
     virtual void publishInternal(const ros::Time & timestamp);

     //! Set used octo map frame id and time stamp
     virtual void newMapDataCB( SMapWithParameters & par );

     /// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
     virtual void handleNode(const tButServerOcTree::iterator& it, const SMapWithParameters & mp);

     /// Called when all nodes was visited.
     virtual void handlePostNodeTraversal(const SMapWithParameters & mp);

     //! Should plugin publish data?
     virtual bool shouldPublish();


     /// Compute color from the height
    std_msgs::ColorRGBA heightMapColor(double h) const;

protected:
    //! Is publishing enabled?
    bool m_publishMarkerArray;

    //! Collision object publisher name
    std::string m_markerArrayPublisherName;

    /// Collision object publisher
    ros::Publisher m_markerArrayPublisher;

    //! Transform listener
    tf::TransformListener m_tfListener;

    //
    bool m_latchedTopics;

    //! Used frame id (input data will be transformed to it)
    std::string m_markerArrayFrameId;

    /// Crawled octomap frame id
    std::string m_ocFrameId;

    /// Transformation from octomap to the collision object frame id - rotation
    Eigen::Matrix3f m_ocToMarkerArrayRot;

    /// Transformation from octomap to the collision object frame id - translation
    Eigen::Vector3f m_ocToMarkerArrayTrans;

    /// Octomap metrics parameters
    double m_minX, m_minY, m_minZ, m_maxX, m_maxY, m_maxZ;

    /// Create marker array as a height map
    bool m_bHeightMap;

    /// Should input data be transformed? (are they in different frames?)
    bool m_bTransform;

    /// Coloring factor
    float m_colorFactor;

    /// Markers color
    std_msgs::ColorRGBA m_color;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class CMarkerArrayPlugin


} // namespace but_env_model

// MarkerArrayPlugin_H_included
#endif
