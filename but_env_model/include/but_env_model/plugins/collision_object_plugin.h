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
#ifndef CollisionObjectPlugin_H_included
#define CollisionObjectPlugin_H_included

#include <but_env_model/server_tools.h>

#include <message_filters/subscriber.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/message_filter.h>

namespace but_env_model
{

class CCollisionObjectPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< moveit_msgs::CollisionObject >
{
public:
    /// Constructor
    CCollisionObjectPlugin(const std::string & name);

    /// Destructor
    virtual ~CCollisionObjectPlugin();

    //! Enable or disable publishing
    void enable( bool enabled ){ m_publishCollisionObject = enabled; }

    //! Should plugin publish data?
    bool shouldPublish();

    //! Initialize plugin - called in server constructor
    virtual void init(ros::NodeHandle & node_handle);

    //! Connect/disconnect plugin to/from all topics
    virtual void pause( bool bPause, ros::NodeHandle & node_handle);

protected:
    //! Called when new scan was inserted and now all can be published
    virtual void publishInternal(const ros::Time & timestamp);

    //! Set used octomap frame id and timestamp
    virtual void newMapDataCB( SMapWithParameters & par );

    /// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
    virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapWithParameters & mp);

protected:
    //! Is publishing enabled?
    bool m_publishCollisionObject;

    //! Collision object publisher name
    std::string m_coPublisherName;

    /// Collision object publisher
    ros::Publisher m_coPublisher;

    //! Transform listener
    tf::TransformListener m_tfListener;

    //
    bool m_latchedTopics;

    //! Used frame id (input data will be transformed to it)
    std::string m_coFrameId;

    /// Crawled octomap frame id
    std::string m_ocFrameId;

    /// Transformation from octomap to the collision object frame id - rotation
    Eigen::Matrix3f m_ocToCoRot;

    /// Transformation from octomap to the collision object frame id - translation
    Eigen::Vector3f m_ocToCoTrans;

    /// Does point need to be converted from ocmap frame id to the collision objects frame id?
    bool m_bConvert;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class CCollisionObjectPlugin


} // namespace but_env_model

// CollisionObjectPlugin_H_included
#endif

