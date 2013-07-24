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
#ifndef Map2DPlugin_H_included
#define Map2DPlugin_H_included

#include <but_env_model/but_server/server_tools.h>

#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace but_env_model
{

    class CMap2DPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< nav_msgs::OccupancyGrid >
    {
    public:
        /// Constructor
        CMap2DPlugin(const std::string & name);

        /// Destructor
        virtual ~CMap2DPlugin();

        //! Enable or disable publishing
        void enable( bool enabled ){ m_publishMap2D = enabled; }

		//! Initialize plugin - called in server constructor
		virtual void init(ros::NodeHandle & node_handle);

        //! Pause/resume plugin. All publishers and subscribers are disconnected on pause
        virtual void pause( bool bPause, ros::NodeHandle & node_handle );

    protected:
        //! Should plugin publish data?
		bool shouldPublish();

		//! Called when new scan was inserted and now all can be published
		virtual void publishInternal(const ros::Time & timestamp);

		//! Set used octomap frame id and timestamp
		virtual void newMapDataCB( SMapWithParameters & par );

		//! Handle free node (does nothing here)
		virtual void handleFreeNode(tButServerOcTree::iterator & it, const SMapWithParameters & mp );

		/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
		virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapWithParameters & mp);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        //! Is publishing enabled?
        bool m_publishMap2D;

        //! Collision object publisher name
        std::string m_map2DPublisherName;

        /// Collision object publisher
        ros::Publisher m_map2DPublisher;

        //! Transform listener
        tf::TransformListener m_tfListener;

        //
        bool m_latchedTopics;

        //! Used frame id (input data will be transformed to it)
        std::string m_map2DFrameId;

        /// Crawled octomap frame id
        std::string m_ocFrameId;

        /// Transformation from octomap to the collision object frame id - rotation
        Eigen::Matrix3f m_ocToMap2DRot;

        /// Transformation from octomap to the collision object frame id - translation
        Eigen::Vector3f m_ocToMap2DTrans;

        /// Padded key minimum
        octomap::OcTreeKey m_paddedMinKey;

        /// Map limits
        double m_minSizeX;
        double m_minSizeY;

        /// Conversion between frame id's must be done...
        bool m_bConvert;

    }; // class CMap2DPlugin


} // namespace but_env_model



// namespace but_env_model


// Map2DPlugin_H_included
#endif

