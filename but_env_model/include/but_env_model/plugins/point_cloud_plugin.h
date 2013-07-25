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
#ifndef PointCloudPlugin_H_included
#define PointCloudPlugin_H_included

#include <but_env_model/server_tools.h>

#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>

#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/pass_through.h>

// PCL includes
#include <pcl/segmentation/extract_polygonal_prism_data.h>


namespace but_env_model
{

    class CPointCloudPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< tPointCloud >
    {
    public:
    	//! Incomming pointcloud type
    	typedef sensor_msgs::PointCloud2 tIncommingPointCloud;

    public:
        /// Constructor
        CPointCloudPlugin(const std::string & name, bool subscribe );

        /// Destructor
        virtual ~CPointCloudPlugin();

        //! Enable or disable publishing
        void enable( bool enabled ) { m_publishPointCloud = enabled; }

        //! Initialize plugin - called in server constructor
        virtual void init(ros::NodeHandle & node_handle);

        //! Initialize plugin - called in server constructor, enable or disable subscription.
        virtual void init(ros::NodeHandle & node_handle, bool subscribe){ m_bSubscribe = subscribe; init(node_handle); }

        //! Initialize plugin - use given input topic name
        virtual void init(ros::NodeHandle & node_handle, const std::string & topic){ m_pcSubscriberName = topic; init(node_handle); }

        //! Pause/resume plugin. All publishers and subscribers are disconnected on pause
        virtual void pause( bool bPause, ros::NodeHandle & node_handle );

        //! Wants plugin new map data?
        virtual bool wantsMap();

		/// Set frame skip
		void setFrameSkip(unsigned long skip){ m_use_every_nth = skip; }

		/// Set use input color switch value
		void setUseInputColor(bool bUseInputColor) {boost::mutex::scoped_lock lock(m_lockData); m_bUseInputColor = bUseInputColor;}

		/// Set default color value
		void setDefaultColor(uint8_t r, uint8_t g, uint8_t b ){ boost::mutex::scoped_lock lock(m_lockData); m_r = r; m_g = g; m_b = b; }

		/// Filter incomming cloud for NAN's and ground. True is default
		void enableCloudFiltering(bool bEnable) { m_bFilterPC = bEnable; }

		/// Transform incomming cloud to the map frame id? True is default value
		void enableCloudTransform(bool bEnable) { m_bTransformPC = bEnable; }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
      //! Set used octomap frame id and timestamp
      virtual void newMapDataCB( SMapWithParameters & par );

      //! Should plugin publish data?
      virtual bool shouldPublish();

      //! Publish data implementation
      virtual void publishInternal( const ros::Time & timestamp );

      /// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	  void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapWithParameters & mp);

	   /**
        * @brief Insert point cloud callback
        *
        * @param cloud Input point cloud
        */
        void insertCloudCallback(const tIncommingPointCloud::ConstPtr& cloud);

        /**
         * Test if incomming pointcloud2 has rgb part
         */
        bool isRGBCloud( const tIncommingPointCloud::ConstPtr& cloud );

		//! Counts frames and checks if node should publish in this frame
		virtual bool useFrame() { return ++m_frame_number % m_use_every_nth == 0; }

    protected:
        //! Is publishing enabled?
        bool m_publishPointCloud;

        //! Point cloud publisher name
        std::string m_pcPublisherName;

        //! Point cloud subscriber name
        std::string m_pcSubscriberName;

        /// Subscriber - point cloud
        message_filters::Subscriber<tIncommingPointCloud> *m_pcSubscriber;

        //! Message filter (we only want point cloud 2 messages)
        tf::MessageFilter<tIncommingPointCloud> *m_tfPointCloudSub;

        /// Point cloud publisher
        ros::Publisher m_pcPublisher;

        /// Input pointcloud frame id used to filter messages
        std::string m_inputPcFrameId;

        //! Should this plugin subscribe to some publishing topic?
        bool m_bSubscribe;

        //! Transform listener
        tf::TransformListener m_tfListener;

        //
        bool m_latchedTopics;

        //! Do pointcloud filtering?
        bool m_bFilterPC;

        //! Transform pointcloud?
        bool m_bTransformPC;

        //! Minimal Z value
        double m_pointcloudMinZ;

        //! Maximal Z value
        double m_pointcloudMaxZ;

        //! Counter
        long counter;

        //! Pointcloud working mode
        bool m_bAsInput;

        //! Output points transform matrix
        Eigen::Matrix4f m_pcOutTM;

        //! Old point cloud used for registration
        tPointCloudPtr m_oldCloud;

        //! Used buffer cloud
        tPointCloudPtr m_bufferCloud;

		//! Current frame number
		unsigned long m_frame_number;

		//! Use every n-th frame (if m_frame_number modulo m_use_every_nth)
		unsigned long m_use_every_nth;

	    //! Use input color information
	    bool m_bUseInputColor;

	    //! If not using input color use this
	    uint8_t m_r, m_g, m_b;


    }; // class CPointCloudPlugin


} // namespace but_env_model


// PointCloudPubPlugin_H_included
#endif

