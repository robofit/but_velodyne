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
#ifndef IMarkersPlugin_H_included
#define IMarkersPlugin_H_included

#include <srs_env_model/but_server/server_tools.h>
#include <srs_env_model/AddPlanes.h>
#include <srs_env_model_msgs/PlaneDesc.h>

#include <srs_interaction_primitives/plane.h>
#include <srs_interaction_primitives/services_list.h>
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_interaction_primitives/AddPlane.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace srs_env_model
{

class CIMarkersPlugin : public CServerPluginBase
{
public:
	typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

public:
	/// Constructor
	CIMarkersPlugin(const std::string & name);

	/// Destructor
	virtual ~CIMarkersPlugin();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);


protected:
    /**
     * @brief Insert or modify plane array
     *
     * @param pa Array of planes
     */
    bool insertPlaneCallback( srs_env_model::AddPlanes::Request & req, srs_env_model::AddPlanes::Response & res );

    /**
     * @brief Insert/modify/remove plane
     *
     * @param plane Plane
     */
    void operatePlane( const srs_env_model_msgs::PlaneDesc & plane );

    /**
     * @brief Service helper - add plane
     *
     * @param plane Added plane
     */
    void addPlaneSrvCall( const srs_env_model_msgs::PlaneDesc & plane, const std::string & name );

    /**
     * @brief Service helper - remove plane
     *
     * @param plane Added plane
     */
    void removePlaneSrvCall( const srs_env_model_msgs::PlaneDesc & plane, const std::string & name );

    /**
     *  @brief Get unique string (used as interactive marker name)
     */
    std::string getUniqueName();

    //! Should plugin publish data?
	virtual bool shouldPublish(){ return false; }

	//! Publish data - virtual function
	virtual void publishInternal( const ros::Time & timestamp ){}

protected:
	/// Insert some planes service
    ros::ServiceServer m_serviceInsertPlanes;

    /// Remove object from the interactive markers server pointer
    ros::ServiceClient m_removeInteractiveMarkerService;

    /// Add plane interactive marker service
    ros::ServiceClient m_addInteractivePlaneService;

    //! Used frame id (input data will be transformed to it)
    std::string m_IMarkersFrameId;

    /// Interactive markers server pointer
    srs_interaction_primitives::InteractiveMarkerServerPtr m_imServer;

    // DETECTED ENTITIES
    /// Plane
    typedef std::pair< std::string, srs_env_model_msgs::PlaneDesc > tNamedPlane;
    typedef std::map< int, tNamedPlane > tPlanesMap;
    tPlanesMap m_dataPlanes;

    //! Planes frame id
    std::string m_planesFrameId;

    //! Server topic name
    std::string m_serverTopicName;

    /// Unique name counter
    long int m_uniqueNameCounter;

    /// Use external server services?
    bool m_bUseExternalServer;


}; // class CIMarkersPlugin


} // namespace srs_env_model



 // namespace srs_env_model


// IMarkersPlugin_H_included
#endif

