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

#include <srs_env_model/but_server/plugins/imarkers_plugin.h>
#include <srs_env_model/topics_list.h>
#include <srs_interaction_primitives/billboard.h>

#include <pcl_ros/transforms.h>


srs_env_model::CIMarkersPlugin::CIMarkersPlugin(const std::string & name)
: srs_env_model::CServerPluginBase(name)
, m_IMarkersFrameId(IM_SERVER_FRAME_ID)
, m_serverTopicName( IM_SERVER_TOPIC_NAME )
, m_uniqueNameCounter(0)
, m_bUseExternalServer( false )
{
}

srs_env_model::CIMarkersPlugin::~CIMarkersPlugin()
{
	if( ! m_bUseExternalServer )
	{
		// Remove all markers
		m_imServer->clear();
	}
}

void srs_env_model::CIMarkersPlugin::init(ros::NodeHandle & node_handle)
{
	// Get interactive markers server topic name
	node_handle.param("im_server_topic_name", m_serverTopicName, m_serverTopicName );

	// Get default frame id
	node_handle.param("im_server_frame_id", m_IMarkersFrameId, m_IMarkersFrameId );

	// Should external server be used
	node_handle.param("im_server_use_external_server", m_bUseExternalServer, false );

	// Use external server or start one?
	if( m_bUseExternalServer )
	{
		// Connect to the services
		m_removeInteractiveMarkerService = node_handle.serviceClient<srs_interaction_primitives::RemovePrimitive> (srs_interaction_primitives::RemovePrimitive_SRV);
		m_addInteractivePlaneService = node_handle.serviceClient<srs_interaction_primitives::AddPlane> (srs_interaction_primitives::AddPlane_SRV);
	}
	else
	{
		// Initialize interactive markers server
		m_imServer.reset(new interactive_markers::InteractiveMarkerServer(m_serverTopicName, "", false));
	}

	// Advertise services
	m_serviceInsertPlanes = node_handle.advertiseService("insert_plane", &srs_env_model::CIMarkersPlugin::insertPlaneCallback, this);


	// Interactive marker server test
	/*
	{
		srs_env_model_msgs::PlaneDesc planedesc;


		// Positioning
		geometry_msgs::Pose p;
		p.position.x = p.position.y = p.position.z = 1.0;
		p.orientation.x = 0.0; p.orientation.y = p.orientation.z = p.orientation.w = 0.6;

		// Scaling
		srs_interaction_primitives::Scale s;
		s.x = s.y = s.z = 10.0;

		// Test plane insertion
		m_planesFrameId = m_IMarkersFrameId;
		planedesc.pose = p;
		planedesc.scale = s;
		planedesc.flags = srs_env_model_msgs::PlaneDesc::INSERT;
		planedesc.id = 0;

		operatePlane( planedesc );

		// Test plane modification
		p.position.x = p.position.y = p.position.z = 1.0;
		p.orientation.x = p.orientation.y = p.orientation.z = p.orientation.w = 0.3;
		planedesc.pose = p;
		planedesc.flags = srs_env_model_msgs::PlaneDesc::MODIFY;

		operatePlane( planedesc );

		PERROR( "Inserting plane");
	}

	m_imServer->applyChanges();

  //*/
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Insert or modify plane array
 *
 * @param pa Array of planes
 */
bool srs_env_model::CIMarkersPlugin::insertPlaneCallback(srs_env_model::AddPlanes::Request & req,
	srs_env_model::AddPlanes::Response & res) {
	std::cerr << "Inset plane called" << std::endl;
	// Get plane array
	srs_env_model_msgs::PlaneArray & planea(req.plane_array);
	m_planesFrameId = planea.header.frame_id;
	std::vector<srs_env_model_msgs::PlaneDesc> & planes(planea.planes);
	std::vector<srs_env_model_msgs::PlaneDesc>::iterator i;

	for (i = planes.begin(); i != planes.end(); ++i) {
		operatePlane(*i);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Insert/modify/remove plane
 *
 * @param plane Plane
 */
void srs_env_model::CIMarkersPlugin::operatePlane(const srs_env_model_msgs::PlaneDesc & plane) {

	std::string name;

	switch (plane.flags) {
	case srs_env_model_msgs::PlaneDesc::INSERT:
		name = getUniqueName();
		m_dataPlanes[plane.id] = tNamedPlane(name, plane);

		addPlaneSrvCall(plane, name);
//		PERROR( "Added plane: " << "Name: " << name << std::endl << plane );

		break;

	case srs_env_model_msgs::PlaneDesc::MODIFY:
		m_dataPlanes[plane.id].second = plane;
		name = m_dataPlanes[plane.id].first;

		// call remove plane, add plane
		removePlaneSrvCall(plane, name);
		addPlaneSrvCall(plane, name);
//		PERROR( "Modified plane: " << "Name: " << name << std::endl << plane );

		break;

	case srs_env_model_msgs::PlaneDesc::REMOVE:
		name = m_dataPlanes[plane.id].first;

		m_dataPlanes.erase(plane.id);
		// call remove plane
		removePlaneSrvCall(plane, name);

//		PERROR( "Removed plane: " << "Name: " << name << std::endl << plane );

		break;

	default:
		break;
	}

	// Apply changes
	if(!m_bUseExternalServer)
		m_imServer->applyChanges();

}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Service helper - add plane
 *
 * @param plane Added plane
 */
void srs_env_model::CIMarkersPlugin::addPlaneSrvCall(const srs_env_model_msgs::PlaneDesc & plane,
		const std::string & name)
{
	if( m_bUseExternalServer )
	{
		// Create service
		srs_interaction_primitives::AddPlane addPlaneSrv;

		// Modify service
		addPlaneSrv.request.name = name;
		addPlaneSrv.request.frame_id = m_planesFrameId;
		addPlaneSrv.request.pose = plane.pose;
		addPlaneSrv.request.scale = plane.scale;
		addPlaneSrv.request.color.r = 1.0;
		addPlaneSrv.request.color.g = 0.0;
		addPlaneSrv.request.color.b = 0.0;
		addPlaneSrv.request.color.a = 0.8;

		m_addInteractivePlaneService.call(addPlaneSrv);
	}
	else
	{
		// Creating Plane object with name "plane1"
		srs_interaction_primitives::Plane *p = new srs_interaction_primitives::Plane(m_imServer, m_IMarkersFrameId, name );

		p->setPose( plane.pose );
		p->setScale( plane.scale );

		// Color
		std_msgs::ColorRGBA c;
		c.r = 1.0;
		c.g = c.b = 0.0;
		c.a = 1.0;
		p->setColor( c );

		p->insert();

	}

}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Service helper - remove plane
 *
 * @param plane Added plane
 */
void srs_env_model::CIMarkersPlugin::removePlaneSrvCall(const srs_env_model_msgs::PlaneDesc & plane,
		const std::string & name)
{
	if( m_bUseExternalServer )
	{
		// Create service
		srs_interaction_primitives::RemovePrimitive removeObjectSrv;

		// Modify service
		removeObjectSrv.request.name = name;

		m_removeInteractiveMarkerService.call(removeObjectSrv);
	}
	else
	{
		m_imServer->erase( name );
	}
}

///////////////////////////////////////////////////////////////////////////////

/**
 *  @brief Get unique string (used as interactive marker name)
 */
std::string srs_env_model::CIMarkersPlugin::getUniqueName() {
	std::stringstream ss;
	ss << "imn" << ++m_uniqueNameCounter;
	return ss.str();
}
