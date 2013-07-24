/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Jan Gorig (xgorig01@stud.fit.vutbr.cz)
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
#ifndef _BUT_ENV_MODEL_PLUGINS_OBJTREEPLUGIN_H
#define _BUT_ENV_MODEL_PLUGINS_OBJTREEPLUGIN_H

#include <message_filters/subscriber.h>
#include <interactive_markers/interactive_marker_server.h>

#include <but_env_model/but_server/server_tools.h>
#include <but_env_model/but_server/objtree/octree.h>
#include <but_env_model/GetPlane.h>
#include <but_env_model/GetAlignedBox.h>
#include <but_env_model/GetBoundingBox.h>
#include <but_env_model/InsertPlane.h>
#include <but_env_model/InsertAlignedBox.h>
#include <but_env_model/InsertBoundingBox.h>
#include <but_env_model/InsertPlanes.h>
#include <but_env_model/ShowObject.h>
#include <but_env_model/RemoveObject.h>
#include <but_env_model/ShowObjtree.h>
#include <but_env_model/GetObjectsInBox.h>
#include <but_env_model/GetObjectsInHalfspace.h>
#include <but_env_model/GetObjectsInSphere.h>

#include <srs_interaction_primitives/services_list.h>


namespace but_env_model
{

class CObjTreePlugin : public CServerPluginBase
{
public:
    typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

	/// Constructor
	CObjTreePlugin(const std::string & name);

	/// Destructor
	virtual ~CObjTreePlugin();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

    virtual void reset();

    enum Operation
    {
        INSERT,
        UPDATE,
        GET_SIMILAR
    };

    //! Pause/resume plugin. All publishers and subscribers are disconnected on pause
    virtual void pause( bool bPause, ros::NodeHandle & node_handle );

protected:
    //! Should plugin publish data?
    virtual bool shouldPublish(){return false; }
    //! Publish data - virtual function
    virtual void publishInternal( const ros::Time & timestamp ){}

    /// Insert new plane, update if plane with same id exists
    bool srvInsertPlane(but_env_model::InsertPlane::Request &req, but_env_model::InsertPlane::Response &res);
    /// Insert new plane, update if similar plane exists
    bool srvInsertPlaneByPosition(but_env_model::InsertPlane::Request &req, but_env_model::InsertPlane::Response &res);
    /// Get similar plane id
    bool srvGetSimilarPlane(but_env_model::InsertPlane::Request &req, but_env_model::InsertPlane::Response &res);
    /// Insert planes array
    bool srvInsertPlanes(but_env_model::InsertPlanes::Request &req, but_env_model::InsertPlanes::Response &res);
    /// Insert new axis aligned box, update if aligned box with same id exists
    bool srvInsertABox(but_env_model::InsertAlignedBox::Request &req, but_env_model::InsertAlignedBox::Response &res);
    /// Insert new axis aligned box, update if similar aligned box exists
    bool srvInsertABoxByPosition(but_env_model::InsertAlignedBox::Request &req, but_env_model::InsertAlignedBox::Response &res);
    /// Get similar axis aligned box id
    bool srvGetSimilarABox(but_env_model::InsertAlignedBox::Request &req, but_env_model::InsertAlignedBox::Response &res);
    /// Insert new bounding box, update if bounding box with same id exists
    bool srvInsertBBox(but_env_model::InsertBoundingBox::Request &req, but_env_model::InsertBoundingBox::Response &res);
    /// Insert new bounding box, update if similar bounding box exists
    bool srvInsertBBoxByPosition(but_env_model::InsertBoundingBox::Request &req, but_env_model::InsertBoundingBox::Response &res);
    /// Get similar bounding box id
    bool srvGetSimilarBBox(but_env_model::InsertBoundingBox::Request &req, but_env_model::InsertBoundingBox::Response &res);
    /// Show object as interaction primitive
    bool srvShowObject(but_env_model::ShowObject::Request &req, but_env_model::ShowObject::Response &res);
    /// Remove object by id
    bool srvRemoveObject(but_env_model::RemoveObject::Request &req, but_env_model::RemoveObject::Response &res);
    /// Show octree structure
    bool srvShowObjtree(but_env_model::ShowObjtree::Request &req, but_env_model::ShowObjtree::Response &res);
    /// Get information about plane
    bool srvGetPlane(but_env_model::GetPlane::Request &req, but_env_model::GetPlane::Response &res);
    /// Get information about axis aligned box
    bool srvGetABox(but_env_model::GetAlignedBox::Request &req, but_env_model::GetAlignedBox::Response &res);
    /// Get information about bounding box
    bool srvGetBBox(but_env_model::GetBoundingBox::Request &req, but_env_model::GetBoundingBox::Response &res);
    /// Get objects ids from box
    bool srvGetObjectsInBox(but_env_model::GetObjectsInBox::Request &req, but_env_model::GetObjectsInBox::Response &res);
    /// Get objects ids from halfspace
    bool srvGetObjectsInHalfspace(but_env_model::GetObjectsInHalfspace::Request &req, but_env_model::GetObjectsInHalfspace::Response &res);
    /// Get objects ids from sphere
    bool srvGetObjectsInSphere(but_env_model::GetObjectsInSphere::Request &req, but_env_model::GetObjectsInSphere::Response &res);

    //Helper methods
    unsigned int insertPlane(const but_env_model_msgs::PlaneDesc &plane, Operation op);
    unsigned int insertABox(unsigned int id, const geometry_msgs::Point32 &position, const geometry_msgs::Vector3 &scale, Operation op);
    unsigned int insertBBox(unsigned int id, const geometry_msgs::Pose &pose, const geometry_msgs::Vector3 &scale, Operation op);
    void showObject(unsigned int id);
    void removeObject(unsigned int id);
    void showObjtree();
    void getObjects(const objtree::Filter *filter, std::vector<unsigned int> &output);

    //Service servers
    ros::ServiceServer m_serviceGetObjectsInBox;
    ros::ServiceServer m_serviceGetObjectsInHalfspace;
    ros::ServiceServer m_serviceGetObjectsInSphere;
    ros::ServiceServer m_serviceGetPlane;
    ros::ServiceServer m_serviceGetABox;
    ros::ServiceServer m_serviceGetBBox;
    ros::ServiceServer m_serviceInsertPlane;
    ros::ServiceServer m_serviceInsertABox;
    ros::ServiceServer m_serviceInsertBBox;
    ros::ServiceServer m_serviceInsertPlaneByPosition;
    ros::ServiceServer m_serviceInsertABoxByPosition;
    ros::ServiceServer m_serviceInsertBBoxByPosition;
    ros::ServiceServer m_serviceGetSimilarPlane;
    ros::ServiceServer m_serviceGetSimilarABox;
    ros::ServiceServer m_serviceGetSimilarBBox;
    ros::ServiceServer m_serviceInsertPlanes;
    ros::ServiceServer m_serviceShowObject;
    ros::ServiceServer m_serviceShowObjtree;
    ros::ServiceServer m_serviceRemoveObject;
    
    //Service clients
    ros::ServiceClient m_clientAddPlane;
    ros::ServiceClient m_clientAddBoundingBox;
    ros::ServiceClient m_clientRemovePrimitive;

    ros::Publisher m_markerPub;

    objtree::Octree m_octree;

private:
    void publishLine(visualization_msgs::Marker &lines, float x1, float y1, float z1, float x2, float y2, float z2);
    void publishCube(visualization_msgs::Marker &lines, float x, float y, float z, float w, float h, float d);
    void publishOctree(const std::list<objtree::Box> &nodes);

    void removePrimitiveMarker(unsigned int id);
};

}

#endif // _BUT_ENV_MODEL_PLUGINS_OBJTREEPLUGIN_H
