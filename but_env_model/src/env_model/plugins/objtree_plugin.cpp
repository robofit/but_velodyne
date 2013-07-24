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

#include <Eigen/Geometry>

#include <but_env_model/services_list.h>
#include <but_env_model/topics_list.h>
#include <but_env_model/plugins/objtree_plugin.h>
#include <but_env_model/objtree/plane.h>
#include <but_env_model/objtree/gbbox.h>
#include <but_env_model/objtree/bbox.h>
#include <but_env_model/objtree/filter.h>

#include <but_interaction_primitives/bounding_box.h>
#include <but_interaction_primitives/plane.h>

static const Eigen::Vector3f upVector(0.0f, 0.0f, 1.0f);


namespace but_env_model
{

CObjTreePlugin::CObjTreePlugin(const std::string &name)
    : CServerPluginBase(name), m_octree(objtree::Box(-10.0f, -10.0f, -10.0f, 20.0f, 20.0f, 20.0f))
{
}

CObjTreePlugin::~CObjTreePlugin()
{
    reset();
}

void CObjTreePlugin::init(ros::NodeHandle &node_handle)
{
    //Advertise services
    m_serviceGetObjectsInBox = node_handle.advertiseService(GetObjectsInBox_SRV, &CObjTreePlugin::srvGetObjectsInBox, this);
    m_serviceGetObjectsInHalfspace = node_handle.advertiseService(GetObjectsInHalfspace_SRV, &CObjTreePlugin::srvGetObjectsInHalfspace, this);
    m_serviceGetObjectsInSphere = node_handle.advertiseService(GetObjectsInSphere_SRV, &CObjTreePlugin::srvGetObjectsInSphere, this);
    m_serviceGetPlane = node_handle.advertiseService(GetPlane_SRV, &CObjTreePlugin::srvGetPlane, this);
    m_serviceGetABox = node_handle.advertiseService(GetAlignedBox_SRV, &CObjTreePlugin::srvGetABox, this);
    m_serviceGetBBox = node_handle.advertiseService(GetBoundingBox_SRV, &CObjTreePlugin::srvGetBBox, this);
    m_serviceInsertPlane = node_handle.advertiseService(InsertPlane_SRV, &CObjTreePlugin::srvInsertPlane, this);
    m_serviceInsertPlaneByPosition = node_handle.advertiseService(InsertPlaneByPosition_SRV, &CObjTreePlugin::srvInsertPlaneByPosition, this);
    m_serviceGetSimilarPlane = node_handle.advertiseService(GetSimilarPlane_SRV, &CObjTreePlugin::srvGetSimilarPlane, this);
    m_serviceInsertPlanes = node_handle.advertiseService(InsertPlanes_SRV, &CObjTreePlugin::srvInsertPlanes, this);
    m_serviceInsertABox = node_handle.advertiseService(InsertAlignedBox_SRV, &CObjTreePlugin::srvInsertABox, this);
    m_serviceInsertABoxByPosition = node_handle.advertiseService(InsertAlignedBoxByPosition_SRV, &CObjTreePlugin::srvInsertABoxByPosition, this);
    m_serviceInsertBBox = node_handle.advertiseService(InsertBoundingBox_SRV, &CObjTreePlugin::srvInsertBBox, this);
    m_serviceInsertBBoxByPosition = node_handle.advertiseService(InsertBoundingBoxByPosition_SRV, &CObjTreePlugin::srvInsertBBoxByPosition, this);
    m_serviceGetSimilarABox = node_handle.advertiseService(GetSimilarAlignedBox_SRV, &CObjTreePlugin::srvGetSimilarABox, this);
    m_serviceGetSimilarBBox = node_handle.advertiseService(GetSimilarBoundingBox_SRV, &CObjTreePlugin::srvGetSimilarBBox, this);
    m_serviceRemoveObject = node_handle.advertiseService(RemoveObject_SRV, &CObjTreePlugin::srvRemoveObject, this);
    m_serviceShowObject = node_handle.advertiseService(ShowObject_SRV, &CObjTreePlugin::srvShowObject, this);
    m_serviceShowObjtree = node_handle.advertiseService(ShowObjtree_SRV, &CObjTreePlugin::srvShowObjtree, this);

    m_clientAddPlane = node_handle.serviceClient<srs_interaction_primitives::AddPlane>(srs_interaction_primitives::AddPlane_SRV);
    m_clientAddBoundingBox = node_handle.serviceClient<srs_interaction_primitives::AddBoundingBox>(srs_interaction_primitives::AddBoundingBox_SRV);
    m_clientRemovePrimitive = node_handle.serviceClient<srs_interaction_primitives::RemovePrimitive>(srs_interaction_primitives::RemovePrimitive_SRV);

    m_markerPub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 5);

    printf("ObjTree plugin initialized!\n");
}

void CObjTreePlugin::reset()
{
    std::map<unsigned int, objtree::Object*> objects(m_octree.objectsAll());

    for(std::map<unsigned int, objtree::Object*>::iterator i = objects.begin(); i != objects.end(); i++)
    {
        removePrimitiveMarker(i->second->id());
    }

    m_octree.clear();
}

bool CObjTreePlugin::srvInsertPlane(but_env_model::InsertPlane::Request &req, but_env_model::InsertPlane::Response &res)
{
    res.object_id = insertPlane(req.plane, INSERT);

    showObject(res.object_id);

    return true;
}

bool CObjTreePlugin::srvInsertABox(but_env_model::InsertAlignedBox::Request &req, but_env_model::InsertAlignedBox::Response &res)
{
    res.object_id = insertABox(req.object_id, req.position, req.scale, INSERT);

    showObject(res.object_id);

    return true;
}

bool CObjTreePlugin::srvInsertBBox(but_env_model::InsertBoundingBox::Request &req, but_env_model::InsertBoundingBox::Response &res)
{
    res.object_id = insertBBox(req.object_id, req.pose, req.scale, INSERT);

    showObject(res.object_id);

    return true;
}

bool CObjTreePlugin::srvInsertPlaneByPosition(but_env_model::InsertPlane::Request &req, but_env_model::InsertPlane::Response &res)
{
    if(m_octree.removeObject(req.plane.id))
        removePrimitiveMarker(req.plane.id);

    res.object_id = insertPlane(req.plane, UPDATE);

    if((unsigned int)req.plane.id != res.object_id)
        removePrimitiveMarker(res.object_id);

    showObject(res.object_id);

    return true;
}

bool CObjTreePlugin::srvInsertABoxByPosition(but_env_model::InsertAlignedBox::Request &req, but_env_model::InsertAlignedBox::Response &res)
{
    if(m_octree.removeObject(req.object_id))
        removePrimitiveMarker(req.object_id);

    res.object_id = insertABox(req.object_id, req.position, req.scale, UPDATE);

    if((unsigned int)req.object_id != res.object_id)
        removePrimitiveMarker(res.object_id);

    showObject(res.object_id);

    return true;
}

bool CObjTreePlugin::srvInsertBBoxByPosition(but_env_model::InsertBoundingBox::Request &req, but_env_model::InsertBoundingBox::Response &res)
{
    if(m_octree.removeObject(req.object_id))
        removePrimitiveMarker(req.object_id);

    res.object_id = insertBBox(req.object_id, req.pose, req.scale, UPDATE);

    if((unsigned int)req.object_id != res.object_id)
        removePrimitiveMarker(res.object_id);

    showObject(res.object_id);

    return true;
}

bool CObjTreePlugin::srvGetSimilarPlane(but_env_model::InsertPlane::Request &req, but_env_model::InsertPlane::Response &res)
{
    res.object_id = insertPlane(req.plane, GET_SIMILAR);

    return true;
}

bool CObjTreePlugin::srvGetSimilarABox(but_env_model::InsertAlignedBox::Request &req, but_env_model::InsertAlignedBox::Response &res)
{
    res.object_id = insertABox(req.object_id, req.position, req.scale, GET_SIMILAR);

    return true;
}

bool CObjTreePlugin::srvGetSimilarBBox(but_env_model::InsertBoundingBox::Request &req, but_env_model::InsertBoundingBox::Response &res)
{
    res.object_id = insertBBox(req.object_id, req.pose, req.scale, GET_SIMILAR);

    return true;
}

bool CObjTreePlugin::srvInsertPlanes(but_env_model::InsertPlanes::Request &req, but_env_model::InsertPlanes::Response &res)
{
    std::vector<but_env_model_msgs::PlaneDesc>::iterator i;
    std::vector<but_env_model_msgs::PlaneDesc> &planes(req.plane_array.planes);

    for(i = planes.begin(); i != planes.end(); i++)
    {
        unsigned int id = insertPlane(*i, INSERT);
        res.object_ids.push_back(id);

        showObject(id);
    }

    return true;
}

bool CObjTreePlugin::srvShowObject(but_env_model::ShowObject::Request &req, but_env_model::ShowObject::Response &res)
{
    showObject(req.object_id);

    return true;
}

bool CObjTreePlugin::srvRemoveObject(but_env_model::RemoveObject::Request &req, but_env_model::RemoveObject::Response &res)
{
    removeObject(req.object_id);

    return true;
}

bool CObjTreePlugin::srvShowObjtree(but_env_model::ShowObjtree::Request &req, but_env_model::ShowObjtree::Response &res)
{
    showObjtree();

    return true;
}

bool CObjTreePlugin::srvGetPlane(but_env_model::GetPlane::Request &req, but_env_model::GetPlane::Response &res)
{
    const objtree::Object *object = m_octree.object(req.object_id);
    //Object hasn't been found
    if(!object) return true;
    if(object->type() != objtree::Object::PLANE) return true;

    const objtree::Plane *plane = (const objtree::Plane*)object;

    res.plane.id = req.object_id;
    res.plane.pose.position.x = plane->pos().x;
    res.plane.pose.position.y = plane->pos().y;
    res.plane.pose.position.z = plane->pos().z;

    //Quaternion from normal
    Eigen::Vector3f normal(plane->normal().x, plane->normal().y, plane->normal().z);
    Eigen::Quaternionf q;
    q.setFromTwoVectors(upVector, normal.normalized());

    res.plane.pose.orientation.x = q.x();
    res.plane.pose.orientation.y = q.y();
    res.plane.pose.orientation.z = q.z();
    res.plane.pose.orientation.w = q.w();

    res.plane.scale.x = plane->boundingMax().x-plane->boundingMin().x;
    res.plane.scale.y = plane->boundingMax().y-plane->boundingMin().y;
    res.plane.scale.z = plane->boundingMax().z-plane->boundingMin().z;

    return true;
}

bool CObjTreePlugin::srvGetABox(but_env_model::GetAlignedBox::Request &req, but_env_model::GetAlignedBox::Response &res)
{
    const objtree::Object *object = m_octree.object(req.object_id);
    //Object hasn't been found
    if(!object) return true;
    if(object->type() != objtree::Object::ALIGNED_BOUNDING_BOX) return true;

    const objtree::BBox *box = (const objtree::BBox*)object;

    res.position.x = box->box().x;
    res.position.y = box->box().y;
    res.position.z = box->box().z;

    res.scale.x = box->box().w;
    res.scale.y = box->box().h;
    res.scale.z = box->box().d;

    return true;
}

bool CObjTreePlugin::srvGetBBox(but_env_model::GetBoundingBox::Request &req, but_env_model::GetBoundingBox::Response &res)
{
    const objtree::Object *object = m_octree.object(req.object_id);
    //Object hasn't been found
    if(!object) return true;
    if(object->type() != objtree::Object::GENERAL_BOUNDING_BOX) return true;

    const objtree::GBBox *box = (const objtree::GBBox*)object;

    res.pose.position.x = box->position().x;
    res.pose.position.y = box->position().y;
    res.pose.position.z = box->position().z;

    res.pose.orientation.x = box->orientation().x;
    res.pose.orientation.y = box->orientation().y;
    res.pose.orientation.z = box->orientation().z;
    res.pose.orientation.w = box->orientation().w;

    res.scale.x = box->scale().x;
    res.scale.y = box->scale().y;
    res.scale.z = box->scale().z;

    return true;
}

bool CObjTreePlugin::srvGetObjectsInBox(but_env_model::GetObjectsInBox::Request &req, but_env_model::GetObjectsInBox::Response &res)
{
    objtree::FilterBox filter(objtree::Box(req.position.x, req.position.y, req.position.z, req.size.x, req.size.y, req.size.z));
    getObjects(&filter, res.object_ids);

    return true;
}

bool CObjTreePlugin::srvGetObjectsInHalfspace(but_env_model::GetObjectsInHalfspace::Request &req, but_env_model::GetObjectsInHalfspace::Response &res)
{
    objtree::FilterPlane filter(req.position.x, req.position.y, req.position.z, req.normal.x, req.normal.y, req.normal.z);
    getObjects(&filter, res.object_ids);

    return true;
}

bool CObjTreePlugin::srvGetObjectsInSphere(but_env_model::GetObjectsInSphere::Request &req, but_env_model::GetObjectsInSphere::Response &res)
{
    objtree::FilterSphere filter(req.position.x, req.position.y, req.position.z, req.radius);
    getObjects(&filter, res.object_ids);

    return true;
}

unsigned int CObjTreePlugin::insertPlane(const but_env_model_msgs::PlaneDesc &plane, CObjTreePlugin::Operation op)
{
    printf("insertPlane called, mode %d\n", op);

    if(op == INSERT && m_octree.removeObject(plane.id))
    {
        //Updating existing plane
        removePrimitiveMarker(plane.id);
    }

    objtree::Point pos(plane.pose.position.x, plane.pose.position.y, plane.pose.position.z);
    objtree::Point scale(plane.scale.x, plane.scale.y, plane.scale.z);

    //Normal from orientation - quaternion
    //Quaternion must be normalized!
    Eigen::Quaternionf q(plane.pose.orientation.w, plane.pose.orientation.x, plane.pose.orientation.y, plane.pose.orientation.z);
    Eigen::Vector3f normal = q*upVector;

    objtree::Plane *newPlane = new objtree::Plane(pos, objtree::Vector(normal.x(), normal.y(), normal.z()), scale);
    newPlane->setId(plane.id);

    switch(op)
    {
        case INSERT: return m_octree.insert(newPlane);
        case UPDATE: return m_octree.insertUpdate(newPlane);
        case GET_SIMILAR:
        {
            unsigned int id = -1;
            const objtree::Object *object = m_octree.getSimilarObject(newPlane);
            if(object)
            {
                id = object->id();
            }

            delete newPlane;
            return id;
        }
    }

    return -1;
}

unsigned int CObjTreePlugin::insertABox(unsigned int id, const geometry_msgs::Point32 &position, const geometry_msgs::Vector3 &scale, CObjTreePlugin::Operation op)
{
    printf("insertAlignedBox called, mode %d\n", op);

    if(op == INSERT && m_octree.removeObject(id))
    {
        //Updating existing box
        removePrimitiveMarker(id);
    }

    objtree::BBox *newBox = new objtree::BBox(objtree::Box(position.x, position.y, position.z, scale.x, scale.y, scale.z));
    newBox->setId(id);

    switch(op)
    {
        case INSERT: return m_octree.insert(newBox);
        case UPDATE: return m_octree.insertUpdate(newBox);
        case GET_SIMILAR:
        {
            unsigned int id = -1;
            const objtree::Object *object = m_octree.getSimilarObject(newBox);
            if(object)
            {
                id = object->id();
            }

            delete newBox;
            return id;
        }
    }

    return -1;
}

unsigned int CObjTreePlugin::insertBBox(unsigned int id, const geometry_msgs::Pose &pose, const geometry_msgs::Vector3 &scale, CObjTreePlugin::Operation op)
{
    printf("insertBoundingBox called, mode %d\n", op);

    if(op == INSERT && m_octree.removeObject(id))
    {
        //Updating existing box
        removePrimitiveMarker(id);
    }

    objtree::Point newPosition(pose.position.x, pose.position.y, pose.position.z);
    objtree::Vector4f newOrientation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    objtree::Point newScale(scale.x, scale.y, scale.z);

    //Compute minimal aligned bounding box
    Eigen::Vector3f min, max;
    Eigen::Vector3f vec[4];
    vec[0] = Eigen::Vector3f(-scale.x/2.0f, -scale.y/2.0f, -scale.z/2.0f);
    vec[1] = Eigen::Vector3f(-scale.x/2.0f,  scale.y/2.0f, -scale.z/2.0f);
    vec[2] = Eigen::Vector3f( scale.x/2.0f, -scale.y/2.0f, -scale.z/2.0f);
    vec[3] = Eigen::Vector3f( scale.x/2.0f,  scale.y/2.0f, -scale.z/2.0f);

    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    min = max = q*vec[0];

    for(int i = 1; i < 4; i++)
    {
        vec[i] = q*vec[i];

        for(int j = 0; j < 3; j++)
        {
            if(min[j] > vec[i][j]) min[j] = vec[i][j];
            if(max[j] < vec[i][j]) max[j] = vec[i][j];
        }
    }

    for(int i = 0; i < 4; i++)
    {
        vec[i] = -vec[i];

        for(int j = 0; j < 3; j++)
        {
            if(min[j] > vec[i][j]) min[j] = vec[i][j];
            if(max[j] < vec[i][j]) max[j] = vec[i][j];
        }
    }

    Eigen::Vector3f alignedScale(max-min);
    objtree::Box alignedBox(min[0]+newPosition.x, min[1]+newPosition.y, min[2]+newPosition.z, alignedScale[0], alignedScale[1], alignedScale[2]);

    objtree::GBBox *newBox = new objtree::GBBox(newPosition, newOrientation, newScale, alignedBox);
    newBox->setId(id);

    switch(op)
    {
        case INSERT: return m_octree.insert(newBox);
        case UPDATE: return m_octree.insertUpdate(newBox);
        case GET_SIMILAR:
        {
            unsigned int id = -1;
            const objtree::Object *object = m_octree.getSimilarObject(newBox);
            if(object)
            {
                id = object->id();
            }

            delete newBox;
            return id;
        }
    }

    return -1;
}

void CObjTreePlugin::showObject(unsigned int id)
{
    const objtree::Object *object = m_octree.object(id);
    if(!object) return;

    char name[64];
    snprintf(name, sizeof(name), "imn%u", id);

    switch(object->type())
    {
        case objtree::Object::ALIGNED_BOUNDING_BOX:
        {
            objtree::BBox *box = (objtree::BBox*)object;

            srs_interaction_primitives::AddBoundingBox addBoxSrv;

            addBoxSrv.request.frame_id = IM_SERVER_FRAME_ID;
            addBoxSrv.request.name = name;
            addBoxSrv.request.description = name;

            addBoxSrv.request.pose.position.x = box->box().x+box->box().w/2;
            addBoxSrv.request.pose.position.y = box->box().y+box->box().h/2;
            addBoxSrv.request.pose.position.z = box->box().z+box->box().d/2;

            addBoxSrv.request.pose.orientation.x = 0.0f;
            addBoxSrv.request.pose.orientation.y = 0.0f;
            addBoxSrv.request.pose.orientation.z = 0.0f;
            addBoxSrv.request.pose.orientation.w = 1.0f;

            addBoxSrv.request.scale.x = box->box().w;
            addBoxSrv.request.scale.y = box->box().h;
            addBoxSrv.request.scale.z = box->box().d;

            addBoxSrv.request.color.r = 1.0;
            addBoxSrv.request.color.g = addBoxSrv.request.color.b = 0.0;

            addBoxSrv.request.color.a = 1.0;

            m_clientAddBoundingBox.call(addBoxSrv);
        }
        break;

        case objtree::Object::GENERAL_BOUNDING_BOX:
        {
            objtree::GBBox *box = (objtree::GBBox*)object;

            srs_interaction_primitives::AddBoundingBox addBoxSrv;

            addBoxSrv.request.frame_id = IM_SERVER_FRAME_ID;
            addBoxSrv.request.name = name;
            addBoxSrv.request.description = name;

            addBoxSrv.request.pose.position.x = box->position().x;
            addBoxSrv.request.pose.position.y = box->position().y;
            addBoxSrv.request.pose.position.z = box->position().z;

            addBoxSrv.request.pose.orientation.x = box->orientation().x;
            addBoxSrv.request.pose.orientation.y = box->orientation().y;
            addBoxSrv.request.pose.orientation.z = box->orientation().z;
            addBoxSrv.request.pose.orientation.w = box->orientation().w;

            addBoxSrv.request.scale.x = box->scale().x;
            addBoxSrv.request.scale.y = box->scale().y;
            addBoxSrv.request.scale.z = box->scale().z;

            addBoxSrv.request.color.r = 1.0;
            addBoxSrv.request.color.g = addBoxSrv.request.color.b = 0.0;

            addBoxSrv.request.color.a = 1.0;

            m_clientAddBoundingBox.call(addBoxSrv);
        }
        break;

        case objtree::Object::PLANE:
        {
            objtree::Plane *plane = (objtree::Plane*)object;

            srs_interaction_primitives::AddPlane addPlaneSrv;

            addPlaneSrv.request.frame_id = IM_SERVER_FRAME_ID;
            addPlaneSrv.request.name = name;
            addPlaneSrv.request.description = name;

            addPlaneSrv.request.pose.position.x = plane->pos().x;
            addPlaneSrv.request.pose.position.y = plane->pos().y;
            addPlaneSrv.request.pose.position.z = plane->pos().z;

            //Quaternion from normal
            Eigen::Vector3f normal(plane->normal().x, plane->normal().y, plane->normal().z);
            Eigen::Quaternionf q;
            q.setFromTwoVectors(upVector, normal.normalized());

            addPlaneSrv.request.pose.orientation.x = q.x();
            addPlaneSrv.request.pose.orientation.y = q.y();
            addPlaneSrv.request.pose.orientation.z = q.z();
            addPlaneSrv.request.pose.orientation.w = q.w();

            addPlaneSrv.request.scale.x = plane->boundingMax().x-plane->boundingMin().x;
            addPlaneSrv.request.scale.y = plane->boundingMax().y-plane->boundingMin().y;
            addPlaneSrv.request.scale.z = plane->boundingMax().z-plane->boundingMin().z;

            addPlaneSrv.request.color.r = 1.0;
            addPlaneSrv.request.color.g = addPlaneSrv.request.color.b = 0.0;
            addPlaneSrv.request.color.a = 1.0;

            m_clientAddPlane.call(addPlaneSrv);
        }
        break;
    }
}

void CObjTreePlugin::removeObject(unsigned int id)
{
    m_octree.removeObject(id);
    removePrimitiveMarker(id);
}

void CObjTreePlugin::showObjtree()
{
    std::set<objtree::Object*> objects;
    std::list<objtree::Box> nodes;
    objtree::FilterZero filter;

    m_octree.nodes(nodes, objects, &filter);

    publishOctree(nodes);

    printf("Number of objects %zd\n", objects.size());
}

void CObjTreePlugin::getObjects(const objtree::Filter *filter, std::vector<unsigned int> &output)
{
    std::set<objtree::Object*> objects;

    m_octree.objects(objects, filter);
    output.resize(objects.size());

    unsigned int i = 0;
    for(std::set<objtree::Object*>::iterator it = objects.begin(); it != objects.end(); it++, i++)
    {
        output[i] = (*it)->id();
    }
}

//Methods for octree visualization

void CObjTreePlugin::publishLine(visualization_msgs::Marker &lines, float x1, float y1, float z1, float x2, float y2, float z2)
{
    geometry_msgs::Point p;

    p.x = x1;
    p.y = y1;
    p.z = z1;

    lines.points.push_back(p);

    p.x = x2;
    p.y = y2;
    p.z = z2;

    lines.points.push_back(p);
}

void CObjTreePlugin::publishCube(visualization_msgs::Marker &lines, float x, float y, float z, float w, float h, float d)
{
    publishLine(lines, x, y, z, x+w, y, z);
    publishLine(lines, x, y+h, z, x+w, y+h, z);
    publishLine(lines, x, y, z+d, x+w, y, z+d);
    publishLine(lines, x, y+h, z+d, x+w, y+h, z+d);

    publishLine(lines, x, y, z, x, y+h, z);
    publishLine(lines, x+w, y, z, x+w, y+h, z);
    publishLine(lines, x, y, z+d, x, y+h, z+d);
    publishLine(lines, x+w, y, z+d, x+w, y+h, z+d);

    publishLine(lines, x, y, z, x, y, z+d);
    publishLine(lines, x+w, y, z, x+w, y, z+d);
    publishLine(lines, x, y+h, z, x, y+h, z+d);
    publishLine(lines, x+w, y+h, z, x+w, y+h, z+d);
}

void CObjTreePlugin::removePrimitiveMarker(unsigned int id)
{
    srs_interaction_primitives::RemovePrimitive removePrimitiveSrv;
    char name[64];
    snprintf(name, sizeof(name), "imn%u", id);
    removePrimitiveSrv.request.name = name;

    m_clientRemovePrimitive.call(removePrimitiveSrv);
}

void CObjTreePlugin::publishOctree(const std::list<objtree::Box> &nodes)
{
    visualization_msgs::Marker lines;

    lines.header.frame_id = IM_SERVER_FRAME_ID;
    lines.header.stamp = ros::Time::now();
    lines.ns = "objtree";
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0f;

    lines.id = 0;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.color.r = lines.color.g = lines.color.b = 0.75f;
    lines.color.a = 1.0f;

    lines.scale.x = 0.03f;

    for(std::list<objtree::Box>::const_iterator i = nodes.begin(); i != nodes.end(); i++)
    {
        publishCube(lines, i->x, i->y, i->z, i->w, i->h, i->d);
    }

    m_markerPub.publish(lines);
}

/**
 *  Pause/resume plugin. All publishers and subscribers are disconnected on pause
 */
void CObjTreePlugin::pause( bool bPause, ros::NodeHandle & node_handle )
{
	if( bPause )
		m_markerPub.shutdown();
	else
		m_markerPub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 5);
}

}
