/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Jan Gorig (xgorig01@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 12/04/2012
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

#include <cstdio>
#include <but_env_model/objtree/octree.h>
#include <but_env_model/objtree/filter.h>
#include <but_env_model/objtree/node.h>

namespace objtree
{

/**
 * Default constructor.
 * Creates octree with default sized root node (16.0 x 16.0 x 16.0).
 * @param maxDepth maximum octree depth
 */
Octree::Octree(unsigned int maxDepth) :
    m_rootSize(0.0f, 0.0f, 0.0f, 16.0f, 16.0f, 16.0f)
{
    m_root = new Node;
    m_maxId = 0;
    m_maxDepth = maxDepth;
}

/**
 * A constructor.
 * Creates octree with custom sized root node.
 * @param rootSize root node bounding box
 * @param maxDepth maximum octree depth
 */
Octree::Octree(const Box &rootSize, unsigned int maxDepth) :
    m_rootSize(rootSize)
{
    m_root = new Node;
    m_maxId = 0;
    m_maxDepth = maxDepth;
}

/**
 * A destructor
 */
Octree::~Octree()
{
    delete m_root;
}

/**
 * Clears all objects in octree. Resets octree to default state.
 */
void Octree::clear()
{
    delete m_root;
    m_objects.clear();

    m_root = new Node;
    m_maxId = 0;
}

/**
 * Inserts object into the biggest node that fits entire object.
 * @param object object to insert
 * @return inserted object id
 */
unsigned int Octree::insertOnFit(Object* object)
{
    Node *node = m_root;

    Box box(m_rootSize);
    Box childBox;

    bool fits = true;
    unsigned char i;

    while(fits)
    {
        for(i = 0; !(fits = object->fitsIntoBox(Node::getChildBox(i, childBox, box))) && i < Node::CHILDREN; i++);

        if(fits)
        {
            node = node->child(i, true);
            box = childBox;
        }
    }

    node->add(object);
    m_objects[m_maxId] = object;
    object->setId(m_maxId++);

    return object->id();
}

/**
 * Inserts object into all intersected leaf nodes.
 * @param object object to insert
 * @param node current node in recursion
 * @param box bounding box of current node
 * @param depth current depth
 * @return inserted object id
 */
unsigned int Octree::insertOnInterfere(Object* object, Node *node, Box box, unsigned int depth)
{
    Box childBox;

    for(unsigned char i = 0; i < Node::CHILDREN; i++)
    {
        if(object->interfereWithBox(Node::getChildBox(i, childBox, box)))
        {
            Node *child = node->child(i, true);

            if(depth < m_maxDepth)
            {
                insertOnInterfere(object, child, childBox, depth+1);
            }
            else
            {
                child->add(object);
            }
        }
    }

    return object->id();
}

/**
 * Inserts or updates object into all intersected leaf nodes.
 * Updates object if there exists a similar object. Inserts otherwise.
 * @param object object to insert
 * @param node current node in recursion
 * @param box bounding box of current node
 * @param depth current depth
 * @param inserted has been new object inserted?
 * @return inserted object id
 */
unsigned int Octree::insertUpdateOnInterfere(Object* object, Node *node, Box box, bool &inserted, unsigned int depth)
{
    Box childBox;

    for(unsigned char i = 0; i < Node::CHILDREN; i++)
    {
        if(object->interfereWithBox(Node::getChildBox(i, childBox, box)))
        {
            Node *child = node->child(i, true);

            if(depth < m_maxDepth)
            {
                insertUpdateOnInterfere(object, child, childBox, inserted, depth+1);
            }
            else
            {
                //Find similar object
                Object *similar = NULL;

                for(std::list<Object*>::const_iterator j = child->objects().begin(); j != child->objects().end() && !similar; j++)
                {
                    if(*j != object && (*j)->isSimilar(object))
                    {
                        similar = *j;
                    }
                }

                for(unsigned int n = 0; n < Node::NEIGHBORS && !similar; n++)
                {
                    Node *neighbor = child->neighbor(n);
                    if(neighbor == NULL)
                    {
                        continue;
                    }

                    for(std::list<Object*>::const_iterator j = neighbor->objects().begin(); j != neighbor->objects().end() && !similar; j++)
                    {
                        if(*j != object && (*j)->isSimilar(object))
                        {
                            similar = *j;
                        }
                    }
                }

                child->add(object);

                //We have found a similar object
                if(similar != NULL)
                {
                    //Replace similar object in objects list
                    if(!inserted)
                    {
                        m_objects[similar->id()] = object;
                        object->setId(similar->id());
                        inserted = true;
#if HISTORY_ENABLED
                        object->takeHistory(similar);
#endif
                    }
                    //We have already replaced other object
                    else
                    {
                        m_objects.erase(similar->id());
                    }

                    delete similar;
                }
                //Similar object hasn't been found
                else
                {
                    if(!inserted)
                    {
                        if(!object->hasId())
                        {
                            m_objects[m_maxId] = object;
                            object->setId(m_maxId++);
                        }
                        else
                        {
                            m_objects[object->id()] = object;
                        }
                    }
                }
            }
        }
    }

    return object->id();
}

/**
 * Returns similar object to selected one.
 * @param object object to compare
 * @param node current node in recursion
 * @param box bounding box of current node
 * @param depth current depth
 * @return similar object pointer, NULL if not found
 */
Object* Octree::getSimilarObject(const Object *object, Node *node, Box box, unsigned int depth)
{
    Box childBox;
    Object *similar;

    for(unsigned char i = 0; i < Node::CHILDREN; i++)
    {
        Node *child = node->child(i);

        if(!child) continue;

        if(object->interfereWithBox(Node::getChildBox(i, childBox, box)))
        {
            if(depth < m_maxDepth)
            {
                similar = getSimilarObject(object, child, childBox, depth+1);
                if(similar) return similar;
            }
            else
            {
                for(std::list<Object*>::const_iterator j = node->objects().begin(); j != node->objects().end(); j++)
                {
                    if(*j != object && (*j)->isSimilar(object))
                    {
                        return *j;
                    }
                }

                for(unsigned int n = 0; n < Node::NEIGHBORS; n++)
                {
                    Node *neighbor = child->neighbor(n);

                    if(!neighbor) continue;

                    for(std::list<Object*>::const_iterator j = neighbor->objects().begin(); j != neighbor->objects().end(); j++)
                    {
                        if(*j != object && (*j)->isSimilar(object))
                        {
                            return *j;
                        }
                    }
                }
            }
        }
    }

    return NULL;
}

/**
 * Returns similar object to selected one.
 * @param object object to compare
 * @return similar object pointer, NULL if not found
 */
Object* Octree::getSimilarObject(const Object* object)
{
    return getSimilarObject(object, m_root, m_rootSize);
}

/**
 * Inserts object into octree.
 * @param object object to insert
 * @return inserted object id
 */
unsigned int Octree::insert(Object* object)
{
    if(!object->hasId())
    {
        m_objects[m_maxId] = object;
        object->setId(m_maxId++);
    }
    else
    {
        m_objects[object->id()] = object;
    }

    return insertOnInterfere(object, m_root, m_rootSize);
}

/**
 * Inserts or updates object into octree.
 * Update object if there exists a similar object. Inserts otherwise.
 * @param object object to insert
 * @return inserted object id
 */
unsigned int Octree::insertUpdate(Object* object)
{
    bool inserted = false;
    return insertUpdateOnInterfere(object, m_root, m_rootSize, inserted);
}

/**
 * Inserts or updates object into octree, second variant.
 * Update object if there exists a similar object. Inserts otherwise.
 * This variant first find a similar object instead of finding it in inserting loop.
 * @param object object to insert
 * @return inserted object id
 */
unsigned int Octree::insertUpdate2(Object* object)
{
    Object *similar = getSimilarObject(object, m_root, m_rootSize);

    if(similar)
    {
        m_objects[similar->id()] = object;
        object->setId(similar->id());

        delete similar;
    }
    else
    {
        m_objects[m_maxId] = object;
        object->setId(m_maxId++);
    }

    return insertOnInterfere(object, m_root, m_rootSize);
}

/**
 * Check if there is a free space in selected point.
 * @param x
 * @param y
 * @param z
 * @return true if there is a free space, false otherwise
 */
bool Octree::isPositionFree(float x, float y, float z)
{
    Node *node = m_root;

    Box box(m_rootSize);

    for(;;)
    {
        unsigned char id = 0;

        if(x >= box.x+box.w/2.0f) id += 1;
        if(y >= box.y+box.h/2.0f) id += 2;
        if(z >= box.z+box.d/2.0f) id += 4;

        node = node->child(id);

        if(node == NULL)
        {
            return true;
        }

        for(std::list<Object*>::const_iterator i = node->objects().begin(); i != node->objects().end(); i++)
        {
            if((*i)->isPointInside(x, y, z))
            {
                return false;
            }
        }

        Node::getChildBox(id, box, box);
    }
}

/**
 * Returns nodes and objects in area filtered by filter.
 * @param nodesList output list of nodes
 * @param objectList output set of objects
 * @param filter pointer to a filter class
 */
void Octree::nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter)
{
    nodes(nodesList, objectList, filter, m_rootSize, m_root);
}

/**
 * Returns nodes and objects in area filtered by filter.
 * @param nodesList output list of nodes
 * @param objectList output set of objects
 * @param filter pointer to a filter class
 * @param dim recursion node bounding box
 * @param node recursion node
 */
void Octree::nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter, Box dim, Node *node)
{
    if(!filter->filter(dim)) return;

    for(std::list<Object*>::const_iterator i = node->objects().begin(); i != node->objects().end(); i++)
    {
        objectList.insert(*i);
    }

    nodesList.push_back(dim);

    for(unsigned int i = 0; i < Node::CHILDREN; i++)
    {
        Node *child = node->child(i);

        if(child)
        {
            Box newDim;
            Node::getChildBox(i, newDim, dim);
            nodes(nodesList, objectList, filter, newDim, child);
        }
    }
}

/**
 * Returns objects in area filtered by filter.
 * @param nodesList output list of nodes
 * @param filter pointer to a filter class
 */
void Octree::objects(std::set<Object*> &objectList, const Filter *filter)
{
    std::list<Box> nodesList;
    nodes(nodesList, objectList, filter, m_rootSize, m_root);
}

const Object* Octree::object(unsigned int id) const
{
    std::map<unsigned int, Object*>::const_iterator i = m_objects.find(id);

    if(i != m_objects.end()) return i->second;
    else return NULL;
}

/**
 * Removes object with selected id from octree.
 * @param id object id
 * @return returns true if there was removed an object
 */
bool Octree::removeObject(unsigned int id)
{
    std::map<unsigned int, Object*>::iterator i = m_objects.find(id);

    if(i != m_objects.end())
    {
        delete i->second;
        m_objects.erase(i);

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * Returns root node.
 * @return root node
 */
Node* Octree::root() const
{
    return m_root;
}

/**
 * Returns maximum internal object id.
 * @return maximum internal object id
 */
unsigned int Octree::maxId() const
{
    return m_maxId;
}

/**
 * Returns number of objects in octree.
 * @return number of octrees
 */
unsigned int Octree::count() const
{
    return m_objects.size();
}

/**
 * Returns all objects in objtree.
 * @return objects map
 */
const std::map<unsigned int, Object*>& Octree::objectsAll() const
{
    return m_objects;
}

}
