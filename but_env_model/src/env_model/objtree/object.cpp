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

#include <but_env_model/objtree/node.h>
#include <but_env_model/objtree/object.h>

namespace objtree
{

/**
 * A constructor.
 */
Object::Object()
{
    m_id = -1;
#if HISTORY_ENABLED
    m_history = NULL;
#endif
}

/**
 * A destructor.
 * Removes object from octree.
 */
Object::~Object()
{
    for(std::list<Node*>::iterator i = m_inNodes.begin(); i != m_inNodes.end(); i++)
    {
        (*i)->removeObject(this);
    }

#if HISTORY_ENABLED
    if(m_history)
    {
        delete m_history;
    }
#endif
}

/**
 * Returns object type.
 * @return object type
 */
Object::Type Object::type() const
{
    return m_type;
}

/**
 * Sets a new object id.
 * @param id new object id
 */
void Object::setId(unsigned int id)
{
    m_id = id;
}

/**
 * Returns true if object has an id.
 * @return true if object has an id, false otherwise
 */
bool Object::hasId() const
{
    return m_id != (unsigned int)-1;
}

/**
 * Returns object id.
 * @return object id
 */
unsigned int Object::id() const
{
    return m_id;
}

/**
 * Informs object it is include in a new node.
 * @param pointer to node
 */
void Object::newNode(Node *node)
{
    m_inNodes.push_back(node);
}

/**
 * Informs object it is removed from node.
 * @param pointer to node
 */
void Object::removeNode(Node *node)
{
    m_inNodes.remove(node);
}

/**
 * Returns number of occupied nodes.
 * @return number of occupied nodes
 */
unsigned int Object::inNodesCount() const
{
    return m_inNodes.size();
}

}
