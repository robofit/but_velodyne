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

#pragma once
#ifndef OBJTREE_OBJECT_H
#define OBJTREE_OBJECT_H

#include <list>

#include <but_env_model/objtree/box.h>
#include <but_env_model/objtree/history.h>

namespace objtree
{

class Node;

/**
 * Abstract object class. Base class for all objects saved in octree.
 */

class Object
{
public:
    enum Type
    {
        PLANE = 1,
        ALIGNED_BOUNDING_BOX = 2,
        GENERAL_BOUNDING_BOX = 3
    };

private:
    std::list<Node*> m_inNodes;
    unsigned int m_id;

protected:
    Type m_type;
#if HISTORY_ENABLED
    History *m_history;
#endif

public:
    Object();
    ~Object();

    virtual bool fitsIntoBox(const Box &box) const = 0;
    virtual bool interfereWithBox(const Box &box) const = 0;
    virtual bool isSimilar(const Object *object) const = 0;
    virtual bool isPointInside(float x, float y, float z) const = 0;

    void setId(unsigned int id);
    unsigned int id() const;
    bool hasId() const;
    Type type() const;

    void newNode(Node *node);
    void removeNode(Node *node);

    unsigned int inNodesCount() const;

#if HISTORY_ENABLED
    /// Gets history.
    History *history() { return m_history; }
    /// Move history from other object.
    void takeHistory(Object *object)
    {
        if(!object->m_history)
        {
            object->m_history = new History;
        }

        object->updateHistory();
        m_history = object->m_history;
        object->m_history = 0;
    }
    /// Updates history
    virtual void updateHistory() = 0;
#endif
};

}

#endif // OBJTREE_OBJECT_H
