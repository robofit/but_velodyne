/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
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
#ifndef OBJTREE_OCTREE_H
#define OBJTREE_OCTREE_H

#include <map>
#include <list>
#include <set>
#include <srs_env_model/but_server/objtree/box.h>

namespace objtree
{

class Node;
class Object;
class Filter;

/**
 * Main objtree class. Provides interface for working with octree.
 */

class Octree
{
public:
    static const unsigned int DEFAULT_MAX_DEPTH = 4;

private:
    Node *m_root;
    Box m_rootSize;
    unsigned int m_maxId;
    unsigned int m_maxDepth;    
    std::map<unsigned int, Object*> m_objects;

public:
    Octree(unsigned int maxDepth = DEFAULT_MAX_DEPTH);
    Octree(const Box &rootSize, unsigned int maxDepth = DEFAULT_MAX_DEPTH);
    ~Octree();

    void clear();

    unsigned int insert(Object* object);
    unsigned int insertOnFit(Object* object);
    unsigned int insertOnInterfere(Object* object, Node *node, Box box, unsigned int depth = 0);

    unsigned int insertUpdate(Object* object);
    unsigned int insertUpdate2(Object* object);
    unsigned int insertUpdateOnInterfere(Object* object, Node *node, Box box, bool &inserted, unsigned int depth = 0);

    Object* getSimilarObject(const Object *object);
    Object* getSimilarObject(const Object *object, Node *node, Box box, unsigned int depth = 0);
    bool isPositionFree(float x, float y, float z);

    Node* root() const;
    unsigned int maxId() const;
    unsigned int count() const;

    const Object* object(unsigned int id) const;
    bool removeObject(unsigned int id);

    void nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter);
    void nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter, Box dim, Node *node);
    void objects(std::set<Object*> &objectList, const Filter *filter);

    const std::map<unsigned int, Object*>& objectsAll() const;
};

}

#endif // OBJTREE_OCTREE_H
