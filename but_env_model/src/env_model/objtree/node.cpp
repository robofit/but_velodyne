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

#include <cstdio>
#include <srs_env_model/but_server/objtree/node.h>

namespace objtree
{

/**
 * A constructor.
 * Creates a new node.
 * @param place position inside the parent
 * @param parent pointer to the parent
 */
Node::Node(unsigned char place, Node *parent)
{
    m_place = place;
    m_parent = parent;

    if(m_parent != NULL)
    {
        //Update neighbor pointers in neighbors
        for(unsigned char neighbor = 0; neighbor < NEIGHBORS; neighbor++)
        {
            m_neighbors[neighbor] = computeNeighbor(neighbor);

            if(m_neighbors[neighbor] != NULL)
            {
                m_neighbors[neighbor]->m_neighbors[reverseNeighborId(neighbor)] = this;
            }
        }
    }
    else
    {
        //Root node hasn't got any neighbors
        for(unsigned char neighbor = 0; neighbor < NEIGHBORS; neighbor++)
        {
            m_neighbors[neighbor] = NULL;
        }
    }

    for(unsigned char i = 0; i < CHILDREN; i++)
    {
        m_children[i] = NULL;
    }
}

/**
 * A destructor.
 */
Node::~Node()
{
    //Nullify all neighbor pointers to this node
    for(unsigned char neighbor = 0; neighbor < NEIGHBORS; neighbor++)
    {
        if(m_neighbors[neighbor] != NULL)
        {
            m_neighbors[neighbor]->m_neighbors[reverseNeighborId(neighbor)] = NULL;
        }
    }

    //Delete all children
    for(unsigned char place = 0; place < CHILDREN; place++)
    {
        if(m_children[place]) delete m_children[place];
    }

    //Delete all objects
    for(std::list<Object*>::iterator i = m_objects.begin(); i != m_objects.end(); i++)
    {
        (*i)->removeNode(this);
        delete *i;
    }
}

/**
 * Returns node parent.
 * @return node parent
 */
Node* Node::parent()
{
    return m_parent;
}

/**
 * Returns pointer to the child.
 * @param place child position
 * @param createNew if true non-existing childs are created
 * @return pointer to child
 */
Node* Node::child(unsigned char place, bool createNew)
{
    if(!m_children[place] && createNew)
    {
        m_children[place] = new Node(place, this);
    }

    return m_children[place];
}

/**
 * Returns pointer to the neighbor.
 * @param dir neighbor direction (0-25)
 * @return pointer to the neighbor
 */
Node* Node::neighbor(unsigned char dir)
{
    return m_neighbors[dir];
}

/**
 * Returns objects in node.
 * @return std::list of objects
 */
const std::list<Object*>& Node::objects() const
{
    return m_objects;
}

/**
 * Adds object to the node.
 * @param object object to add
 */
void Node::add(Object* object)
{
    m_objects.push_back(object);
    object->newNode(this);
}

/**
 * Computes position and size of selected child node.
 * @param place child position
 * @param childBox output computed position and size
 * @param parentBox parent bounding box
 * @return computed position and size
 */
Box& Node::getChildBox(unsigned char place, Box &childBox, const Box &parentBox)
{
    childBox = parentBox;

    childBox.w/=2.0f;
    childBox.h/=2.0f;
    childBox.d/=2.0f;

    switch(place)
    {
        case 0:
            break;
        case 1:
            childBox.x+=childBox.w;
            break;
        case 2:
            childBox.y+=childBox.h;
            break;
        case 3:
            childBox.x+=childBox.w;
            childBox.y+=childBox.h;
            break;
        case 4:
            childBox.z+=childBox.d;
            break;
        case 5:
            childBox.x+=childBox.w;
            childBox.z+=childBox.d;
            break;
        case 6:
            childBox.y+=childBox.h;
            childBox.z+=childBox.d;
            break;
        case 7:
            childBox.x+=childBox.w;
            childBox.y+=childBox.h;
            childBox.z+=childBox.d;
            break;
    }

    return childBox;
}

/**
 * Returns pointer to the child from parent neighbor.
 * @param parentNeighbor parent neighbor direction
 * @param child child position
 * @return pointer to the selected node
 */
Node* Node::parentNeighborChild(unsigned char parentNeighbor, unsigned char child)
{
    if(m_parent->m_neighbors[parentNeighbor] != NULL)
    {
        return m_parent->m_neighbors[parentNeighbor]->m_children[child];
    }

    return NULL;
}

/*
 * Neighbors ids (from top view)
 * Top part:  Middle part:  Bottom part:
 *  6  7  8     14 15 16      23 24 25
 *  3  4  5     12    13      20 21 22
 *  0  1  2      9 10 11      17 18 19
 *
 * Node children ids (from top view)
 * Top part:  Bottom part:
 *   2  3         6  7
 *   0  1         4  5
 *
 * Temporary ids (from top view)
 * 12 13 14 15  28 29 30 31  44 45 46 47  60 61 62 63
 *  8  9 10 11  24 25 26 27  40 41 42 43  56 57 58 59
 *  4  5  6  7  20 21 22 23  36 37 38 39  52 53 54 55
 *  0  1  2  3  16 17 18 19  32 33 34 35  48 49 50 51
 */

/**
 * Computes neighbor from node position and neighbor direction.
 * Returns pointer to the computed node.
 * @param dir neighbor direction
 * @return pointer to the neighbor or NULL if neighbor doesn't exists
 */
Node* Node::computeNeighbor(unsigned char dir)
{
   if(dir >= 13) dir++;

   unsigned char x = m_place%2+1;
   unsigned char y = (m_place%4)/2+1;
   unsigned char z = m_place/4+1;

   x -= dir%3 == 0;
   x += (dir+1)%3 == 0;

   y -= dir%9 <= 2;
   y += dir%9 >= 6;

   z -= dir <= 8;
   z += dir >= 18;

   unsigned char child = 1-x%2;
   if(y%2 == 0) child += 2;
   if(z%2 == 0) child += 4;

   x -= x >= 2;
   y -= y >= 2;
   z -= z >= 2;

   unsigned char parent = z*9+y*3+x;

   if(parent == 13) return m_parent->m_children[child];
   if(parent > 13) parent--;

   return parentNeighborChild(parent, child);
}

/**
 * Returns opposite neighbor direction.
 * @param dir neighbor direction
 * @return opposite neighbor direction
 */
unsigned char Node::reverseNeighborId(unsigned char dir)
{
    return 25-dir;
}

/**
 * Removes object from node.
 * Also removes node if there is no one remaining.
 * @param object pointer to the object
 */
void Node::removeObject(Object *object)
{
    m_objects.remove(object);

    if(m_objects.size() == 0)
    {
        m_parent->m_children[m_place] = NULL;
        m_parent->deleteIfEmpty();
        delete this;
    }
}

/**
 * Removes node if it doesn't contain any children and objects.
 */
void Node::deleteIfEmpty()
{
    //We don't want to delete root node
    if(m_parent == NULL)
    {
        return;
    }

    for(unsigned char child = 0; child < CHILDREN; child++)
    {
        if(m_children[child] != NULL)
        {
            return;
        }
    }

    if(m_objects.size() != 0)
    {
        return;
    }

    m_parent->m_children[m_place] = NULL;
    m_parent->deleteIfEmpty();
    delete this;
}

}
