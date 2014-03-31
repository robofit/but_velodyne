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
#ifndef OBJTREE_NODE_H
#define OBJTREE_NODE_H

#include <cstdio>
#include <list>

#include <but_env_model/objtree/object.h>

namespace objtree
{

/**
 * Octree node class.
 */

class Node
{
public:
  static const unsigned int CHILDREN = 8;
  static const unsigned int NEIGHBORS = 26;

private:
  unsigned char m_place;
  Node* m_parent;

  /*
   * Neighbors ids (from top view)
   * Top part:  Middle part:  Bottom part:
   *  6  7  8     14 15 16      23 24 25
   *  3  4  5     12    13      20 21 22
   *  0  1  2      9 10 11      17 18 19
  */
  Node* m_neighbors[NEIGHBORS];

  /*
   * Node children ids (from top view)
   * Top part:  Bottom part:
   *   2  3         6  7
   *   0  1         4  5
   *
  */
  Node* m_children[CHILDREN];
  std::list<Object*> m_objects;

  static unsigned char reverseNeighborId(unsigned char dir);
  Node* parentNeighborChild(unsigned char parentNeighbor, unsigned char child);
  Node* computeNeighbor(unsigned char dir);

public:
  Node(unsigned char place = 0, Node *parent = NULL);
  ~Node();
  Node* parent();
  Node* child(unsigned char place, bool createNew = false);
  Node* neighbor(unsigned char dir);
  const std::list<Object*>& objects() const;
  void add(Object* object);
  void removeObject(Object *object);
  void deleteIfEmpty();

  static Box& getChildBox(unsigned char place, Box &childBox, const Box &parentBox);
};

}

#endif // OBJTREE_NODE_H
