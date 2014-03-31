/******************************************************************************
 * \file
 *
 * $Id: octonode.h 2155 2012-12-27 17:45:59Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
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
#ifndef OCTONODE_H
#define OCTONODE_H

#include "but_octomap_ros.h"

#include <octomap/OcTreeStamped.h>

namespace but_env_model
{

/**
 * Nodes to be used in a environment model server.
 *
 */
class EModelTreeNode: public octomap::OcTreeNodeStamped
{

public:

  /**
   * Constructor
   */
  EModelTreeNode();

  /**
   * Destructor
   */
  ~EModelTreeNode();

  bool createChild(unsigned int i);

  // overloaded, so that the return type is correct:
  inline EModelTreeNode* getChild(unsigned int i)
  {
    return static_cast<EModelTreeNode*>(octomap::OcTreeDataNode<float>::getChild(i));
  }
  inline const EModelTreeNode* getChild(unsigned int i) const
  {
    return static_cast<const EModelTreeNode*>(octomap::OcTreeDataNode<float>::getChild(
             i));
  }

  //! Get color components
  unsigned char r() const
  {
    return m_r;
  }
  unsigned char g() const
  {
    return m_g;
  }
  unsigned char b() const
  {
    return m_b;
  }
  unsigned char a() const
  {
    return m_a;
  }

  //! Get color components - reference version
  unsigned char & r()
  {
    return m_r;
  }
  unsigned char & g()
  {
    return m_g;
  }
  unsigned char & b()
  {
    return m_b;
  }
  unsigned char & a()
  {
    return m_a;
  }

  //! Set color components
  void setColor(unsigned char r, unsigned char g, unsigned char b,
                unsigned char a = 255)
  {
    m_r = r;
    m_g = g;
    m_b = b;
    m_a = a;
  }

  // has any color been integrated? (pure white is very unlikely...)
  inline bool isColorSet() const
  {
    return ((m_r != 255) || (m_g != 255) || (m_b != 255));
  }

  // overloaded tree pruning taking care of node colors
  bool pruneNode();

  // overloaded tree expanding taking care of node colors
  void expandNode();

  // update node color
  void updateColorChildren();

  // set color to average child color
  void setAverageChildColor();

  /**
   * Read node from binary stream (incl. float value),
   * recursively continue with all children.
   *
   * @param s
   * @return
   */
  std::istream& readValue(std::istream &s);

  /**
   * Write node to binary stream (incl float value),
   * recursively continue with all children.
   * This preserves the complete state of the node.
   *
   * @param s
   * @return
   */
  std::ostream& writeValue(std::ostream &s) const;

protected:
  //! Color data
  unsigned char m_r, m_g, m_b, m_a;

}; // class EModelTreeNode

/**
 * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
 * Basic functionality is implemented in OcTreeBase.
 *
 */
class EMOcTree: public octomap::OccupancyOcTreeBase<EModelTreeNode>
{

public:
  //! Used point type
  typedef octomap::point3d tPoint;

  //! Used node type
  typedef EModelTreeNode tNode;

  typedef pcl::PointCloud<pcl::PointXYZRGB> typePointCloud;

public:

  /**
   * Creates a new (empty) OcTree of a given resolution
   * @param _resolution
   */
  EMOcTree(double _resolution);

  /**
   * Reads an OcTree from a binary file
   * @param _filename
   *
   */
  EMOcTree(std::string _filename);

  /**
   * Destructor
   */
  virtual ~EMOcTree()
  {
  }
  ;

  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  EMOcTree* create() const
  {
    return new EMOcTree(resolution);
  }

  /**
   *  Get tree type as a string.
   */
  std::string getTreeType() const
  {
    return "EMOcTree";
  }

  // set node color at given key or coordinate. Replaces previous color.
  EModelTreeNode* setNodeColor(const octomap::OcTreeKey& key, const unsigned char& r,
                               const unsigned char& g, const unsigned char& b,
                               const unsigned char& a);

  EModelTreeNode* setNodeColor(const float& x, const float& y,
                               const float& z, const unsigned char& r, const unsigned char& g,
                               const unsigned char& b, const unsigned char& a)
  {
    octomap::OcTreeKey key;
//        if (!this->genKey(octomap::point3d(x, y, z), key))
    if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key))
      return NULL;
    return setNodeColor(key, r, g, b, a);
  }

  // integrate color measurement at given key or coordinate. Average with previous color
  EModelTreeNode* averageNodeColor(const octomap::OcTreeKey& key,
                                   const unsigned char& r, const unsigned char& g,
                                   const unsigned char& b, const unsigned char& a);

  EModelTreeNode* averageNodeColor(const float& x, const float& y,
                                   const float& z, const unsigned char& r, const unsigned char& g,
                                   const unsigned char& b, const unsigned char& a)
  {
    octomap::OcTreeKey key;
//      if (!this->genKey(octomap::point3d(x, y, z), key))
    if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key))
      return NULL;
    return averageNodeColor(key, r, g, b, a);
  }

  // integrate color measurement at given key or coordinate.
  // Average with previous color taking account of node probabilities
  EModelTreeNode* integrateNodeColor(const octomap::OcTreeKey& key,
                                     const unsigned char& r, const unsigned char& g,
                                     const unsigned char& b, const unsigned char& a);

  EModelTreeNode* integrateNodeColor(const float& x, const float& y,
                                     const float& z, const unsigned char& r, const unsigned char& g,
                                     const unsigned char& b, const unsigned char& a)
  {
    octomap::OcTreeKey key;
//    if (!this->genKey(octomap::point3d(x, y, z), key))
    if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key))
      return NULL;
    return integrateNodeColor(key, r, g, b, a);
  }

  // update inner nodes, sets color to average child color
  void updateInnerOccupancy();

  //! \return timestamp of last update
  unsigned int getLastUpdateTime();

  void degradeOutdatedNodes(unsigned int time_thres);

  virtual void
  updateNodeLogOdds(EModelTreeNode* node, const float& update) const;
  void integrateMissNoTime(EModelTreeNode* node) const;


  // Overloaded. Inserts colored scan
  void insertColoredScan(const typePointCloud& coloredScan,
                         const octomap::point3d& sensor_origin, double maxrange = -1.,
                         bool pruning = true, bool lazy_eval = false);

protected:
  void updateInnerOccupancyRecurs(EModelTreeNode* node, unsigned int depth);

  /**
   * Static member object which ensures that this EMOcTree prototype
   * ends up in the classIDMapping only once
   */
  class StaticMemberInitializer
  {
  public:
    StaticMemberInitializer()
    {
      EMOcTree* tree = new EMOcTree(0.1);
      AbstractOcTree::registerTreeType(tree);
      std::cerr << "Registered tree type: " << tree->getTreeType() << std::endl;
    }
  };

  /// to ensure static initialization (only once)
  static StaticMemberInitializer ocEMOcTreeMemberInit;
}; // class EMOcTree

}
; // namespace but_env_model

#endif // OCTONODE_H
