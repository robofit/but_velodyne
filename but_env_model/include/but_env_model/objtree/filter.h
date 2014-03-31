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
#ifndef OBJTREE_FILTER_H
#define OBJTREE_FILTER_H

#include <but_env_model/objtree/box.h>

namespace objtree
{

/**
 * Abstract filter class.
 */
class Filter
{
public:
  enum Type
  {
    ZERO = 0,
    BOX = 1,
    PLANE = 2,
    SPHERE = 3
  };

  virtual bool filter(const Box &dim) const = 0;
  virtual Type type() const = 0;
};

/**
 * Box filter class.
 */
class FilterBox : public Filter
{
private:
  Box m_box;

public:
  FilterBox(const Box &box);
  virtual bool filter(const Box &dim) const;

  virtual Type type() const
  {
    return BOX;
  }
};

/**
 * Plane filter class.
 */
class FilterPlane : public Filter
{
private:
  float m_posX, m_posY, m_posZ;
  float m_vecX, m_vecY, m_vecZ;

public:
  FilterPlane(float posX, float posY, float posZ, float vecX, float vecY, float vecZ);
  virtual bool filter(const Box &dim) const;

  virtual Type type() const
  {
    return PLANE;
  }
};

/**
 * No filter class.
 */
class FilterZero : public Filter
{
public:
  virtual bool filter(const Box &dim) const;

  virtual Type type() const
  {
    return ZERO;
  }
};

/**
 * Sphere filter class.
 */
class FilterSphere : public Filter
{
private:
  float m_x, m_y, m_z;
  float m_radiusSquare;

public:
  FilterSphere(float x, float y, float z, float radius);
  virtual bool filter(const Box &dim) const;

  virtual Type type() const
  {
    return SPHERE;
  }
};

}

#endif // OBJTREE_FILTER_H
