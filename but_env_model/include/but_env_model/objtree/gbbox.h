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
 * Date: 21/06/2012
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
#ifndef OBJTREE_GBBOX_H
#define OBJTREE_GBBOX_H

#include <but_env_model/objtree/bbox.h>

namespace objtree
{

/**
 * General bounding box class. Represent not axis aligned bounding box in octree.
 */

class GBBox : public BBox
{
private:
  const Point m_position;
  const Vector4f m_orientation;
  const Point m_scale;

public:
  GBBox(const Point &position, const Vector4f &orientation, const Point &scale, const Box &box);

  virtual bool isSimilar(const Object *object) const;

  const Point& position() const
  {
    return m_position;
  }
  const Vector4f& orientation() const
  {
    return m_orientation;
  }
  const Point& scale() const
  {
    return m_scale;
  }
};

}

#endif // OBJTREE_BBOX_H
