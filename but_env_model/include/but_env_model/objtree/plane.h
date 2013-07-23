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
#ifndef OBJTREE_PLANE_H
#define OBJTREE_PLANE_H

#include <but_env_model/objtree/object.h>
#include <but_env_model/objtree/box.h>
#include <but_env_model/objtree/types.h>

namespace objtree
{

/**
 * Plane class. Represent plane in octree.
 */

class Plane : public Object
{
private:
    Point m_pos;
    Vector m_normal;
    Polygon m_points;
    float m_d;
    Point m_boundingMin, m_boundingMax;

public:
    Plane(const Point &center, const Vector &normal, const Point &scale);
    Plane(const Polygon &points);

    virtual bool fitsIntoBox(const Box &box) const;
    virtual bool interfereWithBox(const Box &box) const;
    virtual bool isSimilar(const Object *object) const;
    virtual bool isPointInside(float x, float y, float z) const;

    const Point& pos() const { return m_pos; }
    const Vector& normal() const { return m_normal; }
    const Polygon& points() const { return m_points; }
    const Point& boundingMin() const { return m_boundingMin;  }
    const Point& boundingMax() const { return m_boundingMax;  }

#if HISTORY_ENABLED
    /// Updates history
    virtual void updateHistory() { m_history->update(m_pos); }
#endif
};

}

#endif // OBJTREE_PLANE_H
