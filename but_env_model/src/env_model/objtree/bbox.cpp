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

#include <cmath>
#include <but_env_model/objtree/bbox.h>

namespace objtree
{

/**
 * A constructor.
 * @param box bounding box position and size
 */
BBox::BBox(const Box &box) :
    m_box(box)
{
    m_type = ALIGNED_BOUNDING_BOX;
}

/**
 * Returns true if BBox fits into a box.
 * @param box box for test
 * @return true if BBox fits into a box, false otherwise
 */
bool BBox::fitsIntoBox(const Box &box) const
{
    if(m_box.x >= box.x && m_box.y >= box.y && m_box.z >= box.z && m_box.x+m_box.w <= box.x+box.w && m_box.y+m_box.h <= box.y+box.h && m_box.z+m_box.d <= box.z+box.d)
    {
        return true;
    }

    return false;
}

/**
 * Returns true if BBox interferes with a box.
 * @param box box for test
 * @return true if BBox interferes with box, false otherwise
 */
bool BBox::interfereWithBox(const Box &box) const
{
    if(m_box.x > box.x+box.w || m_box.x+m_box.w < box.x)
    {
        return false;
    }

    if(m_box.y > box.y+box.h || m_box.y+m_box.h < box.y)
    {
        return false;
    }

    if(m_box.z > box.z+box.d || m_box.z+m_box.d < box.z)
    {
        return false;
    }

    return true;
}

/**
 * Compare other object for similarity.
 * @param object object to compare
 * @return true if object is similar, false otherwise
 */
bool BBox::isSimilar(const Object *object) const
{
    if(object->type() != ALIGNED_BOUNDING_BOX) return false;

    return isSimilarBBox((BBox*)object);
}

/**
 * Compare other bounding box for similarity.
 * @param box bounding box to compare
 * @return true if bounding box is similar, false otherwise
 */
bool BBox::isSimilarBBox(const BBox *box) const
{

    if(fabs(box->m_box.x-m_box.x) > 0.5f) return false;
    if(fabs(box->m_box.y-m_box.y) > 0.5f) return false;
    if(fabs(box->m_box.z-m_box.z) > 0.5f) return false;

    if(fabs(box->m_box.w-m_box.w) > 0.2f) return false;
    if(fabs(box->m_box.h-m_box.h) > 0.2f) return false;
    if(fabs(box->m_box.d-m_box.d) > 0.2f) return false;

    return true;
}

/**
 * Check if point is in the BBox.
 * @param x
 * @param y
 * @param z
 * @return true if point is inside the BBox, false otherwise
 */
bool BBox::isPointInside(float x, float y, float z) const
{
    return x >= m_box.x && x <= m_box.x+m_box.w
            && y >= m_box.y && y <= m_box.y+m_box.h
            && z >= m_box.z && z <= m_box.z+m_box.d;
}

/**
 * Returns position and size of the bounding box
 * @return position and size of the bounding box
 */
const Box& BBox::box() const
{
    return m_box;
}

}
