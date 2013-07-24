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

#include <but_env_model/objtree/gbbox.h>
#include <but_env_model/objtree/types.h>

namespace objtree
{

/**
 * A constructor.
 * @param box bounding box position and size
 */
GBBox::GBBox(const Point &position, const Vector4f &orientation, const Point &scale, const Box &box) :
    BBox(box),
    m_position(position),
    m_orientation(orientation),
    m_scale(scale)
{
    m_type = GENERAL_BOUNDING_BOX;
}

/**
 * Compare other object for similarity.
 * @param object object to compare
 * @return true if object is similar, false otherwise
 */
bool GBBox::isSimilar(const Object *object) const
{
    if(object->type() != GENERAL_BOUNDING_BOX) return false;

    return isSimilarBBox((BBox*)object);
}

}
