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
#ifndef OBJTREE_BBOX_H
#define OBJTREE_BBOX_H

#include <but_env_model/objtree/object.h>
#include <but_env_model/objtree/box.h>

namespace objtree
{

/**
 * Bounding box class. Represent aligned bounding box in octree.
 */

class BBox : public Object
{
private:
    Box m_box;

public:
    BBox(const Box &box);

    virtual bool fitsIntoBox(const Box &box) const;
    virtual bool interfereWithBox(const Box &box) const;
    virtual bool isSimilar(const Object *object) const;
    virtual bool isSimilarBBox(const BBox *object) const;
    virtual bool isPointInside(float x, float y, float z) const;

    const Box& box() const;

#if HISTORY_ENABLED
    /// Updates history
    virtual void updateHistory() { m_history->update(Point(m_box.x, m_box.y, m_box.z)); }
#endif
};

}

#endif // OBJTREE_BBOX_H
