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
#ifndef OBJTREE_BOX_H
#define OBJTREE_BOX_H

namespace objtree
{

/**
 * Simple box structure.
 */

struct Box
{
    float x, y, z;
    float w, h, d;

    inline Box()
    {
    }

    inline Box(float x, float y, float z, float w, float h, float d)
    {
        init(x, y, z, w, h, d);
    }

    inline void init(float x, float y, float z, float w, float h, float d)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
        this->h = h;
        this->d = d;
    }

    bool operator==(const Box &box)
    {
        return box.x == x && box.y == y && box.z == z && box.w == w && box.h == h && box.d == d;
    }
};

}

#endif // OBJTREE_BOX_H
