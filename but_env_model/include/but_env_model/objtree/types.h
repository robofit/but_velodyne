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
#ifndef OBJTREE_TYPES_H
#define OBJTREE_TYPES_H

#include <vector>

namespace objtree
{

/**
 * Simple 3D point structure.
 */

struct Point
{
    float x, y, z;

    Point(float x, float y, float z) : x(x), y(y), z(z) {}
    Point(const Point &p) : x(p.x), y(p.y), z(p.z) {}
    Point() {}

    Point operator-(const Point &p) const { return Point(x-p.x, y-p.y, z-p.z); }
    Point operator+(const Point &p) const { return Point(x+p.x, y+p.y, z+p.z); }
    void operator/=(float num) { x/=num; y/=num; z/=num; }
    Point operator/(float num) const { return Point(x/num, y/num, z/num); }
};

typedef Point Vector;
typedef std::vector<Point> Polygon;

struct Vector4f
{
    float x, y, z, w;

    Vector4f(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vector4f(const Vector4f &v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
    Vector4f() {}
};

}

#endif // OBJTREE_TYPES_H
