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
 * Date: 23/05/2012
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

#include <but_env_model/objtree/history.h>
#include <cstdio>

namespace objtree
{

/**
 * Constructor. Creates empty history.
 */
History::History()
{
    for(unsigned int i = 0; i < length; i++)
    {
        m_history[i].x = m_history[i].y = m_history[i].z = 0.0f;
    }
}

/**
 * Updates object history.
 * @param point point to add
 */
void History::update(Point point)
{
    for(unsigned int i = length-1; i > 0; i--)
    {
        m_history[i] = m_history[i-1];
    }

    m_history[0] = point;
}

}
