/******************************************************************************
 * \file
 *
 * $Id:$
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

#include <but_env_model/plugins/octomap_plugin_tools/testing_oriented_box.h>

/**
 * Constructor
 */
but_env_model::CTestingOrientedBox::CTestingOrientedBox()
: m_minX( 0.0 )
, m_minY( 0.0 )
, m_minZ( 0.0 )
, m_maxX( 0.0 )
, m_maxY( 0.0 )
, m_maxZ( 0.0 )
{

}

/**
 * Constructor - initializing
 */
but_env_model::CTestingOrientedBox::CTestingOrientedBox( double minx, double miny, double minz, double maxx, double maxy, double maxz )
: m_minX( minx )
, m_minY( miny )
, m_minZ( minz )
, m_maxX( maxx )
, m_maxY( maxy )
, m_maxZ( maxz )
{

}

/**
 * Set limits
 */
void but_env_model::CTestingOrientedBox::set( double minx, double miny, double minz, double maxx, double maxy, double maxz )
{
	m_minX = minx; m_minY = miny; m_minZ = minz;
	m_maxX = maxx; m_maxY = maxy; m_maxZ = maxz;
}

/**
 * Set by center and sizes
 */
void but_env_model::CTestingOrientedBox::setCenterSize( double x, double y, double z, double sx, double sy, double sz )
{
	// Compute half size from sizes
	double shx( sx / 2 ), shy( sy / 2), shz( sz / 2 );

	m_minX = x - shx; m_maxX = x + shx;
	m_minY = y - shy; m_maxY = y + shy;
	m_minZ = z - shz; m_maxZ = z + shz;
}
/**
 * Testing function
 */
bool but_env_model::CTestingOrientedBox::isIn( double x, double y, double z )
{
	return ( x >= m_minX ) && ( x < m_maxX ) && ( y >= m_minY ) && ( y < m_maxY ) && ( z >= m_minZ ) && ( z < m_maxZ );
}
