/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
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

#include <srs_env_model/but_server/plugins/octomap_plugin_tools/testing_planes.h>
#include <iostream>

/**
 * Simple constructor
 */
srs_env_model::CTestingPlane::CTestingPlane()
: m_plane()
, m_d( 0.0 )
{

}

/**
 * Copy constructor
 */
srs_env_model::CTestingPlane::CTestingPlane(const CTestingPlane & plane)
: m_plane( plane.m_plane )
, m_d( plane.m_d )
{

}


/**
 * Assignement operator
 */
srs_env_model::CTestingPlane srs_env_model::CTestingPlane::operator =(const CTestingPlane & plane)
{
	m_plane = plane.m_plane;
	m_d = plane.m_d;

	return plane;
}


/**
 * Initialize by three points
 */
srs_env_model::CTestingPlane::CTestingPlane(const tPoint & p1, const tPoint & p2, const tPoint & p3)
{
	// Just call method
	set( p1, p2, p3 );
}


/**
 * Initialize by point and normal
 */
srs_env_model::CTestingPlane::CTestingPlane(const tPoint & point, const tPoint & normal)
{
	// Just call method
	set( point, normal );
}


/**
 * Initialize by three points
 *
 * This points are used:
 *
 *	  normal
 * 		|    p3
 *      |   /
 * 		|  /
 * 		| /
 * 		p1---------------p2
 *
 * 		d1 = p2 - p1
 * 		d2 = p3 - p1
 * 		n = d1 x d2
 */
void srs_env_model::CTestingPlane::set(const tPoint & p1, const tPoint & p2, const tPoint & p3)
{
	// Compute directional vectors
	tPoint dir1( p2 - p1 ), dir2( p3 - p1 );

	// Get normal vector from cross-product
	m_plane = dir1;
	m_plane = m_plane.cross(dir2);
	m_plane.normalize();

//	std::cerr << "Point: " << p1 << std::endl << "Normal: " << m_plane << std::endl;

	// Compute last parameter
	m_d = - p1.dot( m_plane );
}


/**
 * Initialize by point and normal
 */
void srs_env_model::CTestingPlane::set(const tPoint & point, const tPoint & normal)
{
	m_plane = normal;
	m_plane.normalize();
	m_d = - point.dot( m_plane );
}


/**
 * Test point
 */
bool srs_env_model::CTestingPlane::isIn(double x, double y, double z)
{
	tPoint p( x, y, z );

	return p.dot( m_plane ) + m_d >= 0;
}


///////////////////////////////////////////////////////////////////////////////

/*
 * Simple constructor
 */
srs_env_model::CTestingPlanes::CTestingPlanes()
{
}


/**
 * Initialize as a stack of planes
 */
srs_env_model::CTestingPlanes::CTestingPlanes(const tPlanesStack & planes)
{
	// Just call method
	set( planes );
}


/**
 * Initialize by stack of planes
 */
void srs_env_model::CTestingPlanes::set(const tPlanesStack & planes)
{
	if( planes.size() == 0 )
		return;

	m_planes.clear();
	m_planes.reserve( planes.size() );
	std::copy( planes.begin(), planes.end(), m_planes.begin() );
}

/*
 * Test point
 */
bool srs_env_model::CTestingPlanes::isIn( double x, double y, double z )
{
	tPlanesStack::iterator i, end;
	for( i = m_planes.begin(), end = m_planes.end(); i != end; ++i)
	{
		// One wrong is enough
		if( ! i->isIn( x, y, z ) )
			return false;
	}

	return true;
}
