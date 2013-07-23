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
#pragma once
#ifndef TestingSphere_H_included
#define TestingSphere_H_included

#include "testing_object_base.h"

namespace srs_env_model
{
/**
 * Test if point is in the sphere
 */
class CTestingSphere : public CTestingObjectBase
{
public:
	//! Simple constructor
	CTestingSphere() : m_radiussq( 0.0 ), m_x( 0.0 ), m_y(0.0), m_z(0.0) {}

	//! Initializing constructor
	CTestingSphere( double x, double y, double z, double radius )
	: m_radiussq( radius * radius ), m_x( x ), m_y( y ), m_z( z )
	{}

	//! Set sphere parameters
	void set( double x, double y, double z, double radius )
	{ m_radiussq = radius*radius; m_x = x; m_y = y; m_z = z; }

	//! Test point
	virtual bool isIn( double x, double y, double z )
	{
		double sx( x - m_x ), sy( y - m_y ), sz( z - m_z );
		return sx*sx + sy*sy + sz*sz < m_radiussq;
	}

protected:
	//! Sphere radius squared
	double m_radiussq;

	//! Sphere position
	double m_x, m_y, m_z;

}; // class CTestingSphere

} // namespace srs_env_model


// TestingSphere_H_included
#endif
