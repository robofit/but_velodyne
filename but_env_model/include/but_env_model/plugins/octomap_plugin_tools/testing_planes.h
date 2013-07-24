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
#pragma once
#ifndef TestingPlanes_H_included
#define TestingPlanes_H_included

#include "testing_object_base.h"

#include <vector>
#include <Eigen/Dense>


namespace but_env_model
{

/**
 * Test point by plane. Return true if point is in front of this plane.
 */
class CTestingPlane : public CTestingObjectBase
{
public:
	/// Point type
	typedef Eigen::Vector3f tPoint;

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	//! Simple constructor
	CTestingPlane();

	//! Copy constructor
	CTestingPlane( const CTestingPlane &plane );

	//! Assignement operator
	CTestingPlane operator =( const CTestingPlane &plane );

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
	CTestingPlane( const tPoint &p1, const tPoint &p2, const tPoint &p3 );

	//! Initialize by point and normal
	CTestingPlane( const tPoint &point, const tPoint &normal );

	//! Initialize by three points
	void set( const tPoint &p1, const tPoint &p2, const tPoint &p3 );

	//! Initialize by point and normal
	void set( const tPoint &point, const tPoint &normal );

	//! Test point
	virtual bool isIn( double x, double y, double z );

protected:
	//! Plane parameters
	tPoint m_plane;

	//! Last plane parameter
	double m_d;

}; // class CTestingPlane

/**
 * Stack of planes
 */
class CTestingPlanes : public CTestingObjectBase
{
public:
	//! Stack of planes type
	typedef std::vector< CTestingPlane > tPlanesStack;

public:
	//! Simple constructor
	CTestingPlanes();

	//! Initialize as a stack of planes
	CTestingPlanes( const tPlanesStack & planes );

	//! Initialize by stack of planes
	void set( const tPlanesStack &planes );

	//! Add plane to the stack
	void addPlane( const CTestingPlane &plane ) { m_planes.push_back( plane ); }

	//! Clear all planes from stack
	void clearPlanes(){ m_planes.clear(); }

	//! Test point
	virtual bool isIn( double x, double y, double z );

protected:
	//! Planes stack
	tPlanesStack m_planes;

}; // class CTestingPlanes

}



 // namespace but_env_model

// TestingPlanes_H_included
#endif


