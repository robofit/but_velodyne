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

#include <but_env_model/plugins/octomap_plugin_tools/testing_polymesh.h>
#include <Eigen/Geometry>
#include <iostream>

/**
 * Simple constructor
 */
but_env_model::CTestingPolymesh::CTestingPolymesh()
{

}

/**
 * Initialize as oriented box - constructor
 */
but_env_model::CTestingPolymesh::CTestingPolymesh(const tPoint &center, const tQuaternion &orientation, const tPoint & size)
{
	setAsBox( center, orientation, size );
}

/**
 * Initialize tester as a oriented box
 */
void but_env_model::CTestingPolymesh::setAsBox(const tPoint &center, const tQuaternion &orientation, const tPoint & size)
{
	// Corners
	tPoint points[8];

	// Create simple oriented box
	// Cube positions
	//
	//      7-----------6
	//     /|          /|
	//    /           / |
	//   /  |        /  |
	//  4-----------5   |
	//  |   |       |   |
	//  |   3 - - - + - 2
	//  |  /        |  /
	//  |           | /
	//  |/          |/
	//  0-----------1
	//
	//
	//
	//
/*
	points[0][0] = -size[0]; points[0][1] = -size[1]; points[0][2] = -size[2];
	points[1][0] =  size[0]; points[1][1] = -size[1]; points[1][2] = -size[2];
	points[2][0] =  size[0]; points[2][1] =  size[1]; points[2][2] = -size[2];
	points[3][0] = -size[0]; points[3][1] =  size[1]; points[3][2] = -size[2];
	points[4][0] = -size[0]; points[4][1] = -size[1]; points[4][2] =  size[2];
	points[5][0] =  size[0]; points[5][1] = -size[1]; points[5][2] =  size[2];
	points[6][0] =  size[0]; points[6][1] =  size[1]; points[6][2] =  size[2];
	points[7][0] = -size[0]; points[7][1] =  size[1]; points[7][2] =  size[2];
*/
	points[0][0] = -size[0]/2.0; points[0][1] = -size[1]/2.0; points[0][2] = -size[2]/2.0;
	points[1][0] =  size[0]/2.0; points[1][1] = -size[1]/2.0; points[1][2] = -size[2]/2.0;
	points[2][0] =  size[0]/2.0; points[2][1] =  size[1]/2.0; points[2][2] = -size[2]/2.0;
	points[3][0] = -size[0]/2.0; points[3][1] =  size[1]/2.0; points[3][2] = -size[2]/2.0;
	points[4][0] = -size[0]/2.0; points[4][1] = -size[1]/2.0; points[4][2] =  size[2]/2.0;
	points[5][0] =  size[0]/2.0; points[5][1] = -size[1]/2.0; points[5][2] =  size[2]/2.0;
	points[6][0] =  size[0]/2.0; points[6][1] =  size[1]/2.0; points[6][2] =  size[2]/2.0;
	points[7][0] = -size[0]/2.0; points[7][1] =  size[1]/2.0; points[7][2] =  size[2]/2.0;

	/*
	std::cerr << "Points: " << std::endl;

	for( int i = 0; i < 8; ++i )
		{

			std::cerr << "X: " << points[i][0] << ", Y: " << points[i][1] << ", Z: " << points[i][2] << std::endl;

		}
*/

	// Create transformation
	tTransform transform( tTransform::Identity() );
	transform *= Eigen::Translation3f( center );
	transform *=  orientation;

//	std::cerr << "Points: " << std::endl;

	// Transform box coordinates
	for( int i = 0; i < 8; ++i )
	{
		points[i] = transform * points[i];

//		std::cerr << "X: " << points[i][0] << ", Y: " << points[i][1] << ", Z: " << points[i][2] << std::endl;

	}

	// Create planes
	// Planes should be defined:
	//
	//   n
	//   | 3
	//   |/
	//   1----2



	// Bottom
	addPlane( CTestingPlane(points[0], points[1], points[3] ) );

	// Top
	addPlane( CTestingPlane(points[4], points[7], points[5] ) );

	// Front
	addPlane( CTestingPlane(points[0], points[4], points[1] ) );

	// Back
	addPlane( CTestingPlane(points[3], points[2], points[7] ) );

	// Left
	addPlane( CTestingPlane(points[0], points[3], points[4] ) );

	// Right
	addPlane( CTestingPlane(points[1], points[5], points[2] ) );
}

