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
#ifndef TestingPolymesh_H_included
#define TestingPolymesh_H_included

#include "testing_planes.h"

namespace but_env_model
{

/**
 * Initializes planes stack from polygonal mesh objects
 */
class CTestingPolymesh : public CTestingPlanes
{
public:
  /// Point type
  typedef Eigen::Vector3f tPoint;

  /// Quaternion type
  typedef Eigen::Quaternionf tQuaternion;

  /// Transform type
  typedef Eigen::Transform<float, 3, Eigen::Affine> tTransform;

public:
  //! Simple constructor
  CTestingPolymesh();

  //! Construct as oriented box
  CTestingPolymesh(const tPoint &center, const tQuaternion &orientation, const tPoint & size);

  //! Create as oriented box
  void setAsBox(const tPoint &center, const tQuaternion &orientation, const tPoint & size);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class CTestingPolymesh


} // namespace but_env_model

// TestingPolymesh_H_included
#endif

