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
#ifndef TestingOrientedBox_H_included
#define TestingOrientedBox_H_included

#include "testing_object_base.h"


namespace but_env_model
{

class CTestingOrientedBox : public CTestingObjectBase
{
public:
  //! Constructor - simple
  CTestingOrientedBox();

  //! Constructor - initializing
  CTestingOrientedBox(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  //! Set by limits
  void set(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  //! Set by center and sizes
  void setCenterSize(double x, double y, double z, double sx, double sy, double sz);

  //! Testing function
  virtual bool isIn(double x, double y, double z);

protected:
  //! Minimal range
  double m_minX, m_minY, m_minZ;

  //! Maximal range
  double m_maxX, m_maxY, m_maxZ;

}; // CTestingOrientedBox

} // namespace but_env_model


// TestingOrientedBox_H_included
#endif


