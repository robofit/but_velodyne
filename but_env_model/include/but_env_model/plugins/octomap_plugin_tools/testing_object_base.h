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
 #ifndef TestingObjectBase_H_included
 #define TestingObjectBase_H_included
 
 namespace but_env_model
 {
 
 /**
  *	Testing object interface definition
  */
 class CTestingObjectBase
 {
 public:
 	//! Test this point and return true if it is in the tested object
 	virtual bool isIn( double x, double y, double z ) = 0;
 };
 
 } // namespace but_env_model
 
 // TestingObjectBase_H_included
 #endif
 
 
