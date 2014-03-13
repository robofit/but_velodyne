/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
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
#ifndef ENV_MODEL_PERCP_PARAMETERS_LIST_H
#define ENV_MODEL_PERCP_PARAMETERS_LIST_H

#include <string>


namespace but_bb_estimator
{

    const int OUTLIERS_PERCENT_DEFAULT          = 10;

    /**
     * bb_estimator - Default percentage of rows and columns considered for sampling
     * (for calculation of statistics)
     */
    const int SAMPLING_PERCENT_DEFAULT       = 30;

    /**
      * bb_estimator - default scene (world) frame id
      */
    const std::string GLOBAL_FRAME_DEFAULT      = "/map";

    /**
     * bb_estimator - The required maximum ratio of sides length (the longer
     * side is at maximum sidesRatio times longer than the shorter one)
     */
    const double SIDES_RATIO_DEFAULT            = 5.0;
    
}

#endif
