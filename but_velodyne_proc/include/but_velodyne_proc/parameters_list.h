/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 01/07/2013
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
#ifndef BUT_VELODYNE_PARAMETERS_LIST_H
#define BUT_VELODYNE_PARAMETERS_LIST_H

#include <string>

namespace but_velodyne_proc
{
/**
 * laser scan - parameters
 */
const std::string FRAME_ID_PARAM    = "frame_id";
const std::string MIN_Z_PARAM       = "min_z";
const std::string MAX_Z_PARAM       = "max_z";
const std::string ANGULAR_RES_PARAM = "angular_res";
const std::string MIN_RANGE_PARAM   = "min_range";

/**
 * ground map - parameters
 */
//const std::string FRAME_ID_PARAM    = "frame_id";
const std::string MAP2D_RES_PARAM       = "map2d_res";
const std::string MAP2D_WIDTH_PARAM     = "map2d_width";
const std::string MAP2D_HEIGHT_PARAM    = "map2d_height";
//const std::string MIN_RANGE_PARAM       = "min_range";
const std::string MAX_RANGE_PARAM       = "max_range";
//const std::string ANGULAR_RES_PARAM     = "angular_res";
const std::string RADIAL_RES_PARAM      = "radial_res";
const std::string MAX_ROAD_IRREGULARITY_PARAM = "max_road_irregularity";
const std::string MAX_HEIGHT_DIFF_PARAM = "max_height_diff";
const std::string NOISE_FILTER_PARAM    = "noise_filter";
const std::string GROUND_PROB_PARAM     = "ground_prob";
const std::string OBSTACLE_PROB_PARAM   = "obstacle_prob";
}

#endif // BUT_VELODYNE_PARAMETERS_LIST_H
