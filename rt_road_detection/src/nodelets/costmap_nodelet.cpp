/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 28/06/2013
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

#include <nodelet/nodelet.h>
#include "rt_road_detection/costmap.h"

namespace rt_road_detection {

	class CostmapNodelet : public nodelet::Nodelet
	   {

		boost::shared_ptr<TraversabilityCostmap> costmap_;

		virtual void onInit();

	   };


	void CostmapNodelet::onInit() {

		ros::NodeHandle &private_nh = getPrivateNodeHandle();

		costmap_.reset(new TraversabilityCostmap(private_nh));

		NODELET_INFO("TraversabilityCostmapNodelet loaded.");

	}

} // namespace

#include <pluginlib/class_list_macros.h>


// Register this plugin with pluginlib. Names must match nodelets.xml.
PLUGINLIB_EXPORT_CLASS(rt_road_detection::CostmapNodelet,nodelet::Nodelet)
