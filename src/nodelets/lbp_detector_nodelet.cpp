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
#include "rt_road_detection/detectors/lbp_detector_ros.h"

namespace rt_road_detection {

	class LBPDetectorNodelet : public nodelet::Nodelet
	   {

		boost::shared_ptr<LBPDetectorRos> det_;

		virtual void onInit();

	   };


	void LBPDetectorNodelet::onInit() {

		ros::NodeHandle &private_nh = getPrivateNodeHandle();

		det_.reset(new LBPDetectorRos(private_nh));

		NODELET_INFO("SampleHueDetectorNodelet loaded.");

	}

} // namespace

#include <pluginlib/class_list_macros.h>


// Register this plugin with pluginlib. Names must match nodelets.xml.
PLUGINLIB_EXPORT_CLASS(rt_road_detection::LBPDetectorNodelet,nodelet::Nodelet)
