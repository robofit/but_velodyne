/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 04/09/2013
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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <but_velodyne_proc/cloud_assembler.h>

namespace but_velodyne_proc {

class CloudAssemblerNodelet: public nodelet::Nodelet {
public:
	CloudAssemblerNodelet() {
	}
	~CloudAssemblerNodelet() {
	}

private:
	virtual void onInit() {
		cloud_assembler_.reset(
				new CloudAssembler(getNodeHandle(), getPrivateNodeHandle()));
	}

	boost::shared_ptr<CloudAssembler> cloud_assembler_;
};

} // namespace but_velodyne_proc

// Register this plugin with pluginlib. Names must match nodelets.xml.
PLUGINLIB_DECLARE_CLASS(but_velodyne_proc, CloudAssemblerNodelet,
		but_velodyne_proc::CloudAssemblerNodelet, nodelet::Nodelet);

