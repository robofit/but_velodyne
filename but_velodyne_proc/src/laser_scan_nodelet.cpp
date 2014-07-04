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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <but_velodyne_proc/laser_scan.h>


namespace but_velodyne_proc
{

class LaserScanNodelet: public nodelet::Nodelet
{
public:
  LaserScanNodelet() {}
  ~LaserScanNodelet() {}

private:
  virtual void onInit()
  {
    laser_scan_.reset(new LaserScan(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<LaserScan> laser_scan_;
};

} // namespace but_velodyne_proc


// Register this plugin with pluginlib. Names must match nodelets.xml.
PLUGINLIB_DECLARE_CLASS(but_velodyne_proc,
                        LaserScanNodelet,
                        but_velodyne_proc::LaserScanNodelet,
                        nodelet::Nodelet);
