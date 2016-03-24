/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Shohei Fujii (fujii.shohei@gmail.com)
 * Date: 03/16/2016
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
#include <but_velodyne_odom/collar_line_odom.h>
#include <nodelet/nodelet.h>

namespace but_velodyne_odom {
class CollarLineOdomNodelet : public nodelet::Nodelet{
  public:
    CollarLineOdomNodelet() {}
    virtual ~CollarLineOdomNodelet() {}
  private:
    virtual void onInit()
    {
      odom_node_.reset(new CollarLineOdomNode(getPrivateNodeHandle()));
    }
    boost::shared_ptr<CollarLineOdomNode> odom_node_;
};
    
} // namespace but_velodyne_odom

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(but_velodyne_odom,
                        CollarLineOdomNodelet,
                        but_velodyne_odom::CollarLineOdomNodelet,
                        nodelet::Nodelet);
