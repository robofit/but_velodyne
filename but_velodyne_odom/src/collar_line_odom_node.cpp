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

int main(int argc, char** argv) {
    ros::init(argc, argv, "but_velodyne_odom_collar_line_odom_node");
    ros::NodeHandle private_nh("~");
    but_velodyne_odom::CollarLineOdomNode node(private_nh);
    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("but_velodyne_odom_collar_line_odom_node exception: %s", e.what());
        return -1;
    }
    return 0;
}

