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
#ifndef _COLLAR_LINE_ODOM_H_
#define _COLLAR_LINE_ODOM_H_

#include <ros/ros.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

namespace but_velodyne_odom {
using namespace std; // for "log" macro

class CollarLineOdomNode {
    public:
        CollarLineOdomNode(ros::NodeHandle& private_nh);
        virtual ~CollarLineOdomNode();
        void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
        boost::shared_ptr<but_velodyne::MoveEstimator> estimator_;
        boost::shared_ptr<but_velodyne::CollarLinesRegistrationPipeline> registration_;
        but_velodyne::VelodynePointCloud::Ptr pcl_input_cloud_;
        boost::shared_ptr<std::ofstream> graph_file_;
        ros::Subscriber cloud_sub_;
        ros::Publisher odom_pub_;
        Eigen::Matrix4f cumulated_transformation_;

        bool b_save_file_;
        int msg_count_;
};

} // namespace but_velodyne_odom
#endif /* end of include guard */
