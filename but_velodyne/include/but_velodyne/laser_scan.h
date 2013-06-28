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

// include guard
#ifndef but_velodyne_laser_scan_H
#define but_velodyne_laser_scan_H


#include <ros/ros.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

// instantiate template for transforming a VPointCloud
template bool
  pcl_ros::transformPointCloud<VPoint>(const std::string &,
                                       const VPointCloud &,
                                       VPointCloud &,
                                       const tf::TransformListener &);

namespace velodyne_pointcloud
{
  class Transform
  {
  public:

    Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Transform() {}

  private:

    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    message_filters::Subscriber<velodyne_msgs::VelodyneScan> velodyne_scan_;
    tf::MessageFilter<velodyne_msgs::VelodyneScan> *tf_filter_;
    ros::Publisher output_;
    tf::TransformListener listener_;

    /// configuration parameters
    typedef struct {
      std::string frame_id;          ///< target frame ID
    } Config;
    Config config_;

    // Point cloud buffers for collecting points within a packet.  The
    // inPc_ and tfPc_ are class members only to avoid reallocation on
    // every message.
    VPointCloud inPc_;              ///< input packet point cloud
    VPointCloud tfPc_;              ///< transformed packet point cloud
  };


} // namespace but_velodyne

#endif // but_velodyne_laser_scan_H

