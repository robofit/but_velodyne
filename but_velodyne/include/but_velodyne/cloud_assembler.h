/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * * This file is part of software developed by Robo@FIT group.
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

// Include guard
#ifndef but_velodyne_cloud_assembler_H
#define but_velodyne_cloud_assembler_H

#include <ros/ros.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>

#include <velodyne_pointcloud/point_types.h>

// Include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

// ROS - PCL conversions
#include <pcl_conversions/pcl_conversions.h>

#include <boost/circular_buffer.hpp>

#include <pcl/filters/voxel_grid.h>


// Types of point and cloud to work with
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef boost::shared_ptr<VPointCloud> VPointCloudPtr;

typedef boost::circular_buffer<VPointCloud> CloudBuffer;

// Instantiate template for transforming a VPointCloud
template bool pcl_ros::transformPointCloud<VPoint>(const std::string &, const VPointCloud &, VPointCloud &, const tf::TransformListener &);


namespace but_velodyne
{

/******************************************************************************
 *!
 * Estimates and publishes occupancy grid representing "safe ground" around
 * the robot using point clouds coming from Velodyne 3D LIDAR.
 */
class CloudAssembler
{

public:
    //! Default constructor.
	CloudAssembler(ros::NodeHandle nh, ros::NodeHandle private_nh);

    //! Virtual destructor.
    virtual ~CloudAssembler() {}

    //! Processes input Velodyne point cloud and publishes the output message
    virtual void process(const sensor_msgs::PointCloud2::ConstPtr &cloud);

private:
    //! Node handle
    ros::NodeHandle nh_, private_nh_;

    //! Point cloud buffer to avoid reallocation on every message.
    VPointCloud pcl_in_;

    // TF, message filters, etc.
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_filtered_;
    tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
    ros::Subscriber points_sub_;
    ros::Publisher points_pub_;
    tf::TransformListener listener_;

    boost::shared_ptr<CloudBuffer> cloud_buff_;

};


} // namespace but_velodyne

#endif // but_velodyne_cloud_assembler_H
