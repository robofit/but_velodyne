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

#include <but_velodyne/laser_scan.h>
#include <but_velodyne/parameters_list.h>
#include <but_velodyne/topics_list.h>

namespace but_velodyne
{

/******************************************************************************
 */

LaserScan::LaserScan(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
{
    // Load parameters
    private_nh_.param( FRAME_ID_PARAM, params_.frame_id_, params_.frame_id_ );
    private_nh_.param( MIN_Z_PARAM, params_.min_z_, params_.min_z_ );
    private_nh_.param( MAX_Z_PARAM, params_.max_z_, params_.max_z_ );

    // If a tf_prefix param is specified, it will be added to the beginning of the frame ID
    std::string tf_prefix = tf::getPrefixParam( private_nh_ );
    params_.frame_id_ = tf::resolve( tf_prefix, params_.frame_id_ );

    ROS_INFO_STREAM( FRAME_ID_PARAM << " parameter: " << params_.frame_id_ );
    ROS_INFO_STREAM( MIN_Z_PARAM << " parameter: " << params_.min_z_ );
    ROS_INFO_STREAM( MAX_Z_PARAM << " parameter: " << params_.max_z_ );

    // Advertise output laser scan
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( OUTPUT_LASER_SCAN_TOPIC, 10 );

    // Subscribe to Velodyne point cloud
    if( params_.frame_id_.empty() )
    {
        // No TF frame ID conversion required
//        points_sub_ = nh_.subscribe(INPUT_POINT_CLOUD_TOPIC, 1, boost::bind(&LaserScan::process, this, _1) );
        points_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(INPUT_POINT_CLOUD_TOPIC, 1, &LaserScan::process, this );
    }
    else
    {
        points_sub_filtered_.subscribe(nh_, INPUT_POINT_CLOUD_TOPIC, 10);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>( points_sub_filtered_, listener_, params_.frame_id_, 10 );
        tf_filter_->registerCallback( boost::bind(&LaserScan::process, this, _1) );
    }
}


/******************************************************************************
 */
void LaserScan::process(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    if( scan_pub_.getNumSubscribers() == 0 )
    {
        return;
    }

    // Copy message header
    scan_out_.header.stamp = cloud->header.stamp;
    scan_out_.header.frame_id = params_.frame_id_;

    // allocate an output point cloud with same time as raw data
/*    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = scanMsg->header.stamp;
    outMsg->header.frame_id = config_.frame_id;
    outMsg->height = 1;

        // transform the packet point cloud into the target frame
        try
          {
            ROS_DEBUG_STREAM("transforming from" << inPc_.header.frame_id
                             << " to " << config_.frame_id);
            pcl_ros::transformPointCloud(config_.frame_id, inPc_, tfPc_,
                                         listener_);
#if 0       // use the latest transform available, should usually work fine
            pcl_ros::transformPointCloud(inPc_.header.frame_id,
                                         ros::Time(0), inPc_,
                                         config_.frame_id,
                                         tfPc_, listener_);
#endif
          }
        catch (tf::TransformException ex)
          {
            // only log tf error once every 100 times
            ROS_WARN_THROTTLE(100, "%s", ex.what());
            continue;                   // skip this packet
          }*/

    // Publish the accumulated laser scan
    ROS_DEBUG_STREAM_ONCE("Publishing laser scan " << scan_out_.header.stamp);
//    scan_pub_.publish(scan_out_);
}


} // namespace velodyne_pointcloud
