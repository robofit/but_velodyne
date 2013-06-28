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

namespace but_velodyne
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    private_nh.param("frame_id", config_.frame_id, std::string("odom"));
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
    ROS_INFO_STREAM("target frame ID: " << config_.frame_id);

    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    // subscribe to VelodyneScan packets using transform filter
    velodyne_scan_.subscribe(node, "velodyne_packets", 10);
    tf_filter_ =
      new tf::MessageFilter<velodyne_msgs::VelodyneScan>(velodyne_scan_,
                                                         listener_,
                                                         config_.frame_id, 10);
    tf_filter_->registerCallback(boost::bind(&Transform::processScan, this, _1));
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate an output point cloud with same time as raw data
    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = scanMsg->header.stamp;
    outMsg->header.frame_id = config_.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    for (size_t next = 0; next < scanMsg->packets.size(); ++next)
      {
        // clear input point cloud to handle this packet
        inPc_.points.clear();
        inPc_.width = 0;
        inPc_.height = 1;
        inPc_.header.frame_id = scanMsg->header.frame_id;
        inPc_.header.stamp = scanMsg->packets[next].stamp;

        // unpack the raw data
        data_->unpack(scanMsg->packets[next], inPc_);

        // clear transform point cloud for this packet
        tfPc_.points.clear();           // is this needed?
        tfPc_.width = 0;
        tfPc_.height = 1;
        tfPc_.header.stamp = scanMsg->packets[next].stamp;
        tfPc_.header.frame_id = config_.frame_id;

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
          }

        // append transformed packet data to end of output message
        outMsg->points.insert(outMsg->points.end(),
                             tfPc_.points.begin(),
                             tfPc_.points.end());
        outMsg->width += tfPc_.points.size();
      }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud
