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

// Include guard
#ifndef but_velodyne_proc_laser_scan_H
#define but_velodyne_proc_laser_scan_H

#include <ros/ros.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <velodyne_pointcloud/point_types.h>

// Include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>


// Types of point and cloud to work with
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

// Instantiate template for transforming a VPointCloud
template bool pcl_ros::transformPointCloud<VPoint>(const std::string &, const VPointCloud &, VPointCloud &, const tf::TransformListener &);


namespace but_velodyne_proc
{

/******************************************************************************
 *!
 * Simulation of sensor_msgs::LaserScan messages using point clouds
 * coming from Velodyne 3D LIDAR.
 */
class LaserScan
{
public:
  //! Configuration parameters
  struct Params
  {
    //! Target frame ID
    //! - An empty value means to use the same frame ID as the input point cloud has...
    std::string frame_id;

    //! Range to accumulate particular Velodyne scans
    //! - Specifying two exactly similar values means to accumulate all the Velodyne points...
    double min_z, max_z;

    //! Angular resolution [degrees]
    double angular_res;

    //! Minimal range used to remove points close to the robot.
    //! - Negative value means that the minimum will be obtained from the data.
    double min_range;

    //! Default constructor
    Params()
      : frame_id("")
      , min_z(getDefaultMinZ())
      , max_z(getDefaultMaxZ())
      , angular_res(getDefaultAngularRes())
      , min_range(getDefaultMinRange())
    {}

    //! Default parameter values.
    static double getDefaultMinZ()
    {
      return 0.0;
    }
    static double getDefaultMaxZ()
    {
      return 0.0;
    }
    static double getDefaultAngularRes()
    {
      return 0.1;
    }
    static double getDefaultMinRange()
    {
      return -1.0;
    }
  };

public:
  //! Default constructor.
  LaserScan(ros::NodeHandle nh, ros::NodeHandle private_nh);

  //! Virtual destructor.
  virtual ~LaserScan() {}

  //! Processes input Velodyne point cloud and publishes the output message
  virtual void process(const sensor_msgs::PointCloud2::ConstPtr &cloud);

private:
  //! Node handle
  ros::NodeHandle nh_, private_nh_;

  //! Parameters...
  Params params_;

  //! Point cloud buffer to avoid reallocation on every message.
  VPointCloud pcl_in_;

  // TF, message filters, etc.
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_filtered_;
  tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
  ros::Publisher scan_pub_;
  ros::Subscriber points_sub_;
  tf::TransformListener listener_;
};


} // namespace but_velodyne_proc

#endif // but_velodyne_proc_laser_scan_H
