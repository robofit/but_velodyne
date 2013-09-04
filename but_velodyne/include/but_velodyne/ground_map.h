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

// Include guard
#ifndef but_velodyne_ground_map_H
#define but_velodyne_ground_map_H

#include <ros/ros.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <velodyne_pointcloud/point_types.h>

// Include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>


// Types of point and cloud to work with
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

// Instantiate template for transforming a VPointCloud
template bool pcl_ros::transformPointCloud<VPoint>(const std::string &, const VPointCloud &, VPointCloud &, const tf::TransformListener &);


namespace but_velodyne
{

/******************************************************************************
 *!
 * Estimates and publishes occupancy grid representing "safe ground" around
 * the robot using point clouds coming from Velodyne 3D LIDAR.
 */
class GroundMap
{
public:
    //! Configuration parameters
    struct Params
    {
        // Parameters of the output 2D occupancy grid

        //! Target frame ID
        //! - An empty value means to use the same frame ID as the input point cloud has...
        std::string frame_id;

        //! Resolution of the map [m/cell]
        double map2d_res;

        //! Width and height of the map [cells]
        int map2d_width, map2d_height;

        // Parameters of internal spatial sampling of points in polar coordinates.

        //! Angular resolution [degrees]
        double angular_res;

        //! Radial resolution [m/cell]
        double radial_res;

        //! The maximum radius/distance from the center [m]
        double max_radius;

        //! Road irregularity threshold [m]
        double max_road_irregularity;

        //! Default constructor
        Params()
            : frame_id("")
            , map2d_res(getDefaultMapRes())
            , map2d_width(getDefaultMapSize())
            , map2d_height(getDefaultMapSize())
            , angular_res(getDefaultAngularRes())
            , radial_res(getDefaultRadialRes())
            , max_radius(getDefaultMaxRadius())
            , max_road_irregularity(getDefaultMaxRoadIrregularity())
        {}

        // Returns default values of particular parameters.
        static double getDefaultMapRes() { return 0.05; }
        static int getDefaultMapSize() { return 128; }
        static double getDefaultAngularRes() { return 0.5; }
        static double getDefaultRadialRes() { return 0.3; }
        static double getDefaultMaxRadius() { return 3.0; }
        static double getDefaultMaxRoadIrregularity() { return 0.02; }
    };

public:
    //! Default constructor.
    GroundMap(ros::NodeHandle nh, ros::NodeHandle private_nh);

    //! Virtual destructor.
    virtual ~GroundMap() {}

    //! Processes input Velodyne point cloud and publishes the output message
    virtual void process(const sensor_msgs::PointCloud2::ConstPtr &cloud);

private:
    //! Informations accumulated for each sampling/map bin
    struct PolarMapBin
    {
        //! Predefined region index values
        enum Indexes { NOT_SET = 0, FREE = 1, UNKNOWN = 2, OCCUPIED = 3 };

        //! Minimum and maximum height (i.e. z-coordinate).
        double min, max;

        //! Sum of all sample's heights.
        double avg;

        //! Number of samples accumulated in the bin.
        unsigned n;

        //! Region index
        unsigned idx;

        //! Default constructor.
        PolarMapBin() : min(0.0), max(0.0), avg(0.0), n(0), idx(NOT_SET) {}
    };

    struct PolarMapSeed
    {
        //! Position in the map
        int ang, dist;

        //! Constructor.
        PolarMapSeed(int a = 0, int d = 0) : ang(a), dist(d) {}
    };

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
    ros::Publisher map_pub_;
    ros::Subscriber points_sub_;
    tf::TransformListener listener_;

    //! Internal representation of a polar map
    typedef std::vector<PolarMapBin> tPolarMap;

    //! Map to avoid reallocation on every message
    tPolarMap polar_map_;
};


} // namespace but_velodyne

#endif // but_velodyne_ground_map_H
