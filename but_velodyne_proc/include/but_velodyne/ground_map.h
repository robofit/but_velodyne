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
#ifndef but_velodyne_proc_ground_map_H
#define but_velodyne_proc_ground_map_H

#include <ros/ros.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>

#include <velodyne_pointcloud/point_types.h>

// Include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

// ROS - PCL conversions
#include <pcl_conversions/pcl_conversions.h>


// Types of point and cloud to work with
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

// Instantiate template for transforming a VPointCloud
template bool pcl_ros::transformPointCloud<VPoint>(const std::string &, const VPointCloud &, VPointCloud &, const tf::TransformListener &);


namespace but_velodyne_proc
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

    //! Minimal distance used to filter points close to the robot [m]
    //! - Negative value means that no filtering is performed.
    double min_range;

    //! The maximum radius/distance from the center [m]
    double max_range;

    //! Angular resolution [degrees]
    double angular_res;

    //! Radial resolution [m/cell]
    double radial_res;

    //! Road irregularity threshold [m]
    double max_road_irregularity;

    //! Min-max height difference threshold [m]
    double max_height_diff;

    //! Enables simple noise filtering.
    bool noise_filter;

    //! Thresholds to identify ground and obstacles
    double ground_prob, obstacle_prob;

    //! Default constructor
    Params()
      : frame_id("")
      , map2d_res(getDefaultMapRes())
      , map2d_width(getDefaultMapSize())
      , map2d_height(getDefaultMapSize())
      , min_range(getDefaultMinRange())
      , max_range(getDefaultMaxRange())
      , angular_res(getDefaultAngularRes())
      , radial_res(getDefaultRadialRes())
      , max_road_irregularity(getDefaultMaxRoadIrregularity())
      , max_height_diff(getDefaultMaxHeightDiff())
      , noise_filter(true)
      , ground_prob(getDefaultGroundProb())
      , obstacle_prob(getDefaultObstacleProb())
    {}

    // Returns default values of particular parameters.
    static double getDefaultMapRes()
    {
      return 0.05;
    }
//        static int getDefaultMapSize() { return 128; }
    static int getDefaultMapSize()
    {
      return 256;
    }

//        static double getDefaultMinRange() { return 1.0; }
    static double getDefaultMinRange()
    {
      return 1.2;
    }
    static double getDefaultMaxRange()
    {
      return 5.0;
    }
    static double getDefaultAngularRes()
    {
      return 5.0;
    }
    static double getDefaultRadialRes()
    {
      return 0.25;
    }

    static double getDefaultMaxRoadIrregularity()
    {
      return 0.05;
    }
    static double getDefaultMaxHeightDiff()
    {
      return 0.1;
    }

    static double getDefaultGroundProb()
    {
      return 0.9;
    }
    static double getDefaultObstacleProb()
    {
      return 0.4;
    }
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

    //! Average height and variance.
    double avg, var;

    //! Average reflectivity.
    double ref_avg, ref_var;

    //! Helper values.
    double sum, sum_sqr;
    double ref_sum, ref_sum_sqr;
    double min_x, min_y;

    //! Number of samples accumulated in the bin.
    unsigned n;

    //! Region index
    unsigned idx;

    //! Estimated ground tilt/height correction.
    double tilt;

    //! Estimated ground probability
    double prob;

    //! Helper value
    unsigned v;

    //! Default constructor.
    PolarMapBin()
      : min(0.0), max(0.0)
      , avg(0.0), var(0.0)
      , ref_avg(0.0), ref_var(0.0)
      , sum(0.0), sum_sqr(0.0)
      , ref_sum(0.0), ref_sum_sqr(0.0)
      , min_x(0.0), min_y(0.0)
      , n(0)
      , idx(NOT_SET)
      , tilt(0.0), prob(0.0)
      , v(0)
    {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  //! Informations accumulated for sampling/map bin.
  struct RingMapBin
  {
    //! Average position.
    Eigen::Vector3d avg, var;
    double rad_avg, rad_var;

    //! Helper values.
    Eigen::Vector3d sum, sum_sqr;
    double rad_sum, rad_sum_sqr;

    //! Number of samples accumulated in the bin.
    unsigned n;

    //! Estimated ground probability
    double prob;

    //! Default constructor.
    RingMapBin()
      : avg(0, 0, 0), var(0, 0, 0)
      , rad_avg(0.0), rad_var(0.0)
      , sum(0, 0, 0), sum_sqr(0, 0, 0)
      , rad_sum(0.0), rad_sum_sqr(0.0)
      , n(0)
      , prob(0.0)
    {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  //! Seed used in region growing
  struct PolarMapSeed
  {
    //! Position in the map
    int ang, dist;

    //! Constructor.
    PolarMapSeed(int a = 0, int d = 0) : ang(a), dist(d) {}
  };

private:
  //! Conversion to polar coordinates.
  void toPolarCoords(float x, float y, float& ang, float &mag)
  {
    static const float rad_to_deg = 180.0f / float(CV_PI);
    mag = std::sqrt(x * x + y * y);
    ang = std::atan2(y, x) * rad_to_deg;
//        mag = cv::sqrt(x * x + y * y);
//        ang = cv::fastAtan2(y, x); // precision ~0.3 degrees
  }

  //! Returns index of bin in the polar map by converting given polar coordinates [deg, m]
  void getPolarMapIndex(float ang, float mag, int& a, int& d)
  {
    a = int((ang + 180.001f) * inv_angular_res_) % num_of_angular_bins_;
    d = int(mag * inv_radial_res_) % num_of_radial_bins_;
  }

  //! Returns subscripted bin of the polar map.
  PolarMapBin& getPolarMapBin(int a, int d)
  {
    return polar_map_[d * num_of_angular_bins_ + a];
  }

  //! Returns index of bin in the polar map
  void getRingMapIndex(float ang, int ring, int& a, int& r)
  {
    a = int((ang + 180.001f) * inv_angular_rmap_res_) % num_of_angular_rmap_bins_;
    r = int(ring - min_ring_index_);
  }

  //! Returns subscripted bin of the polar histogram.
  RingMapBin& getRingMapBin(int a, int r)
  {
    return ring_map_[r * num_of_angular_rmap_bins_ + a];
  }

  //! Returns value of Gaussian function
  template <typename T>
  T gaussVal(T x, T mean, T sigma)
  {
    //static const T inv_sqrt_2pi = 0.3989422804014327;
    T a = (x - mean) / sigma;
    return /*inv_sqrt_2pi / sigma **/ std::exp(-T(0.5) * a * a);
  }

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

  //! Current size of the polar map.
  int num_of_angular_bins_, num_of_radial_bins_;
  float inv_angular_res_, inv_radial_res_;

  //! Internal representation of a polar histogram
  typedef std::vector<RingMapBin> tRingMap;

  //! Polar histogram
  tRingMap ring_map_;

  //! Current size of the polar histogram.
  int num_of_angular_rmap_bins_;
  float inv_angular_rmap_res_;
  int min_ring_index_, max_ring_index_;
};


} // namespace but_velodyne_proc

#endif // but_velodyne_proc_ground_map_H
