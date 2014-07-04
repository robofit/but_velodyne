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

#include <but_velodyne_proc/ground_map.h>
#include <but_velodyne_proc/parameters_list.h>
#include <but_velodyne_proc/topics_list.h>

#include <cmath>
//#include <opencv2/core/core.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <costmap_2d/cost_values.h>

namespace but_velodyne_proc
{

static const unsigned MIN_NUM_OF_SAMPLES    = 3;

/******************************************************************************
 */

GroundMap::GroundMap(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , num_of_angular_bins_(0)
  , num_of_radial_bins_(0)
  , inv_angular_res_(0.0f)
  , inv_radial_res_(0.0f)
  , num_of_angular_rmap_bins_(0)
  , inv_angular_rmap_res_(0.0f)
  , min_ring_index_(0)
  , max_ring_index_(31)
{
  // Load parameters
  private_nh_.param(FRAME_ID_PARAM, params_.frame_id, params_.frame_id);
  private_nh_.param(MAP2D_RES_PARAM, params_.map2d_res, params_.map2d_res);
  private_nh_.param(MAP2D_WIDTH_PARAM, params_.map2d_width, params_.map2d_width);
  private_nh_.param(MAP2D_HEIGHT_PARAM, params_.map2d_height, params_.map2d_height);

  private_nh_.param(MIN_RANGE_PARAM, params_.min_range, params_.min_range);
  private_nh_.param(MAX_RANGE_PARAM, params_.max_range, params_.max_range);
  private_nh_.param(ANGULAR_RES_PARAM, params_.angular_res, params_.angular_res);
  private_nh_.param(RADIAL_RES_PARAM, params_.radial_res, params_.radial_res);

  private_nh_.param(MAX_ROAD_IRREGULARITY_PARAM, params_.max_road_irregularity, params_.max_road_irregularity);
  private_nh_.param(MAX_HEIGHT_DIFF_PARAM, params_.max_height_diff, params_.max_height_diff);
  private_nh_.param(NOISE_FILTER_PARAM, params_.noise_filter, params_.noise_filter);

  private_nh_.param(GROUND_PROB_PARAM, params_.ground_prob, params_.ground_prob);
  private_nh_.param(OBSTACLE_PROB_PARAM, params_.obstacle_prob, params_.obstacle_prob);

  // Check if all the parameters are valid
  params_.map2d_res = (params_.map2d_res > 0.001) ? params_.map2d_res : 0.001;
  params_.map2d_height = (params_.map2d_height >= 0) ? params_.map2d_height : 0;
  params_.map2d_width = (params_.map2d_width >= 0) ? params_.map2d_width : 0;
  params_.max_range = (params_.max_range > 0.0) ? params_.max_range : 0.0;
  params_.angular_res = (params_.angular_res > 0.01) ? params_.angular_res : 0.01;
  params_.radial_res = (params_.radial_res > 0.01) ? params_.radial_res : 0.01;
  params_.max_road_irregularity = (params_.max_road_irregularity > 0.0) ? params_.max_road_irregularity : 0.0;
  params_.max_height_diff = (params_.max_height_diff > 0.0) ? params_.max_height_diff : 0.0;

  // If a tf_prefix param is specified, it will be added to the beginning of the frame ID
//    std::string tf_prefix = tf::getPrefixParam( private_nh_ );
//    if( !tf_prefix.empty() )
//    {
//        params_.frame_id = tf::resolve( tf_prefix, params_.frame_id);
//    }

  ROS_INFO_STREAM(FRAME_ID_PARAM << " parameter: " << params_.frame_id);
  ROS_INFO_STREAM(MAP2D_RES_PARAM << " parameter: " << params_.map2d_res);
  ROS_INFO_STREAM(MAP2D_WIDTH_PARAM << " parameter: " << params_.map2d_width);
  ROS_INFO_STREAM(MAP2D_HEIGHT_PARAM << " parameter: " << params_.map2d_height);
  ROS_INFO_STREAM(MIN_RANGE_PARAM << " parameter: " << params_.min_range);
  ROS_INFO_STREAM(MAX_RANGE_PARAM << " parameter: " << params_.max_range);
  ROS_INFO_STREAM(ANGULAR_RES_PARAM << " parameter: " << params_.angular_res);
  ROS_INFO_STREAM(RADIAL_RES_PARAM << " parameter: " << params_.radial_res);
  ROS_INFO_STREAM(MAX_ROAD_IRREGULARITY_PARAM << " parameter: " << params_.max_road_irregularity);
  ROS_INFO_STREAM(MAX_HEIGHT_DIFF_PARAM << " parameter: " << params_.max_height_diff);
  ROS_INFO_STREAM(NOISE_FILTER_PARAM << " parameter: " << params_.noise_filter);
  ROS_INFO_STREAM(GROUND_PROB_PARAM << " parameter: " << params_.ground_prob);
  ROS_INFO_STREAM(OBSTACLE_PROB_PARAM << " parameter: " << params_.obstacle_prob);

  // Advertise output occupancy grid
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(OUTPUT_GROUND_MAP_TOPIC, 10);

  // Subscribe to Velodyne point cloud
  if (params_.frame_id.empty())
  {
    // No TF frame ID conversion required
    points_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(INPUT_POINT_CLOUD_TOPIC, 1, &GroundMap::process, this);
  }
  else
  {
    points_sub_filtered_.subscribe(nh_, INPUT_POINT_CLOUD_TOPIC, 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(points_sub_filtered_, listener_, params_.frame_id, 10);
    tf_filter_->registerCallback(boost::bind(&GroundMap::process, this, _1));
  }
}


/******************************************************************************
 */
void GroundMap::process(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
  ROS_INFO_STREAM_ONCE("GroundMap::process(): Point cloud received");

  if (map_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  // Output occupancy grid
  nav_msgs::OccupancyGrid::Ptr map_out;
  map_out = boost::make_shared<nav_msgs::OccupancyGrid>();

  // Retrieve the input point cloud
  pcl::fromROSMsg(*cloud, pcl_in_);

  // Copy message header
  map_out->header.stamp = cloud->header.stamp;

  // Point cloud origin
  float center_x = 0.0f, center_y = 0.0f, center_z = 0.0f;

  // Target TF frame ID
  if (params_.frame_id.empty())
  {
    // No TF transformation required
    map_out->header.frame_id = cloud->header.frame_id;
  }
  else
  {
    // Prescribed frame ID
    map_out->header.frame_id = params_.frame_id;
    if (map_out->header.frame_id != cloud->header.frame_id)
    {
      // Get TF transform
      tf::StampedTransform to_target_frame_tf;
      try
      {
        // Map origin
        geometry_msgs::PointStamped center;
        center.header = pcl_conversions::fromPCL(pcl_in_.header);
        center.point.x = center.point.y = center.point.z = 0.0;

        ROS_INFO_STREAM_ONCE("Transforming point cloud from " << pcl_in_.header.frame_id
                             << " to " << map_out->header.frame_id
                            );

        // Transform the point cloud
        pcl_ros::transformPointCloud(map_out->header.frame_id, pcl_in_, pcl_in_, listener_);

//                ROS_INFO_STREAM_ONCE( "Transforming center from " << center.header.frame_id
//                                      << " to " << map_out->header.frame_id
//                                      );

        // Transform the map origin
        listener_.transformPoint(map_out->header.frame_id, center, center);
        center_x = center.point.x;
        center_y = center.point.y;
        center_z = center.point.z;

//                ROS_INFO_STREAM( "New map center: " << center_x << ", " << center_y << ", " << center_z );
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN_STREAM("Cannot transform the point cloud!");
        return;
      }
    }
  }


  ///// POLAR MAP create a simplified version of the map for the ground plane estimation

  // Calculate the number of angular sampling bins
  num_of_angular_bins_ = int(360 / params_.angular_res);
  float angular_res = 360.0f / num_of_angular_bins_;
  inv_angular_res_ = 1.0f / angular_res;

  // Calculate the number of sampling bins along the polar axis
  num_of_radial_bins_ = int(params_.max_range / params_.radial_res);
  float radial_res = float(params_.max_range / num_of_radial_bins_);
  inv_radial_res_ = 1.0f / radial_res;

  // Create and initialize the map sampling bins
  polar_map_.resize(num_of_angular_bins_ * num_of_radial_bins_);
  for (size_t i = 0; i < polar_map_.size(); ++i)
  {
    polar_map_[i] = PolarMapBin();
  }

  // Accumulate all input points into the polar map
  VPointCloud::iterator itEnd = pcl_in_.end();
  for (VPointCloud::iterator it = pcl_in_.begin(); it != itEnd; ++it)
  {
    float x = it->x - center_x;
    float y = it->y - center_y;
    float z = it->z - center_z;

    // Conversion to the polar coordinates
    float ang, mag;
    toPolarCoords(x, y, ang, mag);

    // POLAR MAP

    // Check the distance
    if (mag > params_.max_range)
      continue;
    if (params_.min_range > 0.0 && mag < params_.min_range)
      continue;

    // Find the corresponding map bin
    int an, rn;
    getPolarMapIndex(ang, mag, an, rn);

    // Accumulate the value
    PolarMapBin &bin = getPolarMapBin(an, rn);
    bin.n += 1;
    if (bin.n == 1)
    {
      bin.min = z;
      bin.min_x = x;
      bin.min_y = y;
    }
    else
    {
      if (z < bin.min)
      {
        bin.min = z;
        bin.min_x = x;
        bin.min_y = y;
      }
    }
  }


  ///// GROUND PLANE ESTIMATION and tilt correction

  // Ground points
  pcl::PointCloud<pcl::PointXYZ> ground_cloud;

  // Fill in the cloud data
  ground_cloud.width = 1;
  ground_cloud.height = 1;
  for (int rn = 0; rn < num_of_radial_bins_; ++rn)
  {
    for (int an = 0; an < num_of_angular_bins_; ++an)
    {
      PolarMapBin &bin = getPolarMapBin(an, rn);

      if (bin.n > 0)
      {
        ground_cloud.width++;
        ground_cloud.points.push_back(pcl::PointXYZ(bin.min_x, bin.min_y, bin.min));
      }
    }
  }

  // Estimate the ground plane using PCL and RANSAC
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(50);
  seg.setInputCloud(ground_cloud.makeShared());
  seg.segment(*inliers, *coefficients);

//    ROS_INFO_STREAM( "Num. of ground inliers = " << inliers->indices.size() );

  if (inliers->indices.size() == 0)
  {
    ROS_WARN_STREAM("Could not estimate the ground plane!");
    return;
  }

  // Estimate ground height in the center of the polar map
  double ground_height = -coefficients->values[3] / coefficients->values[2];


  ///// POLAR MAP and POLAR RING MAP initialization

  // Clear the polar map sampling bins
  for (size_t i = 0; i < polar_map_.size(); ++i)
  {
    polar_map_[i] = PolarMapBin();
  }

  // Calculate the number of angular sampling bins
  num_of_angular_rmap_bins_ = 360;
  float angular_rmap_res = 360.0f / num_of_angular_rmap_bins_;
  inv_angular_rmap_res_ = 1.0f / angular_rmap_res;

  // Create and initialize the bins
  int num_of_rings = max_ring_index_ - min_ring_index_ + 1;
  ring_map_.resize(num_of_angular_rmap_bins_ * num_of_rings);
  for (size_t i = 0; i < ring_map_.size(); ++i)
  {
    ring_map_[i] = RingMapBin();
  }

  // Accumulate all input points into the polar maps
  for (VPointCloud::iterator it = pcl_in_.begin(); it != itEnd; ++it)
  {
//        ROS_INFO_STREAM( "Point: " << it->x << ", " << it->y << ", " << it->z );

    float x = it->x - center_x;
    float y = it->y - center_y;
    float z = it->z - center_z;

    // Ground tilt correction
    double tilt = (-coefficients->values[0] * x - coefficients->values[1] * y - coefficients->values[3]) / coefficients->values[2];
    z -= tilt;

//        ROS_INFO_STREAM( "Tilt correction: " << tilt );

    // Conversion to the polar coordinates
    float ang, mag;
    toPolarCoords(x, y, ang, mag);

//        ROS_INFO_STREAM( "Polar coords: " << mag << ", " << ang );

    // RING MAP

    // Find the corresponding map bin
    int ah, rh;
    if (it->ring > max_ring_index_)
    {
      max_ring_index_ = it->ring;
      num_of_rings = max_ring_index_ - min_ring_index_ + 1;
      ring_map_.resize(num_of_angular_rmap_bins_ * num_of_rings, RingMapBin());

      ROS_INFO_STREAM("Ring map resized: " << num_of_rings);
    }
    getRingMapIndex(ang, it->ring, ah, rh);

//        ROS_INFO_STREAM( "Polar hist bin: " << an << ", " << rn );

    // Accumulate the value
    RingMapBin &hbin = getRingMapBin(ah, rh);
    Eigen::Vector3d p(x, y, z);
    float rad = std::sqrt(x * x + y * y + z * z);
    hbin.n += 1;
    hbin.sum += p;
    hbin.sum_sqr += Eigen::Vector3d(x * x, y * y, z * z);
    hbin.rad_sum += rad;
    hbin.rad_sum_sqr += rad * rad;
    if (hbin.n == 1)
    {
      hbin.rad_avg = rad;
    }
    else
    {
      hbin.rad_avg = (rad < hbin.rad_avg) ? rad : hbin.rad_avg;
    }

    // POLAR MAP

    // Check the distance
    if (mag > params_.max_range)
      continue;
    if (params_.min_range > 0.0 && mag < params_.min_range)
      continue;

    // Find the corresponding map bin
    int an, rn;
    getPolarMapIndex(ang, mag, an, rn);

//        ROS_INFO_STREAM( "Polar map bin: " << an << ", " << rn );

    // Accumulate the value
    PolarMapBin &bin = getPolarMapBin(an, rn);
    bin.n += 1;
    bin.sum += z;
    bin.sum_sqr += z * z;
    bin.ref_sum += it->intensity;
    bin.ref_sum_sqr += it->intensity * it->intensity;
    if (bin.n == 1)
    {
      bin.max = bin.min = z;
    }
    else
    {
      bin.min = (z < bin.min) ? z : bin.min;
      bin.max = (z > bin.max) ? z : bin.max;
    }
  }


  ///// POLAR MAP stats

  // Compute stats and mark all UNKNOWN bins in the polar map
  tPolarMap::iterator mitEnd = polar_map_.end();
  for (tPolarMap::iterator mit = polar_map_.begin(); mit != mitEnd; ++mit)
  {
    // Mark all the map bins where very few samples were accumulated as unknown
    if (mit->n < MIN_NUM_OF_SAMPLES)
    {
      mit->idx = PolarMapBin::UNKNOWN;
      continue;
    }

    // Calculate the average height in the bin, ...
    double inv_n = 1.0 / mit->n;
    mit->avg = mit->sum * inv_n;
    mit->var = (mit->sum_sqr - mit->sum * mit->sum * inv_n) * inv_n;
    mit->ref_avg = mit->ref_sum * inv_n;
    mit->ref_var = (mit->ref_sum_sqr - mit->ref_sum * mit->ref_sum * inv_n) * inv_n;
  }

  // Interpolate empty cells if possible
  for (int rn = 1; rn < (num_of_radial_bins_ - 1); ++rn)
  {
    for (int an = 0; an < num_of_angular_bins_; ++an)
    {
      PolarMapBin &bin = getPolarMapBin(an, rn);

      if (bin.idx == PolarMapBin::UNKNOWN)
      {
        // Get bin's neighbours
        PolarMapBin &n1 = getPolarMapBin(an, rn + 1);
        PolarMapBin &n2 = getPolarMapBin(an, rn - 1);

        // Interpolate values if possible
        if (n1.idx != PolarMapBin::UNKNOWN && n2.idx != PolarMapBin::UNKNOWN)
        {
          bin.n = (n1.n < n2.n) ? n1.n : n2.n;
          bin.min = 0.5 * (n1.min + n2.min);
          bin.max = 0.5 * (n1.max + n2.max);
          bin.avg = 0.5 * (n1.avg + n2.avg);
          bin.var = (n1.var > n2.var) ? n1.var : n2.var;
          bin.ref_avg = 0.5 * (n1.ref_avg + n2.ref_avg);
          bin.ref_var = (n1.ref_var > n2.ref_var) ? n1.ref_var : n2.ref_var;
          bin.idx = PolarMapBin::NOT_SET;
        }
      }
    }
  }


  ///// POLAR RING MAP stats

  // Compute stats stored in the ring map
  tRingMap::iterator hitEnd = ring_map_.end();
  for (tRingMap::iterator hit = ring_map_.begin(); hit != hitEnd; ++hit)
  {
    if (hit->n < 1)
//        if( hit->n < MIN_NUM_OF_SAMPLES )
    {
      continue;
    }

    // Calculate the average position, ...
    double inv_n = 1.0 / hit->n;
    hit->avg = hit->sum * inv_n;
    hit->var = (hit->sum_sqr - (hit->sum.cwiseProduct(hit->sum) * inv_n)) * inv_n;
//        hit->rad_avg = hit->rad_sum * inv_n;
//        hit->rad_var = (hit->rad_sum_sqr - (hit->rad_sum * hit->rad_sum * inv_n)) * inv_n;
  }


  ///// GROUND MODEL

  static const double COEFF = 0.3;
//    static const double COEFF = 0.5;

  // Estimate ground roughness and reflectivity
  double reflectivity_sum = 0.0, reflectivity_sum_sqr = 0.0;
  double roughness_sum = 0.0, roughness_sum_sqr = 0.0;
  unsigned ground_points = 0;
  for (int rn = 0; rn < num_of_radial_bins_; ++rn)
  {
    for (int an = 0; an < num_of_angular_bins_; ++an)
    {
      PolarMapBin &bin = getPolarMapBin(an, rn);

      // Does the bin correspond to the ground height?
      if (bin.idx != PolarMapBin::UNKNOWN)
      {
        if (std::fabs(bin.avg) < COEFF * params_.max_road_irregularity)
        {
          /*                    ground_points += bin.n;
                              reflectivity_sum += bin.ref_sum;
                              reflectivity_sum_sqr += bin.ref_sum_sqr;
                              roughness_sum += bin.sum;
                              roughness_sum_sqr += bin.sum_sqr;*/
          ground_points += 1;
          reflectivity_sum += bin.ref_avg;
          reflectivity_sum_sqr += bin.ref_avg * bin.ref_avg;
//                    roughness_sum += bin.avg;
//                    roughness_sum_sqr += bin.avg * bin.avg;
          roughness_sum += bin.var;
          roughness_sum_sqr += bin.var * bin.var;
        }
      }
    }
  }

//    ROS_INFO_STREAM( "Num. of ground points = " << ground_points );

  if (ground_points == 0)
  {
    ROS_WARN_STREAM("Could not estimate the ground model!");
    return;
  }

  double inv_n = 1.0 / ground_points;
  double reflectivity_mean = reflectivity_sum * inv_n;
  double reflectivity_sigma = std::sqrt((reflectivity_sum_sqr - (reflectivity_sum * reflectivity_sum * inv_n)) * inv_n);
  double roughness_mean = roughness_sum * inv_n;
  double roughness_sigma = std::sqrt((roughness_sum_sqr - (roughness_sum * roughness_sum * inv_n)) * inv_n);

  // Estimate edginess along particular rings
  double edginess_sum = 0.0, edginess_sum_sqr = 0.0;
  ground_points = 0;
  for (int rh = 0; rh < num_of_rings; ++rh)
  {
    for (int ah = 0; ah < num_of_angular_rmap_bins_; ++ah)
    {
      RingMapBin &hbin = getRingMapBin(ah, rh);
      RingMapBin &hbin2 = getRingMapBin((ah + num_of_angular_rmap_bins_ - 1) % num_of_angular_rmap_bins_, rh);

      if (hbin.n >= 1 && hbin2.n >= 1)
//            if( hbin.n >= MIN_NUM_OF_SAMPLES && hbin2.n >= MIN_NUM_OF_SAMPLES )
      {
        if (std::fabs(hbin.avg.z()) < COEFF * params_.max_road_irregularity)
        {
          ground_points += 1;
          double diff = std::fabs(hbin.rad_avg - hbin2.rad_avg);
          edginess_sum += diff;
          edginess_sum_sqr += diff * diff;
        }
      }
    }
  }

  if (ground_points == 0)
  {
    ROS_WARN_STREAM("Could not estimate the ground model!!");
    return;
  }

  inv_n = 1.0 / ground_points;
  double edginess_mean = edginess_sum * inv_n;
  double edginess_sigma = std::sqrt((edginess_sum_sqr - (edginess_sum * edginess_sum * inv_n)) * inv_n);


  ///// POLAR MAP Estimate ground probabilities

  for (int rn = 0; rn < num_of_radial_bins_; ++rn)
  {
    for (int an = 0; an < num_of_angular_bins_; ++an)
    {
      PolarMapBin &bin = getPolarMapBin(an, rn);

      // Does the bin correspond to the ground height?
      if (bin.idx != PolarMapBin::UNKNOWN)
      {
        // Reflectivity
        bin.prob = gaussVal<double>(bin.ref_avg, reflectivity_mean, reflectivity_sigma);
        bin.v += 1;

        // Roughness
//                bin.prob *= gaussVal<double>(bin.avg, roughness_mean, roughness_sigma);
        bin.prob *= gaussVal<double>(bin.var, roughness_mean, roughness_sigma);
        bin.v += 1;
      }
    }
  }


  ///// RING MAP Estimate ground probabilities

  for (int rh = 0; rh < num_of_rings; ++rh)
  {
    double sum = getRingMapBin(num_of_angular_rmap_bins_ - 1, rh).rad_avg;
//        sum += getRingMapBin(num_of_angular_rmap_bins_ - 2, rh).rad_avg;
//        sum += getRingMapBin(num_of_angular_rmap_bins_ - 3, rh).rad_avg;
//        sum *= 0.33333;

    for (int ah = 0; ah < num_of_angular_rmap_bins_; ++ah)
    {
      RingMapBin &hbin = getRingMapBin(ah, rh);

      if (hbin.n >= 1)
//            if( hbin.n >= MIN_NUM_OF_SAMPLES )
      {
        hbin.prob = gaussVal<double>(std::fabs(hbin.rad_avg - sum), edginess_mean, edginess_sigma);
//                hbin.prob = gaussVal<double>(hbin.rad_var, edginess_mean, edginess_sigma);
      }

      // Update the floating mean
//            int ah2 = (ah + num_of_angular_rmap_bins_ - 3) % num_of_angular_rmap_bins_;
//            sum -= 0.33333 * getRingMapBin(ah2, rh).rad_avg;
//            sum += 0.33333 * hbin.rad_avg;
      sum = hbin.rad_avg;
    }
  }

  // Project ground probabilities in the ring map to the polar map...
  for (int rh = 0; rh < num_of_rings; ++rh)
  {
    for (int ah = 0; ah < num_of_angular_rmap_bins_; ++ah)
    {
      RingMapBin &hbin = getRingMapBin(ah, rh);

      // Get the average bin position and calculate his polar coordinates
      float ang, mag;
      toPolarCoords(float(hbin.avg.x()), float(hbin.avg.y()), ang, mag);

//            ROS_INFO_STREAM( "Updating polar map according to the ring map: " << mag << ", " << ang );

      // Check the distance
      if (mag > params_.max_range)
        continue;
      if (params_.min_range > 0.0 && mag < params_.min_range)
        continue;

      // Find the corresponding map bin
      int an, rn;
      getPolarMapIndex(ang, mag, an, rn);

      // Accumulate values
      PolarMapBin &bin = getPolarMapBin(an, rn);
      if (bin.idx != PolarMapBin::UNKNOWN)
      {
//                bin.prob *= hbin.prob;
//                bin.v += 1;
      }
    }
  }


  ///// POLAR MAP

  // Thresholds
  double ground_thr[11], obstacle_thr[11];
  ground_thr[0] = obstacle_thr[0] = 1;
  ground_thr[1] = params_.ground_prob;
  obstacle_thr[1] = params_.obstacle_prob;
  for (unsigned i = 2; i <= 10; ++i)
  {
    ground_thr[i] = ground_thr[i - 1] * params_.ground_prob;
    obstacle_thr[i] = obstacle_thr[i - 1] * params_.obstacle_prob;
  }

  // Evaluate bins
  for (tPolarMap::iterator mit = polar_map_.begin(); mit != mitEnd; ++mit)
  {
    if (mit->idx == PolarMapBin::UNKNOWN)
    {
      continue;
    }

    // Mark all the map bins where the difference between minimal
    // and maximal height is too large as occupied...
    if (std::fabs(mit->max - mit->min) > params_.max_height_diff)
    {
      mit->idx = PolarMapBin::OCCUPIED;

//            ROS_INFO_STREAM( "Obstacle prob: " << mit->prob );
    }

    // Mark all map bins where both the minimal and maximal height lie
    // within the road irregularity tolerance as free...
    if (std::fabs(mit->max - mit->min) < params_.max_road_irregularity
        && std::fabs(mit->max) < params_.max_road_irregularity
        && std::fabs(mit->min) < params_.max_road_irregularity)
    {
      mit->idx = PolarMapBin::FREE;

//            ROS_INFO_STREAM( "Ground prob: " << mit->prob );
    }

    // No probability estimated
    if (mit->v == 0 || mit->v > 10)
    {
      mit->idx = PolarMapBin::UNKNOWN;
      continue;
    }

//        ROS_INFO_STREAM( "Prob.: " << mit->prob );

    // Decide according to the probabilities...
    if (mit->prob > ground_thr[mit->v])
    {
      mit->idx = PolarMapBin::FREE;
    }
    else if (mit->prob < obstacle_thr[mit->v])
    {
      mit->idx = PolarMapBin::OCCUPIED;
    }
  }

  // POLAR MAP ground region growing

  /*    unsigned ground_bins = 0;
      typedef std::list<PolarMapSeed> tSeeds;
      tSeeds Seeds;

      // Traverse the map and add all ground bins as seeds
      for( int rn = min_rn; rn < num_of_radial_bins_; ++rn )
      {
          for( int an = 0; an < num_of_angular_bins_; ++an )
          {
              PolarMapBin &bin = getPolarMapBin(an, rn);

              // Does the bin correspond to the ground height?
              if( bin.idx == PolarMapBin::NOT_SET )
              {
                  if( std::fabs(bin.avg) < 0.5 * params_.max_road_irregularity )
  //                if( std::fabs(bin.avg - ground_height) < 0.5 * params_.max_road_irregularity )
                  {
                      bin.idx = PolarMapBin::FREE;
                      Seeds.push_back(PolarMapSeed(an, rn));
                      ++ground_bins;
                  }
              }
          }
      }

  //    ROS_INFO_STREAM( "Initial queue size = " << Seeds.size() );

      // Grow the ground from the seeds
      while( !Seeds.empty() )
      {
          // Retrieve the seed from the top
          PolarMapSeed s = Seeds.front();
          Seeds.pop_front();

          // Get the corresponding bin
          PolarMapBin &bin = getPolarMapBin(s.ang, s.dist);

          // Let's try to grow the ground
          if( bin.idx == PolarMapBin::FREE )
          {
              // 1st neighbour
              if( s.dist < (num_of_radial_bins_ - 1) )
              {
                  PolarMapBin &n1 = getPolarMapBin(s.ang, s.dist + 1);
                  if( n1.idx == PolarMapBin::NOT_SET )
                  {
  //                    double diff = std::fabs(bin.avg - n1.avg);
                      double max = (bin.max > n1.max) ? bin.max : n1.max;
                      double min = (bin.min < n1.min) ? bin.min : n1.min;
                      double diff = max - min;
                      if( diff < params_.max_road_irregularity )
                      {
                          n1.idx = PolarMapBin::FREE;
                          Seeds.push_back(PolarMapSeed(s.ang, s.dist + 1));
                          ++ground_bins;
                      }
                  }
              }

              // 2nd neighbour
              int an2 = (s.ang + 1) % num_of_angular_bins_;
              PolarMapBin &n2 = getPolarMapBin(an2, s.dist);
              if( n2.idx == PolarMapBin::NOT_SET )
              {
  //                double diff = std::fabs(bin.avg - n2.avg);
                  double max = (bin.max > n2.max) ? bin.max : n2.max;
                  double min = (bin.min < n2.min) ? bin.min : n2.min;
                  double diff = max - min;
                  if( diff < params_.max_road_irregularity )
                  {
                      n2.idx = PolarMapBin::FREE;
                      Seeds.push_back(PolarMapSeed(an2, s.dist));
                      ++ground_bins;
                  }
              }

              // 3rd neighbour
              int an3 = (s.ang + num_of_angular_bins_ - 1) % num_of_angular_bins_;
              PolarMapBin &n3 = getPolarMapBin(an3, s.dist);
              if( n3.idx == PolarMapBin::NOT_SET )
              {
  //                double diff = std::fabs(bin.avg - n3.avg);
                  double max = (bin.max > n3.max) ? bin.max : n3.max;
                  double min = (bin.min < n3.min) ? bin.min : n3.min;
                  double diff = max - min;
                  if( diff < params_.max_road_irregularity )
                  {
                      n3.idx = PolarMapBin::FREE;
                      Seeds.push_back(PolarMapSeed(an3, s.dist));
                      ++ground_bins;
                  }
              }
          }
      }*/

  /*    if( ground_bins == 0 )
      {
          ROS_WARN_STREAM( "Cannot estimate any safe ground around the robot!" );
          return;
      }*/


  ///// Simple noise filter

  if (params_.noise_filter)
  {
    int num_of_filtered_points = 0;
    for (int rn = 1; rn < (num_of_radial_bins_ - 1); ++rn)
    {
      for (int an = 0; an < num_of_angular_bins_; ++an)
      {
        PolarMapBin &bin = getPolarMapBin(an, rn);

        // Does the bin correspond to the ground height?
        if (bin.idx == PolarMapBin::OCCUPIED)
        {
          int an1 = (an + 1) % num_of_angular_bins_;
          int an2 = (an + num_of_angular_bins_ - 1) % num_of_angular_bins_;

          // Num of free bins around
          int n = 0;
          n += (getPolarMapBin(an, rn + 1).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an1, rn + 1).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an2, rn + 1).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an, rn - 1).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an1, rn - 1).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an2, rn - 1).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an1, rn).idx == PolarMapBin::FREE) ? 1 : 0;
          n += (getPolarMapBin(an2, rn).idx == PolarMapBin::FREE) ? 1 : 0;

          // "All" values around must be marked as free
          if (n >= 7)
//                    if( n >= 8 )
          {
            // Filter the occupied bin
            bin.idx = PolarMapBin::FREE;
            ++num_of_filtered_points;
          }
        }
      }
    }
//        ROS_INFO_STREAM( "Num. of filtered bins = " << num_of_filtered_points );
  }

  // Initialize the occupancy grid
  map_out->data.resize(params_.map2d_width * params_.map2d_height);
  for (int j = 0; j < params_.map2d_height; ++j)
  {
    for (int i = 0; i < params_.map2d_width; ++i)
    {
      map_out->data[j * params_.map2d_width + i] = -1;
    }
  }

  float map_res = float(params_.map2d_res);

  // Fill the occupancy grid according to the polar map
  float y = -float(params_.map2d_height / 2) * map_res;
  for (int j = 0; j < params_.map2d_height; ++j, y += map_res)
  {
    float x = -float(params_.map2d_width / 2) * map_res;
    for (int i = 0; i < params_.map2d_width; ++i, x += map_res)
    {
      // Conversion to the polar coordinates
      float ang, mag;
      toPolarCoords(x, y, ang, mag);

      // Check the distance
      if (mag > params_.max_range)
        continue;

      // Find the corresponding polar map bin
      int an, rn;
      getPolarMapIndex(ang, mag, an, rn);

      // Fill in the 2D occupancy grid
      PolarMapBin &bin = getPolarMapBin(an, rn);
      switch (bin.idx)
      {
      case PolarMapBin::NOT_SET:
      case PolarMapBin::UNKNOWN:
        map_out->data[j * params_.map2d_width + i] = -1;
//                    map_out->data[j * params_.map2d_width + i] = costmap_2d::NO_INFORMATION;
        break;

      case PolarMapBin::FREE:
        map_out->data[j * params_.map2d_width + i] = 0;
//                    map_out->data[j * params_.map2d_width + i] = costmap_2d::FREE_SPACE;
        break;

//                case PolarMapBin::NOT_SET:
      case PolarMapBin::OCCUPIED:
        map_out->data[j * params_.map2d_width + i] = 100;
//                    map_out->data[j * params_.map2d_width + i] = costmap_2d::LETHAL_OBSTACLE;
        break;
      }
    }
  }

  // Fill in all message members
  map_out->info.map_load_time = map_out->header.stamp;
  map_out->info.width = uint32_t(params_.map2d_width);
  map_out->info.height = uint32_t(params_.map2d_height);
  map_out->info.resolution = float(params_.map2d_res);
  map_out->info.origin.position.x = -float(params_.map2d_width / 2) * map_res + center_x;
  map_out->info.origin.position.y = -float(params_.map2d_height / 2) * map_res + center_y;
  map_out->info.origin.position.z = ground_height + center_z;
  map_out->info.origin.orientation.w = 1;
  map_out->info.origin.orientation.x = 0;
  map_out->info.origin.orientation.y = 0;
  map_out->info.origin.orientation.z = 0;

  // Publish the map
  ROS_INFO_STREAM_ONCE("Publishing ground map " << map_out->header.stamp);

  map_pub_.publish(map_out);
}


} // namespace but_velodyne_proc
