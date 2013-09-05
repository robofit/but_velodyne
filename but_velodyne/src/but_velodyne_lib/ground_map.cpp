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

#include <but_velodyne/ground_map.h>
#include <but_velodyne/parameters_list.h>
#include <but_velodyne/topics_list.h>

#include <cmath>
//#include <opencv2/core/core.hpp>

namespace but_velodyne
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
{
    // Load parameters
    private_nh_.param( FRAME_ID_PARAM, params_.frame_id, params_.frame_id );
    private_nh_.param( MAP2D_RES_PARAM, params_.map2d_res, params_.map2d_res );
    private_nh_.param( MAP2D_WIDTH_PARAM, params_.map2d_width, params_.map2d_width );
    private_nh_.param( MAP2D_HEIGHT_PARAM, params_.map2d_height, params_.map2d_height );

    private_nh_.param( MIN_RANGE_PARAM, params_.min_range, params_.min_range );
    private_nh_.param( MAX_RANGE_PARAM, params_.max_range, params_.max_range );
    private_nh_.param( ANGULAR_RES_PARAM, params_.angular_res, params_.angular_res );
    private_nh_.param( RADIAL_RES_PARAM, params_.radial_res, params_.radial_res );

    private_nh_.param( MAX_ROAD_IRREGULARITY_PARAM, params_.max_road_irregularity, params_.max_road_irregularity );
    private_nh_.param( MAX_HEIGHT_DIFF_PARAM, params_.max_height_diff, params_.max_height_diff );

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

    ROS_INFO_STREAM( FRAME_ID_PARAM << " parameter: " << params_.frame_id );
    ROS_INFO_STREAM( MAP2D_RES_PARAM << " parameter: " << params_.map2d_res );
    ROS_INFO_STREAM( MAP2D_WIDTH_PARAM << " parameter: " << params_.map2d_width );
    ROS_INFO_STREAM( MAP2D_HEIGHT_PARAM << " parameter: " << params_.map2d_height );
    ROS_INFO_STREAM( MIN_RANGE_PARAM << " parameter: " << params_.min_range );
    ROS_INFO_STREAM( MAX_RANGE_PARAM << " parameter: " << params_.max_range );
    ROS_INFO_STREAM( ANGULAR_RES_PARAM << " parameter: " << params_.angular_res );
    ROS_INFO_STREAM( RADIAL_RES_PARAM << " parameter: " << params_.radial_res );
    ROS_INFO_STREAM( MAX_ROAD_IRREGULARITY_PARAM << " parameter: " << params_.max_road_irregularity );
    ROS_INFO_STREAM( MAX_HEIGHT_DIFF_PARAM << " parameter: " << params_.max_height_diff );

    // Advertise output occupancy grid
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>( OUTPUT_GROUND_MAP_TOPIC, 10 );

    // Subscribe to Velodyne point cloud
    if( params_.frame_id.empty() )
    {
        // No TF frame ID conversion required
        points_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(INPUT_POINT_CLOUD_TOPIC, 1, &GroundMap::process, this );
    }
    else
    {
        points_sub_filtered_.subscribe( nh_, INPUT_POINT_CLOUD_TOPIC, 10 );
        tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>( points_sub_filtered_, listener_, params_.frame_id, 10 );
        tf_filter_->registerCallback( boost::bind(&GroundMap::process, this, _1) );
    }
}


/******************************************************************************
 */
void GroundMap::process(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    ROS_INFO_STREAM_ONCE( "GroundMap::process(): Point cloud received" );

    if( map_pub_.getNumSubscribers() == 0 )
    {
        return;
    }

    // Output occupancy grid
    nav_msgs::OccupancyGrid::Ptr map_out;
    map_out = boost::make_shared<nav_msgs::OccupancyGrid>();

    // Retrieve the input point cloud
    pcl::fromROSMsg( *cloud, pcl_in_ );

    // Copy message header
    map_out->header.stamp = cloud->header.stamp;

    // Point cloud origin
    float center_x = 0.0f, center_y = 0.0f, center_z = 0.0f;

    // Target TF frame ID
    if( params_.frame_id.empty() )
    {
        // No TF transformation required
        map_out->header.frame_id = cloud->header.frame_id;
    }
    else
    {
        // Prescribed frame ID
        map_out->header.frame_id = params_.frame_id;
        if( map_out->header.frame_id != cloud->header.frame_id )
        {
            // Get TF transform
            tf::StampedTransform to_target_frame_tf;
            try
            {
                // Map origin
                geometry_msgs::PointStamped center;
                center.header = pcl_in_.header;
                center.point.x = center.point.y = center.point.z = 0.0;

                ROS_INFO_STREAM_ONCE( "Transforming point cloud from " << pcl_in_.header.frame_id
                                      << " to " << map_out->header.frame_id
                                      );

                // Transform the point cloud
                pcl_ros::transformPointCloud( map_out->header.frame_id, pcl_in_, pcl_in_, listener_ );

//                ROS_INFO_STREAM_ONCE( "Transforming center from " << center.header.frame_id
//                                      << " to " << map_out->header.frame_id
//                                      );

                // Transform the map origin
                listener_.transformPoint( map_out->header.frame_id, center, center );
                center_x = center.point.x;
                center_y = center.point.y;
                center_z = center.point.z;

//                ROS_INFO_STREAM( "New map center: " << center_x << ", " << center_y << ", " << center_z );
            }
            catch( tf::TransformException ex )
            {
                ROS_WARN_STREAM( "Cannot transform the point cloud!" );
                return;
            }
        }
    }

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
    for( size_t i = 0; i < polar_map_.size(); ++i )
    {
        polar_map_[i] = PolarMapBin();
    }

    // Accumulate all input points into the polar map
    VPointCloud::iterator itEnd = pcl_in_.end();
    for( VPointCloud::iterator it = pcl_in_.begin(); it != itEnd; ++it )
    {
//        ROS_INFO_STREAM("Point: " << it->x << ", " << it->y << ", " << it->z);

        float x = it->x - center_x;
        float y = it->y - center_y;
        float z = it->z - center_z;

        // Conversion to the polar coordinates
        float ang, mag;
        toPolarCoords(x, y, ang, mag);

//        ROS_INFO_STREAM("Polar coords: " << mag << ", " << ang);

        // Check the distance
        if( mag > params_.max_range )
            continue;
        if( params_.min_range > 0.0 && mag < params_.min_range )
            continue;

        // Find the corresponding map bin
        int an, rn;
        getPolarMapIndex(ang, mag, an, rn);

//        ROS_INFO_STREAM("Polar bin: " << an << ", " << rn);

        // Accumulate the value
        PolarMapBin &bin = getPolarMapBin(an, rn);
        bin.n += 1;
        bin.sum += z;
        bin.sum_sqr += z * z;
        if( bin.n > 1 )
        {
            bin.min = (z < bin.min) ? z : bin.min;
            bin.max = (z > bin.max) ? z : bin.max;
        }
        else
        {
            bin.max = bin.min = z;
        }

        if( bin.dst_ring == -1 )
        {
            bin.dst_n += 1;
            bin.dst_sum += mag;
            bin.dst_sum_sqr += mag * mag;
            bin.dst_ring = int(it->ring);
        }
        else if( bin.dst_ring == int(it->ring) )
        {
            bin.dst_n += 1;
            bin.dst_sum += mag;
            bin.dst_sum_sqr += mag * mag;
        }
    }

    // Compute stats and mark all UNKNOWN bins
    tPolarMap::iterator mitEnd = polar_map_.end();
    for( tPolarMap::iterator mit = polar_map_.begin(); mit != mitEnd; ++mit )
    {
        // Mark all the map bins where very few samples were accumulated as unknown
        if( mit->n < MIN_NUM_OF_SAMPLES )
        {
            mit->idx = PolarMapBin::UNKNOWN;
            continue;
        }

        // Calculate the average height in the bin, ...
        double inv_n = 1.0 / mit->n;
        mit->avg = mit->sum * inv_n;
        mit->var = (mit->sum_sqr - mit->sum * mit->sum * inv_n) * inv_n;

        inv_n = 1.0 / mit->dst_n;
        mit->dst_avg = mit->dst_sum * inv_n;
        mit->dst_var = (mit->dst_sum_sqr - mit->dst_sum * mit->dst_sum * inv_n) * inv_n;
    }

    // Interpolate empty cells if possible
    for( int rn = 1; rn < (num_of_radial_bins_ - 1); ++rn )
    {
        for( int an = 0; an < num_of_angular_bins_; ++an )
        {
            PolarMapBin &bin = getPolarMapBin(an, rn);

            if( bin.idx == PolarMapBin::UNKNOWN )
            {
                // Get bin's neighbours
                PolarMapBin &n1 = getPolarMapBin(an, rn + 1);
                PolarMapBin &n2 = getPolarMapBin(an, rn - 1);

                // Interpolate values if possible
                if( n1.idx != PolarMapBin::UNKNOWN && n2.idx != PolarMapBin::UNKNOWN )
                {
                    bin.n = n1.n + n2.n;
                    bin.min = 0.5 * (n1.min + n2.min);
                    bin.max = 0.5 * (n1.max + n2.max);
                    bin.avg = 0.5 * (n1.avg + n2.avg);
                    bin.var = (n1.var > n2.var) ? n1.var : n2.var;
                    bin.idx = PolarMapBin::NOT_SET;
                }
            }
        }
    }

    // Find all OCCUPIED bins
    for( tPolarMap::iterator mit = polar_map_.begin(); mit != mitEnd; ++mit )
    {
        if( mit->idx == PolarMapBin::UNKNOWN )
        {
            continue;
        }

        // Mark all the map bins where the difference between minimal
        // and maximal height is significant as occupied
        if( std::fabs(mit->max - mit->min) > params_.max_height_diff )
        {
            mit->idx = PolarMapBin::OCCUPIED;
        }

        // Variance in distance from the center
        if( mit->dst_var > params_.max_road_irregularity * params_.max_road_irregularity )
        {
            mit->idx = PolarMapBin::OCCUPIED;
        }
    }

    typedef std::vector<double> tDblVec;
    tDblVec heights;
    heights.reserve(num_of_angular_bins_);

    // For each angle in polar coordinates find the closest
    // bin to the center that has a known height
    int min_rn = num_of_radial_bins_ - 1;
    for( int an = 0; an < num_of_angular_bins_; ++an )
    {
        for( int rn = 0; rn < num_of_radial_bins_; ++rn )
        {
            PolarMapBin &bin = getPolarMapBin(an, rn);

            if( bin.idx == PolarMapBin::NOT_SET )
            {
                heights.push_back(bin.min);
//                heights.push_back(bin.avg);
                min_rn = (rn < min_rn) ? rn : min_rn;
                break;
            }
        }
    }

    if( heights.empty() )
    {
        ROS_WARN_STREAM( "Cannot estimate ground height!" );
        return;
    }

    // Assume that the median of minimum height in the closest bins
    // is the ground height
    std::sort(heights.begin(), heights.end());
    double ground_height = heights[heights.size() / 2];

    // Ground tilt correction
/*    for( int an = 0; an < (num_of_angular_bins_ / 2 + 1); ++an )
    {
        for( int rn = 0; rn < num_of_radial_bins_; ++rn )
        {
            PolarMapBin &b1 = getPolarMapBin(an, rn);
            PolarMapBin &b2 = getPolarMapBin((an + num_of_angular_bins_ / 2) % num_of_angular_bins_, rn);

            if( b1.idx != PolarMapBin::UNKNOWN && b2.idx != PolarMapBin::UNKNOWN )
            {
            }
        }
    }*/

//    ROS_INFO_STREAM( "Estimated ground height = " << ground_height );

    unsigned ground_bins = 0;
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
                double diff = std::fabs(bin.avg - ground_height);
//                double diff = std::fabs(bin.min - ground_height);
                if( diff < params_.max_road_irregularity )
                {
                    bin.idx = PolarMapBin::FREE;
                    Seeds.push_back(PolarMapSeed(an, rn));
                    ++ground_bins;
                }
            }
        }
    }

//    ROS_INFO_STREAM( "Initial queue size = " << Seeds.size() );

    // Grow from the seeds
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
                    double diff = std::fabs(bin.avg - n1.avg);
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
                double diff = std::fabs(bin.avg - n2.avg);
                double diff2 = std::fabs(bin.dst_avg - n2.dst_avg);
                if( diff < params_.max_road_irregularity
                    && diff2 < params_.max_road_irregularity )
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
                double diff = std::fabs(bin.avg - n3.avg);
                double diff2 = std::fabs(bin.dst_avg - n3.dst_avg);
                if( diff < params_.max_road_irregularity
                    && diff2 < params_.max_road_irregularity )
                {
                    n3.idx = PolarMapBin::FREE;
                    Seeds.push_back(PolarMapSeed(an3, s.dist));
                    ++ground_bins;
                }
            }
        }
    }

    if( ground_bins == 0 )
    {
        ROS_WARN_STREAM( "Cannot estimate any safe ground around the robot!" );
        return;
    }

    // Initialize the occupancy grid
    map_out->data.resize(params_.map2d_width * params_.map2d_height);
    for( int j = 0; j < params_.map2d_height; ++j )
    {
        for( int i = 0; i < params_.map2d_width; ++i )
        {
            map_out->data[j * params_.map2d_width + i] = -1;
        }
    }

    float map_res = float(params_.map2d_res);

    // Fill the occupancy grid according to the polar map
    float y = -float(params_.map2d_height / 2) * map_res;
    for( int j = 0; j < params_.map2d_height; ++j, y += map_res )
    {
        float x = -float(params_.map2d_width / 2) * map_res;
        for( int i = 0; i < params_.map2d_width; ++i, x += map_res )
        {
            // Conversion to the polar coordinates
            float ang, mag;
            toPolarCoords(x, y, ang, mag);

            // Check the distance
            if( mag > params_.max_range )
                continue;

            // Find the corresponding polar map bin
            int an, rn;
            getPolarMapIndex(ang, mag, an, rn);

            // Fill in the 2D occupancy grid
            PolarMapBin &bin = getPolarMapBin(an, rn);
            switch( bin.idx )
            {
//                case PolarMapBin::NOT_SET:
                case PolarMapBin::UNKNOWN:
                    map_out->data[j * params_.map2d_width + i] = -1;
                    break;

                case PolarMapBin::FREE:
                    map_out->data[j * params_.map2d_width + i] = 0;
                    break;

                case PolarMapBin::NOT_SET:
                case PolarMapBin::OCCUPIED:
                    map_out->data[j * params_.map2d_width + i] = 100;
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
    ROS_INFO_STREAM_ONCE( "Publishing ground map " << map_out->header.stamp );

    map_pub_.publish( map_out );
}


} // namespace but_velodyne
