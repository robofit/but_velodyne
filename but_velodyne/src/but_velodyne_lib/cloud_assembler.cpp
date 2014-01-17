/******************************************************************************
 * \file
 *
 * $Id:$
 * *
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

#include <but_velodyne/cloud_assembler.h>
#include <but_velodyne/parameters_list.h>
#include <but_velodyne/topics_list.h>

#include <cmath>
//#include <opencv2/core/core.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <costmap_2d/cost_values.h>

namespace but_velodyne
{


/******************************************************************************
 */

CloudAssembler::CloudAssembler(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)

{

	points_sub_filtered_.subscribe( nh_, "/velodyne_points", 10 );
	tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>( points_sub_filtered_, listener_, "odom", 10 );
	tf_filter_->registerCallback( boost::bind(&CloudAssembler::process, this, _1) );

	points_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output", 1);


	cloud_buff_.reset(new CloudBuffer(5));

}

/******************************************************************************
 */
void CloudAssembler::process(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    ROS_INFO_STREAM_ONCE( "CloudAssembler::process(): Point cloud received" );

    if( points_pub_.getNumSubscribers() == 0 )
    {
        return;
    }

    //! Point cloud buffer to avoid reallocation on every message.
    VPointCloud vpcl;
    TPointCloudPtr tpcl(new TPointCloud());

    // Retrieve the input point cloud
    pcl::fromROSMsg( *cloud, vpcl );

    std::cout << "fromROSMsg ok" << std::endl;

    pcl::copyPointCloud(vpcl, *tpcl);

    std::cout << "copy ok" << std::endl;

    pcl_ros::transformPointCloud( "odom", *tpcl, *tpcl, listener_ );

    std::cout << "tf ok" << std::endl;

    cloud_buff_->push_back(*tpcl);

    TPointCloudPtr pcl_out(new TPointCloud());

    //pcl::copyPointCloud(pcl_in_, *pcl_out);

    //if (cloud_buff_->size() > 1)
    for (unsigned int i = 0; i < cloud_buff_->size(); i++) {

    	*pcl_out += cloud_buff_->at(i);

    	//pcl_out.points.insert(cloud_buff_->at(i).end(), pcl_out.points.begin(), pcl_out.points.end());

    }

    std::cout << "merge ok" << std::endl;

    //pcl::VoxelGrid< TPoint > sor;
    pcl::ApproximateVoxelGrid<TPoint> sor;
    sor.setInputCloud (pcl_out);
    sor.setDownsampleAllData (false);
    sor.setLeafSize (0.01, 0.01, 0.01);

    std::cout << "before filter" << std::endl;

    TPointCloudPtr pcl_filt(new TPointCloud());

    sor.filter(*pcl_filt);

    std::cout << "filter ok" << std::endl;

    sensor_msgs::PointCloud2::Ptr cloud_out(new sensor_msgs::PointCloud2());



    pcl::toROSMsg(*pcl_filt, *cloud_out);

    std::cout << "toROSMsg ok" << std::endl;

    std::cout << "points: " << pcl_out->points.size() << std::endl;

    cloud_out->header.stamp = cloud->header.stamp;
    cloud_out->header.frame_id = "odom";

    points_pub_.publish(cloud_out);


}


} // namespace but_velodyne
