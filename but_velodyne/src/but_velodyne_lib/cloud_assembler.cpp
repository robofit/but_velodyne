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

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <costmap_2d/cost_values.h>

namespace but_velodyne
{


CloudAssembler::CloudAssembler(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)

{

	points_sub_filtered_.subscribe( nh_, "/velodyne_points", 1 );
	tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>( points_sub_filtered_, listener_, "odom", 1 );
	tf_filter_->registerCallback( boost::bind(&CloudAssembler::process, this, _1) );

	points_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output", 1);


	cloud_buff_.reset(new CloudBuffer(10));



}

bool CloudAssembler::getRobotPose(ros::Time time, geometry_msgs::PoseStamped& res) {

	geometry_msgs::PoseStamped p;

	geometry_msgs::PoseStamped out;

	p.header.stamp = time;
	p.header.frame_id = "base_link";
	p.pose.position.x = 0;
	p.pose.position.y = 0;
	p.pose.position.z = 0;

	p.pose.orientation.x = 0;
	p.pose.orientation.y = 0;
	p.pose.orientation.z = 0;
	p.pose.orientation.w = 1;

	if (listener_.waitForTransform("odom",p.header.frame_id,p.header.stamp,ros::Duration(0.25))) {

		listener_.transformPose("odom",p,out);

		res = out;
		return true;

	} else {

		ROS_WARN("Cant get robot pose.");

	}

	return false;

}

void CloudAssembler::process(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    ROS_INFO_STREAM_ONCE( "CloudAssembler::process(): Point cloud received" );

    if( points_pub_.getNumSubscribers() == 0 )
    {
        return;
    }

    geometry_msgs::PoseStamped p;

    if (!getRobotPose(cloud->header.stamp,p)) return;

    bool update = false;

    double dist = sqrt(pow(robot_pose_.pose.position.x - p.pose.position.x, 2) + pow(robot_pose_.pose.position.y - p.pose.position.y, 2));

    if (dist > 0.05) update = true;

    VPointCloud vpcl;
    TPointCloudPtr tpcl(new TPointCloud());

    // Retrieve the input point cloud
    pcl::fromROSMsg( *cloud, vpcl );

    pcl::copyPointCloud(vpcl, *tpcl);

    pcl::PassThrough< TPoint > pass;

    pass.setInputCloud (tpcl);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-10.0, 10.0);
    pass.filter(*tpcl);

	pass.setInputCloud (tpcl);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-10.0, 10.0);
	pass.filter(*tpcl);

	pass.setInputCloud (tpcl);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.5, 2.0);
	pass.filter(*tpcl);

	pcl::ApproximateVoxelGrid<TPoint> psor;
	psor.setInputCloud (tpcl);
	psor.setDownsampleAllData (false);
	psor.setLeafSize (0.05, 0.05, 0.05);
	psor.filter(*tpcl);

    pcl::StatisticalOutlierRemoval< TPoint > foutl;
    foutl.setInputCloud(tpcl);
    foutl.setMeanK(50);
    foutl.setStddevMulThresh(1.0);
    foutl.filter(*tpcl);

    pcl_ros::transformPointCloud( "odom", *tpcl, *tpcl, listener_ );

    // get accumulated cloud
    TPointCloudPtr pcl_out(new TPointCloud());

    for (unsigned int i = 0; i < cloud_buff_->size(); i++) {

    	*pcl_out += cloud_buff_->at(i);

    }

    // registration
    if (cloud_buff_->size() > 0) {

    	pcl::IterativeClosestPoint< TPoint, TPoint> icp;
    	icp.setInputCloud(tpcl);
    	icp.setInputTarget(pcl_out);
    	pcl::PointCloud<TPoint> aligned;
    	icp.align(aligned);

    	if (icp.hasConverged()) {

    		*tpcl = aligned;
    		//std::cout <<  icp.getFitnessScore() << std::endl;

    	}

    }


    if (update) cloud_buff_->push_back(*tpcl);

    *pcl_out += *tpcl;

    pcl::ApproximateVoxelGrid<TPoint> sor;
    sor.setInputCloud (pcl_out);
    sor.setDownsampleAllData (false);
    sor.setLeafSize (0.05, 0.05, 0.05);

    TPointCloudPtr pcl_filt(new TPointCloud());

    sor.filter(*pcl_filt);

    sensor_msgs::PointCloud2::Ptr cloud_out(new sensor_msgs::PointCloud2());

    pcl::toROSMsg(*pcl_filt, *cloud_out);

    //std::cout << "points: " << pcl_out->points.size() << std::endl;

    cloud_out->header.stamp = cloud->header.stamp;
    cloud_out->header.frame_id = "odom";

    points_pub_.publish(cloud_out);


}


} // namespace but_velodyne
