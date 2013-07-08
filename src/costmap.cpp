/*
 * costmap.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include "rt_road_detection/costmap.h"

using namespace rt_road_detection;


TraversabilityCostmap::TraversabilityCostmap() {

	nh_ = ros::NodeHandle("~");

	it_.reset(new image_transport::ImageTransport(nh_));

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	not_road_sub_.subscribe(*it_,"not_road_in",1,hints);
	approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(5), not_road_sub_, disparity_sub_) );
	//approximate_sync_->registerCallback(boost::bind(TraversabilityCostmap::notRoadCB, this, _1, _2)); // GRRRRRRRRR

	//boost::shared_ptr<image_transport::Subscriber> sub;
	//sub.reset(new image_transport::Subscriber("not_road_in", 1, &TraversabilityCostmap::notRoadCB,this));
	//not_road_sub_.push_back(sub);

}

TraversabilityCostmap::~TraversabilityCostmap() {


}


void TraversabilityCostmap::camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info) {

	// update camera model
	model_.fromCameraInfo(cam_info);

	cam_info_received_ = true;

}


void TraversabilityCostmap::notRoadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImage& disp) {

	if (!cam_info_received_) return;

	ROS_INFO("not_road CB");


}

void TraversabilityCostmap::roadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImage& disp) {

	if (!cam_info_received_) return;


}
