/*
 * sample_hue_detector_ros.cpp
 *
 *  Created on: 12.7.2013
 *      Author: zdenal
 */

#include "rt_road_detection/detectors/hsv_hist_detector_ros.h"


using namespace rt_road_detection;

HSVHistDetectorRos::HSVHistDetectorRos(ros::NodeHandle private_nh) {

	nh_ = private_nh;

	it_.reset(new image_transport::ImageTransport(nh_));


	//int hbins = 18, int sbins = 25, int wnd_size = 30, int wnd_step = 5

	nh_.param("hbins",hbins_,18);
	nh_.param("shbins",sbins_,25);
	nh_.param("wnd_size",wnd_size_,30);
	nh_.param("wnd_step",wnd_step_,5);

	nh_.param("prob_hit",prob_hit_,0.6);
	nh_.param("prob_miss",prob_miss_,0.4);

	nh_.param("frame_skip",frame_skip_,2);

	nh_.param<std::string>("filename",fn_,"");

	skiped_ = 0;

	det_.reset(new HSVHistDetector(prob_hit_, prob_miss_, hbins_, sbins_, wnd_size_, wnd_step_));

	if (fn_ != "") det_->read(fn_);
	else ROS_ERROR("Please set filename param for hsv_dist detector!!!");

	det_->setWnd( wnd_size_, wnd_step_);		// overwrite params stored during training

	std::string top_rgb_in = "rgb_in";
	std::string top_det_out = "det_out";

	if (top_rgb_in == ros::names::remap(top_rgb_in)) ROS_WARN("Topic %s was not remapped!",top_rgb_in.c_str());
	else ROS_INFO("Topic %s remapped to %s.",top_rgb_in.c_str(),ros::names::remap(top_rgb_in).c_str());

	if (top_det_out == ros::names::remap(top_det_out)) ROS_WARN("Topic %s was not remapped!",top_det_out.c_str());
	else ROS_INFO("Topic %s remapped to %s.",top_det_out.c_str(),ros::names::remap(top_det_out).c_str());

	//image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
	sub_ = it_->subscribe(ros::names::remap(top_rgb_in), 1, &HSVHistDetectorRos::imageCallback,this);
	pub_ = it_->advertise(ros::names::remap(top_det_out), 1);

	dyn_reconf_f_ = boost::bind(&HSVHistDetectorRos::reconfigureCallback, this, _1, _2);
	dyn_reconf_srv_.setCallback(dyn_reconf_f_);

}

HSVHistDetectorRos::~HSVHistDetectorRos() {


}

void HSVHistDetectorRos::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	if (skiped_ < frame_skip_) {

		skiped_++;
		return;

	} else skiped_ = 0;

	cv_bridge::CvImageConstPtr rgb;

	try {

		rgb = cv_bridge::toCvShare(msg, msg->encoding); // TODO use toCvShare

	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR_THROTTLE(1.0, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  return;
	}

	ROS_INFO_ONCE("Received first RGB image.");

	if (pub_.getNumSubscribers() == 0) return;

	ROS_INFO_ONCE("Publishing first detection.");

	cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);

	cv::Mat hsv;

	if (rgb->encoding == "rgb8") cv::cvtColor( rgb->image, hsv, CV_RGB2HSV );  // Hue in range 0-360
	else if ((rgb->encoding == "bgr8")) cv::cvtColor( rgb->image, hsv, CV_BGR2HSV );
	else {

	  ROS_WARN_THROTTLE(1,"Strange encoding!");
	  return;
	}

	cv::Mat_<float> bin_mask(hsv.size());

	det_->detect(hsv, bin_mask);

	out_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	//out->encoding = sensor_msgs::image_encodings::MONO8;
	out_msg->header = msg->header;
	out_msg->image = bin_mask;

	pub_.publish(out_msg->toImageMsg());

}

void HSVHistDetectorRos::reconfigureCallback(rt_road_detection::HSVHistDetectorConfig &config, uint32_t level) {


	ROS_INFO("Reconfigure callback.");

}
