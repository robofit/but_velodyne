/*
 * lbp_detector_ros.cpp
 *
 *  
 *  Author: xgoldm03 <xgoldm03@stud.fit.vutbr.cz
 */

#include "rt_road_detection/detectors/lbp_detector_ros.h"


using namespace rt_road_detection;

LBPDetectorRos::LBPDetectorRos(ros::NodeHandle private_nh) {

	nh_ = private_nh;

	it_.reset(new image_transport::ImageTransport(nh_));

	int width_block,width_cell,height_block,height_cell;
	
	double prob_max_, prob_min_, flat_surface_in_block,prob_overexposure,svm_threshold;
	
	string fileName;
	string svm_file="svm.xml";
	
	//param for histogram compute
	nh_.param("width_cell",width_cell,16);
	nh_.param("width_block",width_block,64);
	nh_.param("height_cell",height_cell,16);
	nh_.param("height_block",height_block,64);

	
	nh_.param("svm_file",fileName,svm_file);
	nh_.param("svm_threshold",svm_threshold,0.0);
	nh_.param("frame_skip",frame_skip_,2);
	
	
	//probability param
	nh_.param("prob_min",prob_min_, 0.3);
	nh_.param("prob_max",prob_max_, 0.7);
	nh_.param("prob_overexposure",prob_overexposure, 0.5);
	
	nh_.param("flat_surface_in_block",flat_surface_in_block, 0.3);
	
	//nh_.param("flat_surface_avg_color",flat_surface_avg_color, 220);

	skiped_ = 0;

	ifstream fin(fileName.c_str());
	
	if (!fin)  // check to see if file exists
	{
	  ROS_ERROR("File with coeficients for SVM classifier %s doesn't exists",fileName.c_str());
	}
	
	fin.close();
	
	ROS_INFO("PARAM: width cell: %d height cell: %d width block: %d height block: %d", width_cell, height_cell, width_block, height_block);
	
	det_.reset(new LBPDetector(width_cell,height_cell,width_block,height_block,prob_min_,prob_max_,flat_surface_in_block,prob_overexposure,svm_threshold,fileName));

	
	std::string top_rgb_in = "rgb_in";
	std::string top_det_out = "det_out";

	if (top_rgb_in == ros::names::remap(top_rgb_in)) ROS_WARN("Topic %s was not remapped!",top_rgb_in.c_str());
	else ROS_INFO("Topic %s remapped to %s.",top_rgb_in.c_str(),ros::names::remap(top_rgb_in).c_str());

	if (top_det_out == ros::names::remap(top_det_out)) ROS_WARN("Topic %s was not remapped!",top_det_out.c_str());
	else ROS_INFO("Topic %s remapped to %s.",top_det_out.c_str(),ros::names::remap(top_det_out).c_str());

	//image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
	sub_ = it_->subscribe(ros::names::remap(top_rgb_in), 1, &LBPDetectorRos::imageCallback,this);
	pub_ = it_->advertise(ros::names::remap(top_det_out), 1);

	dyn_reconf_f_ = boost::bind(&LBPDetectorRos::reconfigureCallback, this, _1, _2);
	dyn_reconf_srv_.setCallback(dyn_reconf_f_);

}

LBPDetectorRos::~LBPDetectorRos() {


}

void LBPDetectorRos::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

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

	//if (pub_.getNumSubscribers() == 0) return;


	cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);

	det_->map(rgb,out_msg);
	
	ROS_INFO_ONCE("Publishing first detection.");

	pub_.publish(out_msg->toImageMsg());

}

void LBPDetectorRos::reconfigureCallback(rt_road_detection::LBPDetectorConfig &config, uint32_t level) {

	
	det_->setParams(config.width_cell,config.height_cell,config.width_block,config.height_block, config.svm_threshold,config.flat_surface_in_block);
	ROS_INFO("New settings used.");

}
