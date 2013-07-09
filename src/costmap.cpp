/*
 * costmap.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include "rt_road_detection/costmap.h"

using namespace rt_road_detection;


TraversabilityCostmap::TraversabilityCostmap() {

	ros::param::param<float>("~map_res",map_res_,0.2);
	ros::param::param<float>("~map_size",map_size_,20.0);
	ros::param::param("~queue_length",queue_length_,5);

	occ_grid_update_.reset(new nav_msgs::OccupancyGrid());

	//sensor_frame_ = "left_cam_link";
	sensor_frame_ = "";

	// TODO make it configurable!!!!
	// are we going to make this internal costmap in base_link or in odom????
	occ_grid_update_->header.frame_id = "base_link";
	occ_grid_update_->info.resolution = map_res_;
	//occ_grid_update_->info.origin. // TODO update this using TF/odom (and then do periodical updates) ????
	occ_grid_update_->info.origin.position.x = - map_size_/2.0;
	occ_grid_update_->info.origin.position.y = - map_size_/2.0;
	occ_grid_update_->info.origin.orientation.x = 0.0;
	occ_grid_update_->info.origin.orientation.y = 0.0;
	occ_grid_update_->info.origin.orientation.z = 0.0;
	occ_grid_update_->info.origin.orientation.w = 1.0;

	occ_grid_update_->info.width = (uint32_t)floor(map_size_/map_res_);
	occ_grid_update_->info.height = (uint32_t)floor(map_size_/map_res_);

	uint32_t data_len = occ_grid_update_->info.width * occ_grid_update_->info.height;

	occ_grid_update_->data.resize(data_len);

	for(uint32_t i=0; i < data_len; i++) {

		occ_grid_update_->data[i] = -1; // initial value (unknown)...

	}

	occ_grid_ = cv::Mat::ones(occ_grid_update_->info.width, occ_grid_update_->info.height,CV_32FC1);
	occ_grid_ *= 0.5;

	nh_ = ros::NodeHandle("~");

	occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map",10);

	it_.reset(new image_transport::ImageTransport(nh_));

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);


	disparity_sub_.subscribe(nh_,"/stereo/disparity",queue_length_);

	// TODO read list of detectors from YAML???
	subscribe("/detectors/not_road/sample_grass",false);
	subscribe("/detectors/road/sample_asphalt",true);

	sub_l_info_.subscribe(nh_,"/stereo/left/camera_info",queue_length_);
	sub_r_info_.subscribe(nh_,"/stereo/right/camera_info",5);

	cam_info_approximate_sync_.reset( new CamInfoApproximateSync(CamInfoApproximatePolicy(queue_length_), sub_l_info_, sub_r_info_) );
	cam_info_approximate_sync_->registerCallback(boost::bind(&TraversabilityCostmap::camInfoCB, this, _1, _2));

	timer_ = nh_.createTimer(ros::Duration(0.1), &TraversabilityCostmap::timer, this);

}

void TraversabilityCostmap::subscribe(std::string topic, bool road) {

	if (road) ROS_INFO("Subscribing to %s topic (road).",ros::names::remap(topic).c_str());
	else ROS_INFO("Subscribing to %s topic (not_road).",ros::names::remap(topic).c_str());

	boost::shared_ptr<image_transport::SubscriberFilter> sub;
	boost::shared_ptr<ApproximateSync> sync;
	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	sub.reset(new image_transport::SubscriberFilter);
	sync.reset( new ApproximateSync(ApproximatePolicy(queue_length_), *sub, disparity_sub_) );

	sub->subscribe(*it_,ros::names::remap(topic),queue_length_,hints);

	if (road) sync->registerCallback(boost::bind(&TraversabilityCostmap::roadCB, this, _1, _2));
	else sync->registerCallback(boost::bind(&TraversabilityCostmap::notRoadCB, this, _1, _2));

	sub_list_.push_back(sub);
	approximate_sync_list_.push_back(sync);


}

TraversabilityCostmap::~TraversabilityCostmap() {

	sub_list_.clear();
	approximate_sync_list_.clear();

}


void TraversabilityCostmap::camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info_left, const sensor_msgs::CameraInfoConstPtr& cam_info_right) {

	ROS_INFO_ONCE("camInfoCB");

	// update camera model
	model_.fromCameraInfo(cam_info_left,cam_info_right);

	cam_info_received_ = true;

}

void TraversabilityCostmap::updateIntOccupancyGrid(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp, bool road) {

	std::string target_frame = img->header.frame_id;

	if (sensor_frame_ != "") target_frame = sensor_frame_;

	if (!tfl_.canTransform("base_link", target_frame, img->header.stamp)) {

		ROS_INFO_THROTTLE(1.0,"Waiting for TF...");
		return;

	} else ROS_INFO_ONCE("TF available.");

	const sensor_msgs::Image& dimage = disp->image;
	const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

	const cv::Mat_<uint8_t> imat(img->height, img->width, (uint8_t*)&img->data[0], img->step);

	cv::Mat_<cv::Vec3f> points;

	model_.projectDisparityImageTo3d(dmat, points, true);

	uint32_t p_valid = 0;
	uint32_t p_used = 0;

	int i = 0;
	for (int32_t u = 0; u < points.rows; ++u) {
	  for (int32_t v = 0; v < points.cols; ++v, ++i) {

		  if (!isValidPoint(points(u,v))) {


			  continue;

		  }

		  p_valid++;

		  geometry_msgs::PointStamped pt;
		  pt.header.frame_id = target_frame;
		  pt.header.stamp = img->header.stamp;
		  pt.point.x = points(u,v).val[0];
		  pt.point.y = points(u,v).val[1];
		  pt.point.z = points(u,v).val[2];

		  // transform point from sensor frame to odom frame
		  try {

			  tfl_.transformPoint("base_link",pt,pt);

		  } catch (tf::TransformException& ex) {

			  ROS_ERROR("%s",ex.what());
			  continue;

		  }

		  // skip points which are too low or too high
		  if (pt.point.z < -0.2 || pt.point.z > 0.5) continue; // TODO make this configurable / use some plane detection???

		  // skip points which are outside of our internal occupancy map
		  if (pt.point.x < 0.1 || (pt.point.x > map_size_/2.0) ) continue;
		  if ( (pt.point.y < (-map_size_/2.0)) || (pt.point.y > map_size_/2.0) ) continue; // robot is in the center of map

		  ROS_INFO_ONCE("Some points... ;)");

		  //std::cout << pt.point.x << " " << pt.point.y << " " << pt.point.z << std::endl;

		  uint32_t x  = (uint32_t)floor(pt.point.x/map_res_ + ((map_size_/2.0)/map_res_));
		  uint32_t y  = (uint32_t)floor(pt.point.y/map_res_ + ((map_size_/2.0)/map_res_));

		  if (x > occ_grid_update_->info.width || y > occ_grid_update_->info.width) std::cout << "idx out of occ map!!!" << std::endl;

		  float val = (float)imat(u,v);

		 val /= 255.0; // normalize

		  // if val is zero, detector doesn't know anything about that area
		  if (val == 0.0) continue;

		  p_used++;

		  if (!road) val = 1.0 - val; // 1.0 means occupied (det. produce 255 for road), normalize to range 0-100

		  //if (road) std::cout << x << " " << y << " " << val << "(road)" << std::endl;
		  //else std::cout << x << " " << y << " " << val << "(not_road)" << std::endl;

		  // TODO update occupancy grid
		  //  !!!!! this is only for testing with one detector !!!!!!! updating in smarter way (probabilistic?) needed...
		  occ_grid_(x,y) = val;


		} // for cols

	} // for rows

	if (road) std::cout << p_valid << "/" << p_used << " valid/used points (road)." << std::endl;
	else std::cout << p_valid << "/" << p_used << " valid/used points (not_road)." << std::endl;

}

void TraversabilityCostmap::timer(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("timer");

	occ_grid_update_->data.clear();

	uint32_t data_len = occ_grid_update_->info.width * occ_grid_update_->info.height;

	occ_grid_update_->data.resize(data_len);

	for (int32_t u = 0; u < occ_grid_.rows; ++u) {
		  for (int32_t v = 0; v < occ_grid_.cols; ++v) {

			  if (occ_grid_(u,v) == 0.5) {

				  occ_grid_update_->data[(v*occ_grid_update_->info.width) + u] = -1;

			  } else {

				  occ_grid_update_->data[(v*occ_grid_update_->info.width) + u] = (int8_t)floor(occ_grid_(u,v)*100.0);

			  }

		  }

	}

	// reset internal occupancy grid
	//occ_grid_ = cv::Mat::ones(occ_grid_update_->info.width, occ_grid_update_->info.height,CV_32FC1);

	if (occ_grid_pub_.getNumSubscribers() > 0)
			occ_grid_pub_.publish(occ_grid_update_);

}

void TraversabilityCostmap::notRoadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp) {

	ROS_INFO_ONCE("not_road CB");

	if (!cam_info_received_) return;

	updateIntOccupancyGrid(img,disp,false);

}

void TraversabilityCostmap::roadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp) {

	ROS_INFO_ONCE("road CB");

	if (!cam_info_received_) return;

	updateIntOccupancyGrid(img,disp,true);

}

inline bool TraversabilityCostmap::isValidPoint(const cv::Vec3f& pt) {

	return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);

}
