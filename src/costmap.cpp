/*
 * costmap.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include "rt_road_detection/costmap.h"

using namespace rt_road_detection;


TraversabilityCostmap::TraversabilityCostmap() {

	map_res_ = 0.01; // 1px = 10cm
	map_size_ = 20.0;

	occ_grid_update_.reset(new nav_msgs::OccupancyGrid());

	// TODO make it configurable!!!!
	// are we going to make this internal costmap in base_link or in odom????
	occ_grid_update_->header.frame_id = "base_link";
	occ_grid_update_->info.resolution = map_res_;
	//occ_grid_update_->info.origin. // TODO update this using TF/odom (and then do periodical updates) ????
	/*occ_grid_update_->info.origin.position.x = -(20.0/map_res_)/2.0;
	occ_grid_update_->info.origin.position.y = -(20.0/map_res_)/2.0;*/

	occ_grid_update_->info.width = (uint32_t)floor(map_size_/map_res_);
	occ_grid_update_->info.height = (uint32_t)floor(map_size_/map_res_);

	uint32_t data_len = occ_grid_update_->info.width * occ_grid_update_->info.height;

	occ_grid_update_->data.resize(data_len);

	for(uint32_t i=0; i < data_len; i++) {

		occ_grid_update_->data[i] = -1; // initial value (unknown)...

	}

	nh_ = ros::NodeHandle("~");

	occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map",10);

	it_.reset(new image_transport::ImageTransport(nh_));

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	ROS_INFO("Subscribing to %s topic.",ros::names::remap("not_road_in").c_str());

	not_road_sub_.subscribe(*it_,ros::names::remap("not_road_in"),1,hints);
	disparity_sub_.subscribe(nh_,"/stereo/disparity",5); // TODO make queue length configurable!

	approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(5), not_road_sub_, disparity_sub_) );
	approximate_sync_->registerCallback(boost::bind(&TraversabilityCostmap::notRoadCB, this, _1, _2));

	sub_l_info_.subscribe(nh_,"/stereo/left/camera_info",5);
	sub_r_info_.subscribe(nh_,"/stereo/right/camera_info",5);

	cam_info_approximate_sync_.reset( new CamInfoApproximateSync(CamInfoApproximatePolicy(5), sub_l_info_, sub_r_info_) );
	cam_info_approximate_sync_->registerCallback(boost::bind(&TraversabilityCostmap::camInfoCB, this, _1, _2));


	//boost::shared_ptr<image_transport::Subscriber> sub;
	//sub.reset(new image_transport::Subscriber("not_road_in", 1, &TraversabilityCostmap::notRoadCB,this));
	//not_road_sub_.push_back(sub);

}

TraversabilityCostmap::~TraversabilityCostmap() {


}


void TraversabilityCostmap::camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info_left, const sensor_msgs::CameraInfoConstPtr& cam_info_right) {

	ROS_INFO_ONCE("camInfoCB");

	// update camera model
	model_.fromCameraInfo(cam_info_left,cam_info_right);

	cam_info_received_ = true;

}


void TraversabilityCostmap::notRoadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp) {

	ROS_INFO_ONCE("not_road CB");

	if (!cam_info_received_) return;

	if (!tfl_.canTransform("base_link",img->header.frame_id,img->header.stamp)) {

		ROS_INFO_THROTTLE(1.0,"Waiting for TF...");
		return;

	} else ROS_INFO_ONCE("TF available.");

	const sensor_msgs::Image& dimage = disp->image;
	const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

	//const sensor_msgs::Image& image = img;
	const cv::Mat_<uint8_t> imat(img->height, img->width, (uint8_t*)&img->data[0], img->step);

	cv::Mat_<cv::Vec3f> points;

	// TODO transform points to odom frame????????

	model_.projectDisparityImageTo3d(dmat, points, true);

	float map_lim = floor(map_size_/map_res_);

	uint32_t p_not_valid = 0;

	int i = 0;
	for (int32_t u = 0; u < points.rows; ++u) {
	  for (int32_t v = 0; v < points.cols; ++v, ++i) {

		  if (!isValidPoint(points(u,v))) {

			  p_not_valid++;
			  continue;

		  }

		  geometry_msgs::PointStamped pt;
		  pt.header.frame_id = img->header.frame_id;
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

		  uint32_t x  = (uint32_t)floor(pt.point.x/map_res_);
		  uint32_t y  = (uint32_t)floor(pt.point.y/map_res_ + (map_size_/2.0)/map_res_);

		  //std::cout << x << " " << y << std::endl;

		  float val = (float)imat(x,y);

		  val = 100.0 - (val/255.0)*100.0; // normalize to range 0-100

		  // TODO update occupancy grid
		  //  !!!!! this is only for testing with one detector !!!!!!! updating in smarter way (probabilistic?) needed...
		  // TODO use different internal representation? cv::Mat_<float>???????
		  occ_grid_update_->data[(x*occ_grid_update_->info.width) + y] = (int8_t)val;


	    } // for cols
	} // for rows

	if (occ_grid_pub_.getNumSubscribers() > 0)
		occ_grid_pub_.publish(occ_grid_update_);

}

void TraversabilityCostmap::roadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp) {

	if (!cam_info_received_) return;


}

inline bool TraversabilityCostmap::isValidPoint(const cv::Vec3f& pt) {

	return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);

}
