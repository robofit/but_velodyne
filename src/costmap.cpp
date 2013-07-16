/*
 * costmap.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include "rt_road_detection/costmap.h"

using namespace rt_road_detection;
using namespace std;

TraversabilityCostmap::TraversabilityCostmap() {

	ros::param::param<float>("~map_res",map_res_,0.1);
	ros::param::param<float>("~map_size",map_size_,20.0);
	ros::param::param("~queue_length",queue_length_,5);

	ros::param::param<float>("~prob_max",prob_max_,0.95);
	ros::param::param<float>("~prob_min",prob_min_,0.05);

	ros::param::param("~prob_min",occ_grid_filter_, true);

	ros::param::param<string>("~map_frame",map_frame_,"odom");


	// TODO make occupancy grid "sliding" !!!!
	occ_grid_meta_.resolution = map_res_;
	occ_grid_meta_.origin.position.x = - map_size_/2.0; // TODO consider also initial position of the robot
	occ_grid_meta_.origin.position.y = - map_size_/2.0;
	occ_grid_meta_.origin.orientation.x = 0.0;
	occ_grid_meta_.origin.orientation.y = 0.0;
	occ_grid_meta_.origin.orientation.z = 0.0;
	occ_grid_meta_.origin.orientation.w = 1.0;

	occ_grid_meta_.width = (uint32_t)floor(map_size_/map_res_);
	occ_grid_meta_.height = (uint32_t)floor(map_size_/map_res_);

	uint32_t data_len = occ_grid_meta_.width * occ_grid_meta_.height;


	occ_grid_ = cv::Mat::ones(occ_grid_meta_.width, occ_grid_meta_.height,CV_32FC1);
	occ_grid_ *= 0.5;

	nh_ = ros::NodeHandle("~");

	occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map",10);

	it_.reset(new image_transport::ImageTransport(nh_));

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);


	disparity_sub_.subscribe(nh_,"/stereo/disparity",queue_length_);

	// TODO read list of detectors from YAML
	subscribe("/detectors/not_road/sample_grass");
	subscribe("/detectors/road/sample_asphalt");

	sub_l_info_.subscribe(nh_,"/stereo/left/camera_info",queue_length_);
	sub_r_info_.subscribe(nh_,"/stereo/right/camera_info",5);

	cam_info_approximate_sync_.reset( new CamInfoApproximateSync(CamInfoApproximatePolicy(queue_length_), sub_l_info_, sub_r_info_) );
	cam_info_approximate_sync_->registerCallback(boost::bind(&TraversabilityCostmap::camInfoCB, this, _1, _2));

	timer_ = nh_.createTimer(ros::Duration(1.0), &TraversabilityCostmap::timer, this);

	srv_get_map_ = nh_.advertiseService("get_map",&TraversabilityCostmap::getMap,this);
	srv_reset_map_ = nh_.advertiseService("reset_map",&TraversabilityCostmap::resetMap,this);


}

bool TraversabilityCostmap::getMap(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {

	nav_msgs::OccupancyGrid grid;

	createOccGridMsg(grid);

	res.map = grid;

	return true;

}

bool TraversabilityCostmap::resetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

	ROS_INFO("Reseting occupancy map.");

	occ_grid_ = cv::Mat::ones(occ_grid_meta_.width, occ_grid_meta_.height,CV_32FC1);
	occ_grid_ *= 0.5;

	return true;

}

void TraversabilityCostmap::subscribe(string topic) {

	ROS_INFO("Subscribing to %s topic.",ros::names::remap(topic).c_str());

	boost::shared_ptr<image_transport::SubscriberFilter> sub;
	boost::shared_ptr<ApproximateSync> sync;
	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	sub.reset(new image_transport::SubscriberFilter);
	sync.reset( new ApproximateSync(ApproximatePolicy(queue_length_), *sub, disparity_sub_) );

	sub->subscribe(*it_,ros::names::remap(topic),queue_length_,hints);

	sync->registerCallback(boost::bind(&TraversabilityCostmap::detectorCB, this, _1, _2, sub_list_.size()));

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

void TraversabilityCostmap::normalize(cv::Point3d& v)
{
  double len = norm(v);
  v.x /= len;
  v.y /= len;
  v.z /= len;
}

void TraversabilityCostmap::updateIntOccupancyGrid(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp) {

	if (!tfl_.waitForTransform(map_frame_,img->header.frame_id, img->header.stamp, ros::Duration(0.25))) {

		ROS_INFO_THROTTLE(1.0,"Waiting for TF...");
		return;

	} else ROS_INFO_ONCE("TF available.");

	const sensor_msgs::Image& dimage = disp->image;
	const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

	// TODO add some check if incoming message is in right encoding!!!
	const cv::Mat_<float> imat(img->height, img->width, (float*)&img->data[0], img->step);

	cv::Mat_<cv::Vec3f> points;

	// it's so easy to get 3D points ;)
	model_.projectDisparityImageTo3d(dmat, points, true);

	tf::StampedTransform tBaseToCam;

	try {

	      tfl_.lookupTransform(map_frame_, img->header.frame_id, img->header.stamp, tBaseToCam);

	    } catch(tf::TransformException& ex) {
	      ROS_WARN("TF exception:\n%s", ex.what());
	      return;
	    }

	 tf::Vector3 cameraOrigin = tBaseToCam.getOrigin();
	 //cameraOrigin.setY(cameraOrigin.getY()); // Was adding + 0.14 to it because it was using the wide_stereo

	 float cameraHeight = cameraOrigin.getZ();

	 //printf("Camera origin = %.2f %.2f %.2f\n", cameraOrigin.getX(), cameraOrigin.getY(), cameraOrigin.getZ());
	 //printf("Camera height = %.2fm\n", cameraHeight);

	 for (int32_t u = 0; u < points.rows; ++u) {
	 	for (int32_t v = 0; v < points.cols; ++v) {


	 	  // for points where we have depth information
	 	  // skip points which are too high
		  if (isValidPoint(points(u,v))) {

			  geometry_msgs::PointStamped pt;
			  pt.header.frame_id = img->header.frame_id;
			  pt.header.stamp = img->header.stamp;
			  pt.point.x = points(u,v).val[0];
			  pt.point.y = points(u,v).val[1];
			  pt.point.z = points(u,v).val[2];

			  // transform valid points from sensor frame to odom frame
			  try {

				  tfl_.transformPoint(map_frame_,pt,pt);

			  } catch (tf::TransformException& ex) {

				  ROS_ERROR("%s",ex.what());
				  continue;

			  }

			  // skip points which are too high
			  if (pt.point.z > 0.5) continue;

		  }

		  // following code taken from https://mediabox.grasp.upenn.edu/svn/penn-ros-pkgs/pr2_poop_scoop/trunk/perceive_poo/src/perceive_poo.cpp
		  // something similar: http://gt-ros-pkg.googlecode.com/svn/trunk/hrl/hrl_behaviors/hrl_move_floor_detect/src/move_floor_detect.cpp
		  // and http://ua-ros-pkg.googlecode.com/svn-history/r766/trunk/arrg/ua_vision/object_tracking/src/object_tracker.cpp
	 		cv::Point3d ray = model_.left().projectPixelTo3dRay(cv::Point(v,u));

	 		normalize(ray);

	 	 	tf::Vector3 cameraRay, baseRay;

			cameraRay.setX(ray.x);
			cameraRay.setY(ray.y);
			cameraRay.setZ(ray.z);

	        baseRay = tBaseToCam(cameraRay);

	        // We want the ray relative to the camera origin in the
	        // base coordinate frame
	        baseRay -= cameraOrigin;

	        // We are interested in the point where the ray hits the ground
	        if(baseRay.getZ() < 0)
	        {
	          float s = cameraHeight / -baseRay.getZ();
	          geometry_msgs::Point32 pt;
	          pt.x = cameraOrigin.getX() + baseRay.getX() * s;
	          pt.y = cameraOrigin.getY() + baseRay.getY() * s;
	          float dx = pt.x - cameraOrigin.getX();
	          float dy = pt.y - cameraOrigin.getY();
	          double dist = sqrt(dx*dx + dy*dy);

	          //printf("x: %f, y: %f, dist = %f\n", poo.x, poo.y, dist);

	          if (dist < 6) {

	        	  uint32_t x  = (uint32_t)floor(pt.x/map_res_ + ((map_size_/2.0)/map_res_));
	        	  uint32_t y  = (uint32_t)floor(pt.y/map_res_ + ((map_size_/2.0)/map_res_));

	        	  // check if indexes are ok, if not skip the point
				  if (x > occ_grid_meta_.width || y > occ_grid_meta_.width)  {

					  ROS_WARN_THROTTLE(0.5, "idx out of occ map!!!");
					  continue;

				  }

				  float val = imat(u,v);

				  // if val is zero, detector doesn't know anything about that area
				  //if (val == 0.0) continue;

				  //p_used++;

				  // update of occupancy grid
				  // TODO check if this is ok
				  // TODO update also some neighborhood pixels??????
				  occ_grid_(x,y) = (occ_grid_(x,y)*val) / ( (val*occ_grid_(x,y)) + (1.0-val)*(1-occ_grid_(x,y)));

				  if (occ_grid_(x,y) > prob_max_) occ_grid_(x,y) = prob_max_;
				  if (occ_grid_(x,y) < prob_min_) occ_grid_(x,y) = prob_min_;

	          }

	       }

	 	} // for
	 } // for



	/*if (road) cout << p_valid << "/" << p_used << " valid/used points (road)." << endl;
	else cout << p_valid << "/" << p_used << " valid/used points (not_road)." << endl;*/

}

void TraversabilityCostmap::createOccGridMsg(nav_msgs::OccupancyGrid& grid) {

	cv::Mat_<float> occ = occ_grid_.clone();

	if (occ_grid_filter_) {

		ROS_INFO_ONCE("Filtering map.");

		cv::GaussianBlur(occ_grid_,occ,cv::Size(3,3),0); // TODO check sigma value!!!!!

	} else ROS_INFO_ONCE("Not filtering map.");


	grid.header.frame_id = map_frame_;
	grid.header.stamp = ros::Time::now();

	grid.info = occ_grid_meta_;

	grid.data.clear();

	uint32_t data_len = occ_grid_meta_.width * occ_grid_meta_.height;

	grid.data.resize(data_len);

	for (int32_t u = 0; u < occ_grid_.rows; ++u) {
		  for (int32_t v = 0; v < occ_grid_.cols; ++v) {

			  if (occ(u,v) == 0.5) {

				  grid.data[(v*occ_grid_meta_.width) + u] = -1;

			  } else {

				  // TODO recalculate range <prob_min,prob_max> to <0,1> ??
				  float new_val = occ(u,v);

				  if (new_val == prob_max_) new_val = 1.0;
				  if (new_val == prob_min_) new_val = 0.0;

				  grid.data[(v*occ_grid_meta_.width) + u] = (int8_t)floor(new_val*100.0);

			  }

		  }

	}


}

void TraversabilityCostmap::timer(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("timer");

	nav_msgs::OccupancyGrid grid;

	createOccGridMsg(grid);

	if (occ_grid_pub_.getNumSubscribers() > 0)
			occ_grid_pub_.publish(grid);

}

void TraversabilityCostmap::detectorCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp, const int& idx) {

	ROS_INFO_ONCE("detector callback");

	if (!cam_info_received_) return;

	if (img->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {

			ROS_ERROR_THROTTLE(1.0, "Wrong detector image (%d) encoding! Float (TYPE_32FC1) in range <0,1> required!", idx);
			return;

		}


	updateIntOccupancyGrid(img,disp);

}


inline bool TraversabilityCostmap::isValidPoint(const cv::Vec3f& pt) {

	return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !isinf(pt[2]);

}
