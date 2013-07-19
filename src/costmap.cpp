/*
 * costmap.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include "rt_road_detection/costmap.h"

using namespace rt_road_detection;
using namespace std;

TraversabilityCostmap::TraversabilityCostmap(ros::NodeHandle priv_nh) {

	ros::param::param<float>("~map_res",map_res_,0.2);
	ros::param::param<float>("~map_size",map_size_,10.0);
	ros::param::param("~queue_length",queue_length_,10);

	ros::param::param<float>("~prob_max",prob_max_,0.95);
	ros::param::param<float>("~prob_min",prob_min_,0.05);

	ros::param::param<double>("~proj_dist",max_proj_dist_,3.0);

	ros::param::param("~grid_filter",occ_grid_filter_, true);

	ros::param::param("~use_disparity",use_disparity_, true); // TODO don't subscribe to disparity when this set to false

	ros::param::param<string>("~map_frame",map_frame_,"odom");


	occ_grid_meta_.resolution = map_res_;
	occ_grid_meta_.origin.position.x = - map_size_/2.0; // initial position of a robot is filled later
	occ_grid_meta_.origin.position.y = - map_size_/2.0;
	occ_grid_meta_.origin.orientation.x = 0.0;
	occ_grid_meta_.origin.orientation.y = 0.0;
	occ_grid_meta_.origin.orientation.z = 0.0;
	occ_grid_meta_.origin.orientation.w = 1.0;

	occ_grid_meta_.width = (uint32_t)floor(map_size_/map_res_);
	occ_grid_meta_.height = (uint32_t)floor(map_size_/map_res_);

	uint32_t data_len = occ_grid_meta_.width * occ_grid_meta_.height;


	occ_grid_ = toccmap::ones(occ_grid_meta_.width, occ_grid_meta_.height);
	occ_grid_ *= 0.5;

	nh_ = priv_nh;

	occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map",5);

	it_.reset(new image_transport::ImageTransport(nh_));

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	disparity_sub_.subscribe(nh_,"/stereo/disparity",queue_length_);

	// TODO read list of detectors from YAML
	subscribe("/detectors/sample_grass");
	subscribe("/detectors/sample_asphalt");

	sub_l_info_.subscribe(nh_,"/stereo/left/camera_info",queue_length_);
	sub_r_info_.subscribe(nh_,"/stereo/right/camera_info",queue_length_);

	cam_info_approximate_sync_.reset( new CamInfoApproximateSync(CamInfoApproximatePolicy(queue_length_), sub_l_info_, sub_r_info_) );
	cam_info_approximate_sync_->registerCallback(boost::bind(&TraversabilityCostmap::camInfoCB, this, _1, _2));

	timer_ = nh_.createTimer(ros::Duration(1.0), &TraversabilityCostmap::timer, this);

	srv_get_map_ = nh_.advertiseService("get_map",&TraversabilityCostmap::getMap,this);
	srv_reset_map_ = nh_.advertiseService("reset_map",&TraversabilityCostmap::resetMap,this);

	initialized_ = false;


}

double TraversabilityCostmap::round(double d)
{
  return floor(d + 0.5);
}


bool TraversabilityCostmap::updateMapOrigin() {


	geometry_msgs::Point oldp = map_origin_.pose.position;
	worldToMap(oldp);

	geometry_msgs::PoseStamped robot_pose;

	robotPose(robot_pose);

	geometry_msgs::Point newp = robot_pose.pose.position;
	worldToMap(newp);

	int dx = (int)round(newp.x - oldp.x);
	int dy = (int)round(newp.y - oldp.y);

	robotPose(map_origin_);

	occ_grid_meta_.origin.position.x = map_origin_.pose.position.x - map_size_/2.0;
	occ_grid_meta_.origin.position.y = map_origin_.pose.position.y - map_size_/2.0;

	toccmap new_map = toccmap::ones(occ_grid_.rows, occ_grid_.cols);
	new_map *= 0.5;

	// TODO is there more effective way how to do that????
	for (int32_t u = 0; u < new_map.rows; u++) {
	for (int32_t v = 0; v < new_map.cols; v++) {

		if ((u-dx) >= 0 && (u-dx) < new_map.rows && (v-dy) >= 0 && (v-dy) < new_map.cols) {

			new_map(u-dx,v-dy) = occ_grid_(u,v);

		}

	}
	}


	occ_grid_ = new_map;

	return true;

}

bool TraversabilityCostmap::robotPose(geometry_msgs::PoseStamped& pose) {

	geometry_msgs::PoseStamped p;

	p.header.frame_id = "base_link"; // just for initialization
	p.header.stamp = ros::Time::now();
	p.pose.position.x = 0;
	p.pose.position.y = 0;
	p.pose.position.z = 0;
	p.pose.orientation.x = 0;
	p.pose.orientation.y = 0;
	p.pose.orientation.z = 0;
	p.pose.orientation.w = 1;

	if (tfl_.waitForTransform(map_frame_, p.header.frame_id, p.header.stamp, ros::Duration(3.0))) {

		bool tr = false;

		try {

		  tfl_.transformPose(map_frame_,p, pose);
		  tr = true;

		} catch(tf::TransformException& ex) {
		  ROS_WARN("TF exception:\n%s", ex.what());

		}

		return tr;


	} else {

		ROS_ERROR("Transform not available!");
		return false;

	}

}

bool TraversabilityCostmap::getMap(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {

	nav_msgs::OccupancyGrid grid;

	createOccGridMsg(grid);

	res.map = grid;

	return true;

}

bool TraversabilityCostmap::resetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

	ROS_INFO("Reseting occupancy map.");

	occ_grid_ = toccmap::ones(occ_grid_meta_.width, occ_grid_meta_.height);
	occ_grid_ *= 0.5;

	updateMapOrigin();

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

void TraversabilityCostmap::worldToMap(geometry_msgs::Point& p) {

	p.x = round((p.x - map_origin_.pose.position.x)/map_res_ + ((map_size_/2.0)/map_res_));
	p.y = round((p.y - map_origin_.pose.position.y)/map_res_ + ((map_size_/2.0)/map_res_));

	if (p.x < 0) p.x = 0;
	if (p.y < 0) p.y = 0;

	if (p.x > (double)occ_grid_.rows) p.x = (double)occ_grid_.rows;
	if (p.y > (double)occ_grid_.cols) p.y = (double)occ_grid_.cols;


}

void TraversabilityCostmap::updateIntOccupancyGrid(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp) {

	if (!initialized_) return;

	if (!tfl_.waitForTransform(map_frame_,img->header.frame_id, img->header.stamp, ros::Duration(0.1))) {

		ROS_INFO_THROTTLE(1.0,"Waiting for TF...");
		return;

	} else ROS_INFO_ONCE("TF available.");


	if (img->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {

		ROS_WARN_THROTTLE(1.0, "Wrong image encoding!");
		return;

	}

	const sensor_msgs::Image& dimage = disp->image;
	const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
	const cv::Mat_<float> imat(img->height, img->width, (float*)&img->data[0], img->step);

	cv::Mat_<cv::Vec3f> points;

	// it's so easy to get 3D points ;)
	if (use_disparity_) model_.projectDisparityImageTo3d(dmat, points, true);

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

	 ROS_ASSERT(points.rows == imat.rows);
	 ROS_ASSERT(points.cols == imat.cols);

	 for (int32_t u = 0; u < points.rows; u++) {
	 	for (int32_t v = 0; v < points.cols; v++) {


	 	  // for points where we have depth information
	 	  // skip points which are too high
		  if (use_disparity_ && isValidPoint(points(u,v))) {

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
	          geometry_msgs::Point pt;
	          pt.x = cameraOrigin.getX() + baseRay.getX() * s;
	          pt.y = cameraOrigin.getY() + baseRay.getY() * s;
	          float dx = pt.x - cameraOrigin.getX();
	          float dy = pt.y - cameraOrigin.getY();
	          double dist = sqrt(dx*dx + dy*dy);

	          //printf("x: %f, y: %f, dist = %f\n", poo.x, poo.y, dist);

	          if (dist < max_proj_dist_) {

	        	  worldToMap(pt);

	        	  uint32_t x  = (uint32_t)pt.x;
	        	  uint32_t y  = (uint32_t)pt.y;

	        	  // check if indexes are ok, if not skip the point
				  if (x > occ_grid_meta_.width || y > occ_grid_meta_.width)  {

					  ROS_WARN_THROTTLE(1.0, "idx out of occ map!!!");
					  continue;

				  }

				  float val = imat(u,v);

				  // limit probability values
				  if (val > prob_max_) val = prob_max_;
				  if (val < prob_min_) val = prob_min_;

				  // update of occupancy grid
				  // TODO should we limit also value of occ_grid_(x,y)????
				  occ_grid_(x,y) = (occ_grid_(x,y)*val) / ( (val*occ_grid_(x,y)) + (1.0-val)*(1-occ_grid_(x,y)));

	          }

	       }

	 	} // for
	 } // for

}

void TraversabilityCostmap::createOccGridMsg(nav_msgs::OccupancyGrid& grid) {

	toccmap occ;

	if (occ_grid_filter_) {

		ROS_INFO_ONCE("Filtering map.");

		occ = occ_grid_.clone();

		//cv::GaussianBlur(occ,occ,cv::Size(5,5),0); // TODO check sigma value?

		int erosion_size = 5;
		int dilation_size = 5;

		cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS,
		                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
		                                       cv::Point( dilation_size, dilation_size ) );

		cv::erode( occ, occ, element ); // TODO make it configurable (size / iterations)
		cv::dilate( occ, occ, element );

		cv::erode( occ, occ, element );
		cv::dilate( occ, occ, element );

		cv::medianBlur(occ, occ, 3);


	} else {

		ROS_INFO_ONCE("Not filtering map.");
		occ = occ_grid_;

	}


	grid.header.frame_id = map_frame_;
	grid.header.stamp = ros::Time::now();

	grid.info = occ_grid_meta_;

	grid.data.clear();

	uint32_t data_len = occ_grid_.rows * occ_grid_.cols;

	grid.data.resize(data_len, -1);

	for (int32_t u = 0; u < occ_grid_.rows; u++) {
		  for (int32_t v = 0; v < occ_grid_.cols; v++) {


			if (occ(u,v) != 0.5) { // keep -1 for 0.5

				  // TODO do some thresholding?
				  float new_val = occ(u,v);

				  grid.data[(v*occ_grid_.rows) + u] = (int8_t)round(new_val*100.0);

			  }

		  }

	}


}

void TraversabilityCostmap::timer(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("timer");

	if (!initialized_) {

		if (robotPose(map_origin_)) {

			// initialize the map origin using current position of the robot
			occ_grid_meta_.origin.position.x = map_origin_.pose.position.x - map_size_/2.0;
			occ_grid_meta_.origin.position.y = map_origin_.pose.position.y - map_size_/2.0;

			ROS_INFO("Initialized.");

			initialized_ = true;

		} else {

			ROS_INFO("Waiting for TF...");
			return;

		}

	}


	geometry_msgs::PoseStamped p;
	robotPose(p);

	double dist = sqrt(pow(p.pose.position.x - map_origin_.pose.position.x,2) + pow(p.pose.position.y - map_origin_.pose.position.y,2));

	if (dist > map_size_*0.05) { // TODO make configurable

		ROS_INFO("Robot traveled %f meters. Updating map origin.",dist);
		updateMapOrigin();

	}

	if (occ_grid_pub_.getNumSubscribers() > 0) {

		nav_msgs::OccupancyGrid grid;
		createOccGridMsg(grid);

		occ_grid_pub_.publish(grid);

	}


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
