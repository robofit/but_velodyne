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

	initialized_ = false;

	ros::param::param<float>("~map_res",map_res_,0.2);
	ros::param::param<float>("~map_size",map_size_,10.0);
	ros::param::param("~queue_length",queue_length_,10);

	ros::param::param<float>("~prob_max",prob_max_,0.95);
	ros::param::param<float>("~prob_min",prob_min_,0.05);

	ros::param::param<double>("~proj_dist",max_proj_dist_,3.0);

	ros::param::param("~grid_filter",occ_grid_filter_, true);

	ros::param::param("~use_disparity",use_disparity_, false); // TODO don't subscribe to disparity when this set to false
	ros::param::param("~use_scan",use_scan_, true);

	if (use_scan_ && use_disparity_) {

		ROS_WARN("Can't use scan and disparity at the same time. Will use just scan.");
		use_disparity_ = false;

	}

	ros::param::param<string>("~map_frame",map_frame_,"odom");

	ros::param::param<double>("~origin_update_th",origin_update_th_,0.5);

	ros::param::param<string>("~detectors_ns",detectors_ns_,"/detectors");


	ros::param::param<int>("~morph_filter_ks_size",morph_filter_ks_size_,5);
	ros::param::param<int>("~median_filter_ks_size",median_filter_ks_size_,5);
	ros::param::param<int>("~morph_filter_iter",morph_filter_iter_,2);

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
	
	occ_grid_img_pub_ = it_->advertise("occ_map_img", 1);

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	if (use_disparity_) disparity_sub_.subscribe(nh_,"/stereo/disparity",queue_length_);
	if (use_scan_) scan_sub_.subscribe(nh_,"/velodyne/scan",queue_length_);

	XmlRpc::XmlRpcValue pres;

	if (nh_.getParam("detectors",pres)) {

		ROS_ASSERT(pres.getType() == XmlRpc::XmlRpcValue::TypeArray);

		ROS_INFO("We will use %d detectors.",pres.size());

		for (int i=0; i < pres.size(); i++) {

			XmlRpc::XmlRpcValue xpr = pres[i];

			if (xpr.getType() != XmlRpc::XmlRpcValue::TypeStruct) {

			  ROS_ERROR("Wrong syntax in YAML config.");
			  continue;

			}

			// read the name
			if (!xpr.hasMember("name")) {

			  ROS_ERROR("Preset doesn't have 'name' property defined.");
			  continue;

			} else {

			  XmlRpc::XmlRpcValue name = xpr["name"];

			  std::string tmp = detectors_ns_ + "/" + static_cast<std::string>(name);

			  subscribe(tmp);


			}


		} // for

	} else {

		ROS_ERROR("Can't get list of detectors.");

	}


	sub_l_info_.subscribe(nh_,"/stereo/left/camera_info",queue_length_);
	sub_r_info_.subscribe(nh_,"/stereo/right/camera_info",queue_length_);

	cam_info_approximate_sync_.reset( new CamInfoApproximateSync(CamInfoApproximatePolicy(queue_length_), sub_l_info_, sub_r_info_) );
	cam_info_approximate_sync_->registerCallback(boost::bind(&TraversabilityCostmap::camInfoCB, this, _1, _2));

	timer_ = nh_.createTimer(ros::Duration(1.0), &TraversabilityCostmap::timer, this);

	srv_get_map_ = nh_.advertiseService("get_map",&TraversabilityCostmap::getMap,this);
	srv_reset_map_ = nh_.advertiseService("reset_map",&TraversabilityCostmap::resetMap,this);

	cache_buff_.reset(new tcache_buff(5));

}

double TraversabilityCostmap::round(double d)
{
  return floor(d + 0.5);
}


bool TraversabilityCostmap::updateMapOrigin() {


	geometry_msgs::Point oldp = map_origin_.pose.position;
	worldToMap(oldp);

	geometry_msgs::PoseStamped robot_pose;

	if (!robotPose(robot_pose)) return false;

	geometry_msgs::Point newp = robot_pose.pose.position;
	worldToMap(newp);

	int dx = (int)round(newp.x - oldp.x);
	int dy = (int)round(newp.y - oldp.y);

	if (!robotPose(map_origin_)) return false;

	occ_grid_meta_.origin.position.x = map_origin_.pose.position.x - map_size_/2.0;
	occ_grid_meta_.origin.position.y = map_origin_.pose.position.y - map_size_/2.0;

	toccmap new_map = toccmap::ones(occ_grid_.rows, occ_grid_.cols);
	new_map *= 0.5;

	// TODO is there more effective way how to do that????
	for (int32_t u = 0; u < new_map.rows; u++)
	for (int32_t v = 0; v < new_map.cols; v++) {

		if ((u-dx) >= 0 && (u-dx) < new_map.rows && (v-dy) >= 0 && (v-dy) < new_map.cols) {

			new_map(u-dx,v-dy) = occ_grid_(u,v);

		}

	} // for for


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

	if (tfl_.waitForTransform(map_frame_, p.header.frame_id, p.header.stamp, ros::Duration(0.5))) {

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

	nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid());
	cv_bridge::CvImagePtr img;

	createOccGridMsg(grid,img);

	res.map = *grid;

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

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);

	if (use_disparity_) {

		boost::shared_ptr<image_transport::SubscriberFilter> sub;
		boost::shared_ptr<ApproximateSync> sync;

		sub.reset(new image_transport::SubscriberFilter);
		sync.reset( new ApproximateSync(ApproximatePolicy(queue_length_), *sub, disparity_sub_) );

		sub->subscribe(*it_,ros::names::remap(topic),queue_length_,hints);

		sync->registerCallback(boost::bind(&TraversabilityCostmap::detectorCB, this, _1, _2, sub_list_.size()));

		sub_list_.push_back(sub);
		approximate_sync_list_.push_back(sync);

	} else if (use_scan_) {

		boost::shared_ptr<image_transport::SubscriberFilter> sub;
		boost::shared_ptr<ApproximateSyncScan> sync;

		sub.reset(new image_transport::SubscriberFilter);
		sync.reset( new ApproximateSyncScan(ApproximatePolicyScan(queue_length_), *sub, scan_sub_) );

		sub->subscribe(*it_,ros::names::remap(topic),queue_length_,hints);
		sync->registerCallback(boost::bind(&TraversabilityCostmap::detectorCBscan, this, _1, _2, sub_list_.size()));

		sub_list_.push_back(sub);
		approximate_sync_scan_list_.push_back(sync);


	} else {

		boost::shared_ptr<image_transport::Subscriber> sub;
		sub.reset(new image_transport::Subscriber);
		*sub = it_->subscribe(ros::names::remap(topic),queue_length_,boost::bind(&TraversabilityCostmap::detectorCBalt, this, _1, sub_list_wo_disp_.size())); // TODO add hints

		sub_list_wo_disp_.push_back(sub);

	}


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

void TraversabilityCostmap::updateIntOccupancyGrid(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp, const sensor_msgs::LaserScanConstPtr& scan) {

	if (!initialized_) return;

	if (!tfl_.waitForTransform(map_frame_,img->header.frame_id, img->header.stamp, ros::Duration(0.1))) {

		ROS_INFO_THROTTLE(1.0,"Waiting for transform from %s to %s",img->header.frame_id.c_str(),map_frame_.c_str());
		return;

	} else ROS_INFO_ONCE("TF available.");


	if (img->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {

		ROS_WARN_THROTTLE(1.0, "Wrong image encoding!");
		return;

	}

	const cv::Mat_<float> imat(img->height, img->width, (float*)&img->data[0], img->step);

	int cache_idx = -1;

	for (int i = 0; i < (int)cache_buff_->size(); i++) {

		if ((*cache_buff_)[i]->stamp == img->header.stamp) {

			cache_idx = i;
			break;

		}

	}

	// cache stuff which can be used more than once
	if (cache_idx == -1) {

		tcache_ptr cache;

		cache.reset(new tcache);

		/*cout << "cache used: " << cache_->used << endl;
		cache_->used = 0;*/
		cache->stamp = img->header.stamp;

		// get transform between map_frame and camera frame
		try {

		  tfl_.lookupTransform(map_frame_, img->header.frame_id, img->header.stamp, cache->tCamToBase);

		} catch(tf::TransformException& ex) {
		  ROS_WARN("TF exception:\n%s", ex.what());
		  return;
		}

		try {

		  tfl_.lookupTransform(img->header.frame_id, map_frame_,img->header.stamp, cache->tBaseToCam);

		} catch(tf::TransformException& ex) {
		  ROS_WARN("TF exception:\n%s", ex.what());
		  return;
		}

		// it's so easy to get 3D points ;)
		if (use_disparity_){

			const sensor_msgs::Image& dimage = disp->image;
			const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

			cv::Mat_<cv::Vec3f> points;

			model_.projectDisparityImageTo3d(dmat, points, true);

			ROS_ASSERT(points.rows == imat.rows);
			ROS_ASSERT(points.cols == imat.cols);

			cache->disp_skip.clear();
			cache->disp_skip.resize(imat.rows, std::vector<bool>(imat.cols,false));

			for (int32_t u = 0; u < imat.rows; u++)
			for (int32_t v = 0; v < imat.cols; v++) {

			  if (isValidPoint(points(u,v))) {

				  geometry_msgs::PointStamped pt;
				  pt.header.frame_id = img->header.frame_id;
				  pt.header.stamp = img->header.stamp;
				  pt.point.x = points(u,v).val[0];
				  pt.point.y = points(u,v).val[1];
				  pt.point.z = points(u,v).val[2];

				  try {

            // TODO use cached transform...
					  tfl_.transformPoint(map_frame_,pt,pt);

				  } catch (tf::TransformException& ex) {

					  ROS_ERROR("%s",ex.what());
					  return;

				  }

				  if (pt.point.z > 0.5) cache->disp_skip[u][v] = true;


			  } // if


			} // for for



		} // if disp


		tf::Vector3 cameraOrigin = cache->tCamToBase.getOrigin();
		float cameraHeight = cameraOrigin.getZ();

		vector<double> scan_dist;


		if (use_scan_) {

			scan_dist.resize(imat.cols,max_proj_dist_);

			sensor_msgs::PointCloud cloud;
			
			// TODO there is very small diff. in timestamps -> what about to modify scan timestamp slightly instead of waiting??? ;)
			if (!tfl_.waitForTransform(map_frame_,scan->header.frame_id, scan->header.stamp + ros::Duration(scan->scan_time + 0.01), ros::Duration(0.2))) {
			
			  ROS_INFO_THROTTLE(1.0,"Waiting for transform from %s to %s",scan->header.frame_id.c_str(),map_frame_.c_str());
			  return;
			
			}

			try {

				projector_.transformLaserScanToPointCloud(map_frame_,*scan,cloud,tfl_);


			} catch(tf::TransformException& ex) {
			  ROS_WARN("TF exception:\n%s", ex.what());
			  return;
			}

			//int pt_cnt = 0;

			for (int i=0; i < (int)cloud.points.size(); i++) {

				tf::Vector3 map_pt, cam_pt;

				map_pt.setX(cloud.points[i].x);
				map_pt.setY(cloud.points[i].y);
				map_pt.setZ(cloud.points[i].z - 0.8); // TODO read velodyne position using TF

				//cout << "odo " << map_pt.getX() << " " << map_pt.getY() << " " << map_pt.getZ() << endl;

				cam_pt = cache->tBaseToCam(map_pt);

				//cout << "cam " << cam_pt.getX() << " " << cam_pt.getY() << " " << cam_pt.getZ() << endl;

				// skip points behind camera
				if (cam_pt.getZ() < 0.0) continue;

				cv::Point3d pt;

				pt.x = cam_pt.getX();
				pt.y = cam_pt.getY();
				pt.z = cam_pt.getZ();

				cv::Point2d uv;

				uv = model_.left().project3dToPixel(pt);

				//cout << "img " << uv.x << " " << uv.y << endl;

				if ( (uv.x >= 0.0) && (uv.y >= 0.0) && ((int)round(uv.x) < imat.cols) && ((int)round(uv.y) < imat.rows) ) {

					//pt_cnt++;

					// make point relative to a camera
					map_pt -= cameraOrigin;

					float dx = map_pt.getX() - cameraOrigin.getX();
					float dy = map_pt.getY() - cameraOrigin.getY();
					double dist = sqrt(dx*dx + dy*dy);

					//cout << map_pt.getX() << " " <<  map_pt.getY() << " " <<  map_pt.getZ() << " col " << uv.x << " d: " << dist << endl;

					int idx = (int)round(uv.x);

					if (scan_dist[idx] > dist) scan_dist[idx] = dist;

					// update also some points in neighborhood
					if (idx-5 >= 0 && scan_dist[idx-5] > dist) scan_dist[idx-5] = dist;
					if (idx-4 >= 0 && scan_dist[idx-4] > dist) scan_dist[idx-4] = dist;
					if (idx-3 >= 0 && scan_dist[idx-3] > dist) scan_dist[idx-3] = dist;
					if (idx-2 >= 0 && scan_dist[idx-2] > dist) scan_dist[idx-2] = dist;
					if (idx-1 >= 0 && scan_dist[idx-1] > dist) scan_dist[idx-1] = dist;
					if (idx+1 < (int)scan_dist.size() && scan_dist[idx+1] > dist) scan_dist[idx+1] = dist;
					if (idx+2 < (int)scan_dist.size() && scan_dist[idx+2] > dist) scan_dist[idx+2] = dist;
					if (idx+3 < (int)scan_dist.size() && scan_dist[idx+3] > dist) scan_dist[idx+3] = dist;
					if (idx+4 < (int)scan_dist.size() && scan_dist[idx+4] > dist) scan_dist[idx+4] = dist;
					if (idx+5 < (int)scan_dist.size() && scan_dist[idx+5] > dist) scan_dist[idx+5] = dist;


				}

		} // for cloud

		//cout << "points in fov: " << pt_cnt << endl;

		/*int unused_dists = 0;

		for (int i=0; i < (int)scan_dist.size(); i++) {

			if (scan_dist[i] == max_proj_dist_) unused_dists++;

		}

		cout << "unused cols: " << unused_dists << endl;*/

		} // if use scan

		cache->pts.clear();
		cache->pts.resize(imat.rows, std::vector<geometry_msgs::Point>(imat.cols));

		//int lim_pts = 0;

		// cache rays
		for (int32_t u = 0; u < imat.rows; u++)
		for (int32_t v = 0; v < imat.cols; v++) {

			  // following code taken from https://mediabox.grasp.upenn.edu/svn/penn-ros-pkgs/pr2_poop_scoop/trunk/perceive_poo/src/perceive_poo.cpp
			  // something similar: http://gt-ros-pkg.googlecode.com/svn/trunk/hrl/hrl_behaviors/hrl_move_floor_detect/src/move_floor_detect.cpp
			  // and http://ua-ros-pkg.googlecode.com/svn-history/r766/trunk/arrg/ua_vision/object_tracking/src/object_tracker.cpp

			 // TODO check if it really should be like this (v,u)
			cv::Point3d ray = model_.left().projectPixelTo3dRay(cv::Point(v,u));

			normalize(ray);

			tf::Vector3 cameraRay, baseRay;

			cameraRay.setX(ray.x);
			cameraRay.setY(ray.y);
			cameraRay.setZ(ray.z);

			baseRay = cache->tCamToBase(cameraRay);


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

			  if (dist < max_proj_dist_) {

				  if (use_scan_ && (dist > scan_dist[v]) ) {

					  //lim_pts++;

					  //cout << "ll " << dist << " " << scan_dist[v] << endl;
					  cache->pts[u][v].x = -1.0;

				  } else {

					  worldToMap(pt);

					  cache->pts[u][v] = pt;

				  }

			  } else {

				  cache->pts[u][v].x = -1.0;

			  }

			} // if baseRay


		} // for for

		//cout << "lim pts " << lim_pts << endl;

		cache_buff_->push_back(cache);
		cache_idx = cache_buff_->size() - 1;

	} else {

		(*cache_buff_)[cache_idx]->used++;

	}


	 for (int32_t u = 0; u < imat.rows; u++)
	 for (int32_t v = 0; v < imat.cols; v++) {


		  // skip certain points where we have depth information
		  if (use_disparity_ && (*cache_buff_)[cache_idx]->disp_skip[u][v]) continue;


		  if ((*cache_buff_)[cache_idx]->pts[u][v].x != -1.0) {

			  uint32_t x  = (uint32_t)(*cache_buff_)[cache_idx]->pts[u][v].x;
			  uint32_t y  = (uint32_t)(*cache_buff_)[cache_idx]->pts[u][v].y;

			  // TODO move this to caching part
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
			  if (val != 0.5) occ_grid_(x,y) = (occ_grid_(x,y)*val) / ( (val*occ_grid_(x,y)) + (1.0-val)*(1-occ_grid_(x,y)));


		  } // if cache pts


	 } // for for

}

void TraversabilityCostmap::createOccGridMsg(nav_msgs::OccupancyGridPtr grid, cv_bridge::CvImagePtr img) {

	toccmap occ;

	if (occ_grid_filter_) {

		ROS_INFO_ONCE("Filtering map.");

		occ = occ_grid_.clone();

		int erosion_size = morph_filter_ks_size_;
		int dilation_size = morph_filter_ks_size_;

		cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS,
		                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
		                                       cv::Point( dilation_size, dilation_size ) );

		for (int i=0; i < morph_filter_iter_; i++) {

			cv::erode( occ, occ, element );
			cv::dilate( occ, occ, element );

		}

		if (median_filter_ks_size_%2 == 1) cv::medianBlur(occ, occ, median_filter_ks_size_);


	} else {

		ROS_INFO_ONCE("Not filtering map.");
		occ = occ_grid_;

	}


	grid->header.frame_id = map_frame_;
	grid->header.stamp = ros::Time::now();

	grid->info = occ_grid_meta_;

	grid->data.clear();

	uint32_t data_len = occ_grid_.rows * occ_grid_.cols;

	grid->data.resize(data_len, -1);

	for (int32_t u = 0; u < occ_grid_.rows; u++) {
		  for (int32_t v = 0; v < occ_grid_.cols; v++) {


			if (occ(u,v) != 0.5) { // keep -1 for 0.5

				  // TODO do some thresholding?
				  float new_val = occ(u,v);

				  grid->data[(v*occ_grid_.rows) + u] = (int8_t)round(new_val*100.0);

			  }

		  }

	}
	
	if (img != NULL) {
	
	  img->header = grid->header;
	  img->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	  img->image = occ_grid_;
	
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

			return;

		}

	}


	geometry_msgs::PoseStamped p;
	if (!robotPose(p)) return;

	double dist = sqrt(pow(p.pose.position.x - map_origin_.pose.position.x,2) + pow(p.pose.position.y - map_origin_.pose.position.y,2));

	if (dist > origin_update_th_) { // TODO make configurable

		ROS_INFO("Robot traveled %f meters. Updating map origin.",dist);
		updateMapOrigin();

	}

	if (occ_grid_pub_.getNumSubscribers() > 0 || occ_grid_img_pub_.getNumSubscribers() > 0) {

		nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
		cv_bridge::CvImagePtr img_msg(new cv_bridge::CvImage);
		
		createOccGridMsg(grid,img_msg);

		occ_grid_pub_.publish(grid);
		occ_grid_img_pub_.publish(img_msg->toImageMsg());

	}


}

void TraversabilityCostmap::detectorCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp, const int& idx) {

	ROS_INFO_ONCE("detector callback");

	if (!cam_info_received_) return;

	if (img->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {

			ROS_ERROR_THROTTLE(1.0, "Wrong detector image (%d) encoding! Float (TYPE_32FC1) in range <0,1> required!", idx);
			return;

		}

	const sensor_msgs::LaserScanConstPtr scan;

	updateIntOccupancyGrid(img,disp,scan);

}

void TraversabilityCostmap::detectorCBalt(const sensor_msgs::ImageConstPtr& img, const int& idx) {

	ROS_INFO_ONCE("detector callback (without disparity)");

	if (!cam_info_received_) return;

	if (img->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {

			ROS_ERROR_THROTTLE(1.0, "Wrong detector image (%d) encoding! Float (TYPE_32FC1) in range <0,1> required!", idx);
			return;

		}

	const stereo_msgs::DisparityImageConstPtr ptr;
	const sensor_msgs::LaserScanConstPtr scan;

	updateIntOccupancyGrid(img,ptr,scan);

}

void TraversabilityCostmap::detectorCBscan(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::LaserScanConstPtr& scan, const int& idx) {

	ROS_INFO_ONCE("detector callback (with scan)");

	if (!cam_info_received_) return;

	if (img->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {

			ROS_ERROR_THROTTLE(1.0, "Wrong detector image (%d) encoding! Float (TYPE_32FC1) in range <0,1> required!", idx);
			return;

		}

	const stereo_msgs::DisparityImageConstPtr ptr;

	updateIntOccupancyGrid(img,ptr,scan);

}


inline bool TraversabilityCostmap::isValidPoint(const cv::Vec3f& pt) {

	return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !isinf(pt[2]);

}
