/*
 * costmap.h
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */

#ifndef COSTMAP_H_
#define COSTMAP_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
//#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_listener.h>

namespace rt_road_detection {

	typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, stereo_msgs::DisparityImage> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

	typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> CamInfoApproximatePolicy;
	typedef message_filters::Synchronizer<CamInfoApproximatePolicy> CamInfoApproximateSync;

	typedef cv::Mat_<float> toccmap;

	class TraversabilityCostmap {

		public:

			TraversabilityCostmap(ros::NodeHandle priv_nh);
			~TraversabilityCostmap();

			double round(double d);


		protected:

			geometry_msgs::PoseStamped map_origin_;

			// thresholds for probability
			float prob_max_;
			float prob_min_;

			image_geometry::StereoCameraModel model_;

			void camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info_left, const sensor_msgs::CameraInfoConstPtr& cam_info_right);

			// callback for detectors detecting traversable areas (e.g. roads / pavements)
			void detectorCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp, const int& idx);
			void detectorCBalt(const sensor_msgs::ImageConstPtr& img, const int& idx);

			ros::NodeHandle nh_;


			std::vector< boost::shared_ptr<image_transport::Subscriber> > sub_list_wo_disp_;
			std::vector< boost::shared_ptr<image_transport::SubscriberFilter> > sub_list_;
			std::vector< boost::shared_ptr<ApproximateSync> > approximate_sync_list_;

			int queue_length_;

			message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;

			boost::shared_ptr<ApproximateSync> approximate_sync_;

			message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info_, sub_r_info_;
			boost::shared_ptr<CamInfoApproximateSync> cam_info_approximate_sync_;


			boost::shared_ptr<image_transport::ImageTransport> it_;

			bool cam_info_received_;

			// this is internal representation of occupancy grid, where 1.0 means occupied
			// TODO consider some filtering / hole filling???
			toccmap occ_grid_;

			bool occ_grid_filter_;

			nav_msgs::MapMetaData occ_grid_meta_;
			ros::Publisher occ_grid_pub_;

			inline bool isValidPoint(const cv::Vec3f& pt);

			float map_res_;
			float map_size_;
			std::string map_frame_;

			tf::TransformListener tfl_;

			void updateIntOccupancyGrid(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImageConstPtr& disp);

			void subscribe(std::string topic);

			void timer(const ros::TimerEvent& ev);

			// timer for periodical publishing of occupancy grid
			ros::Timer timer_;

			ros::ServiceServer srv_get_map_;
			ros::ServiceServer srv_reset_map_;

			bool getMap(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);
			bool resetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

			void createOccGridMsg(nav_msgs::OccupancyGrid& grid);

			void normalize(cv::Point3d& v);

			bool robotPose(geometry_msgs::PoseStamped& pose);
			bool updateMapOrigin();

			void worldToMap(geometry_msgs::Point& p);

			bool initialized_;

			bool filter_output_;

			double max_proj_dist_;

			bool use_disparity_;

			double origin_update_th_;

			int morph_filter_ks_size_;
			int morph_filter_iter_;
			int median_filter_ks_size_;

			std::string detectors_ns_;

	};


}


#endif /* COSTMAP_H_ */
