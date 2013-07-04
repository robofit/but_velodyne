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
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>
//#include <image_geometry/stereo_camera_model.h>

namespace rt_traversability_layer {

	typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, stereo_msgs::DisparityImage> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

	class TraversabilityCostmap {

		public:

			TraversabilityCostmap();
			~TraversabilityCostmap();

		protected:

			image_geometry::PinholeCameraModel model_;

			void camInfoCB(const sensor_msgs::CameraInfoConstPtr& cam_info);

			void notRoadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImage& disp);
			void roadCB(const sensor_msgs::ImageConstPtr& img, const stereo_msgs::DisparityImage& disp);

			ros::NodeHandle nh_;

			//std::vector< boost::shared_ptr<image_transport::Subscriber> > not_road_sub_;

			image_transport::SubscriberFilter not_road_sub_; // TODO make vector...
			message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;

			boost::shared_ptr<ApproximateSync> approximate_sync_;

			boost::shared_ptr<image_transport::ImageTransport> it_;

			bool cam_info_received_;

	};


}


#endif /* COSTMAP_H_ */
