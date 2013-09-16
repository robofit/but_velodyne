/*
 * hsv_hist_detector_ros.h
 *
 *  Created on: 12.7.2013
 *      Author: beranv
 */

#ifndef HSV_HIST_DETECTOR_ROS_H_
#define HSV_HIST_DETECTOR_ROS_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "rt_road_detection/detectors/hsv_hist_detector.h"
#include <dynamic_reconfigure/server.h>
#include "rt_road_detection/HSVHistDetectorConfig.h"

namespace rt_road_detection {

	class HSVHistDetectorRos
	{

	   public:

			HSVHistDetectorRos(ros::NodeHandle private_nh);
			~HSVHistDetectorRos();


	   protected:

			boost::shared_ptr<image_transport::ImageTransport> it_;
			boost::shared_ptr<HSVHistDetector> det_;

			image_transport::Publisher pub_;
			image_transport::Subscriber sub_;

			void imageCallback(const sensor_msgs::ImageConstPtr& msg);
			void reconfigureCallback(HSVHistDetectorConfig &config, uint32_t level);

			ros::NodeHandle nh_;

			dynamic_reconfigure::Server<HSVHistDetectorConfig> dyn_reconf_srv_;
			dynamic_reconfigure::Server<HSVHistDetectorConfig>::CallbackType dyn_reconf_f_;

			int frame_skip_;
			int skiped_;

			int hbins_;
			int sbins_;
			int wnd_size_;
			int wnd_step_;

			std::string fn_;

			double prob_hit_;
			double prob_miss_;

	   };

}

#endif /* SAMPLE_HUE_DETECTOR_ROS_H_ */
