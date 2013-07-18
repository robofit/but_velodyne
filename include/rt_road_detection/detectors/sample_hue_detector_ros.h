/*
 * sample_hue_detector_ros.h
 *
 *  Created on: 12.7.2013
 *      Author: zdenal
 */

#ifndef SAMPLE_HUE_DETECTOR_ROS_H_
#define SAMPLE_HUE_DETECTOR_ROS_H_

#include "rt_road_detection/detectors/sample_hue_detector.h"
#include <dynamic_reconfigure/server.h>
#include "rt_road_detection/SampleHueDetectorConfig.h"

namespace rt_road_detection {

class SampleHueDetectorRos
	   {

	   public:

			SampleHueDetectorRos(ros::NodeHandle private_nh);
			~SampleHueDetectorRos();


	   protected:

			boost::shared_ptr<image_transport::ImageTransport> it_;
			boost::shared_ptr<SampleHueDetector> det_;

			image_transport::Publisher pub_;
			image_transport::Subscriber sub_;

			void imageCallback(const sensor_msgs::ImageConstPtr& msg);
			void reconfigureCallback(SampleHueDetectorConfig &config, uint32_t level);

			ros::NodeHandle nh_;

			dynamic_reconfigure::Server<SampleHueDetectorConfig> dyn_reconf_srv_;
			dynamic_reconfigure::Server<SampleHueDetectorConfig>::CallbackType dyn_reconf_f_;

			double prob_hit_;
			double prob_miss_;

	   };

}

#endif /* SAMPLE_HUE_DETECTOR_ROS_H_ */
