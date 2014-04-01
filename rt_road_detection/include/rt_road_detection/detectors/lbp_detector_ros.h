/*
 * lbp_detector_ros.h
 *
 *  Created on: 12.7.2013
 *      Author: xgoldm03
 */

#ifndef LBP_DETECTOR_ROS_H_
#define LBP_DETECTOR_ROS_H_

#include "rt_road_detection/detectors/lbp_detector.h"
#include <dynamic_reconfigure/server.h>
#include "rt_road_detection/LBPDetectorConfig.h"

namespace rt_road_detection {

	class LBPDetectorRos
	{

	public:

		    LBPDetectorRos(ros::NodeHandle private_nh);
		    ~LBPDetectorRos();


	protected:

		    boost::shared_ptr<image_transport::ImageTransport> it_;
		    boost::shared_ptr<LBPDetector> det_;

		    image_transport::Publisher pub_;
		    image_transport::Subscriber sub_;

		    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		    void reconfigureCallback(LBPDetectorConfig &config, uint32_t level);

		    ros::NodeHandle nh_;

		    dynamic_reconfigure::Server<LBPDetectorConfig> dyn_reconf_srv_;
		    dynamic_reconfigure::Server<LBPDetectorConfig>::CallbackType dyn_reconf_f_;

		    int frame_skip_;
		    int skiped_;

	};

}

#endif /* LBP_DETECTOR_ROS_H_*/
