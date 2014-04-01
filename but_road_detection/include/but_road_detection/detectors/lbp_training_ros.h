/*
 * lbp_detector_ros.h
 *
 *  Created on: 12.7.2013
 *      Author: xgoldm03
 */

#ifndef LBP_DETECTOR_ROS_H_
#define LBP_DETECTOR_ROS_H_

#include "but_road_detection/detectors/lbp_detector.h"

namespace but_road_detection
{

class LBPTrainingRos
{

public:

  LBPTrainingRos(ros::NodeHandle private_nh);
  ~LBPTrainingRos();


protected:

  ros::NodeHandle nh_;

};

}

#endif /* LBP_DETECTOR_ROS_H_*/
