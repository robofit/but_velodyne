/*
 * sample_road_detector.h
 *
 *  Created on: 3.7.2013
 *      Author: imaterna
 */

#ifndef SAMPLE_ROAD_DETECTOR_H_
#define SAMPLE_ROAD_DETECTOR_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace but_road_detection
{


class SampleHueDetector
{

public:

  SampleHueDetector(int hue_min, int hue_max, int sat_min, int median_blur_ks, double hit, double miss);
  ~SampleHueDetector();
  bool detect(cv_bridge::CvImageConstPtr in, cv_bridge::CvImagePtr out);

  bool setParams(int hue_min, int hue_max, int median_blur_ks);
  bool setProbs(double hit, double miss);

protected:

  int hue_min_;
  int hue_max_;
  int sat_min_;
  int median_blur_ks_;

  double prob_hit_;
  double prob_miss_;

private:

};


};

#endif /* SAMPLE_ROAD_DETECTOR_H_ */
