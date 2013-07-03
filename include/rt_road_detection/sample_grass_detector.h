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

namespace rt_road_detection {


	class SampleRoadDetector {


		public:

			SampleRoadDetector(int hue_min, int hue_max, int median_blur_ks);
			~SampleRoadDetector();
			bool detect(cv_bridge::CvImagePtr in, cv_bridge::CvImagePtr out);

		protected:

			int hue_min_;
			int hue_max_;
			int median_blur_ks_;

		private:

	};


};

#endif /* SAMPLE_ROAD_DETECTOR_H_ */
