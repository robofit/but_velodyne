/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Zdenek Materna (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 28/06/2013
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "rt_road_detection/sample_grass_detector.h"

using namespace rt_road_detection;

SampleRoadDetector::SampleRoadDetector(int hsv_min, int hsv_max, int median_blur_ks) {


	hue_min_ = hsv_min;
	hue_max_ = hsv_max;
	median_blur_ks_ = median_blur_ks;

}

SampleRoadDetector::~SampleRoadDetector() {


}

bool SampleRoadDetector::detect(cv_bridge::CvImagePtr in, cv_bridge::CvImagePtr out) {

	cv::Mat hsv;

	//in->image.convertTo(hsv, CV_32FC1);

	hsv = in->image;

	cv::cvtColor( hsv, hsv, CV_BGR2HSV_FULL ); // Hue in range 0-360

	std::vector<cv::Mat> hsv_vec;
	cv::split(hsv,hsv_vec);

	//cv::equalizeHist(hsv_vec[0],hsv_vec[0]);

	cv::Mat bin_mask(hsv_vec[0]);

	//cv::GaussianBlur(hsv_vec[0], hsv_vec[0],cv::Size(3,3),0);

	for(int row = 0; row < hsv_vec[0].rows; ++row) {
	    uchar* p = hsv_vec[0].ptr(row);
	    for(int col = 0; col < hsv_vec[0].cols; ++col) {

	    	if (*p < hue_min_ || *p > hue_max_) bin_mask.at<uchar>(row,col) = 0;
	    	else {

	    		double min = (double)(*p - hue_min_);
	    		double max = (double)(hue_max_ - hue_min_);

	    		bin_mask.at<uchar>(row,col) = (uchar)floor((min/max)*255.0);

	    	}

	    	p++;  //points to each pixel value in turn assuming a CV_8UC1 greyscale image

	    }

	}

	//cv::inRange(hsv_vec[0],cv::Scalar(hue_min_),cv::Scalar(hue_max_),bin_mask);

	cv::medianBlur(bin_mask,bin_mask,median_blur_ks_);

	//bin_mask /= 255.0;

	//out->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	out->encoding = sensor_msgs::image_encodings::MONO8;
	out->header = in->header;
	out->image = bin_mask;

	return true;

}
