/*
 * lbp_detector.h
 *
 *  Created on: 12.7.2013
 *      Author: xgoldm03
 */


#ifndef LBP_DETECTOR_H_
#define LBP_DETECTOR_H_


#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
#include "rt_road_detection/detectors/lbp/lbp.h"


using namespace std;

namespace rt_road_detection {

  class LBPDetector
  {
		CvSVM svm;
		LBP lbp;

	public: 
		LBPDetector(int _width_cell,int _height_cell, int _width_block, int _height_block, double _prob_min, double _prob_max ,double _flat_surface_in_block, double _prob_overexposure,double _svm_threshold, string svm_file);
		
		//for train
		LBPDetector();
		
		void train(string train_data_path,string output_file);
		
		
		bool map(cv_bridge::CvImageConstPtr in, cv_bridge::CvImagePtr out);
		void setCoeficients(string file);
		void detect(cv::InputArray input,float * probability);
		bool setParams(int _width_cell,int _height_cell, int _width_block, int _height_block, double _svm_threshold, double _flat_surface_in_block) ;
		void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, vector<Rect>& rois, char separator = ';');
		
		
	private:
		void feature_extractor(InputArray input,OutputArray output);
		
		int width_block;
		int height_block;
		int width_cell;
		int height_cell;
		
		double prob_min;
		double prob_max;
		double svm_threshold;
		
		
		int flat_surface_avg_color;
		
		double prob_overexposure;
		double flat_surface_in_block;
  };

}

#endif /* LBP_DETECTOR */
