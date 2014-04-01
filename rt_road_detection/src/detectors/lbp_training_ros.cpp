/*
 * lbp_training_ros.cpp
 *
 *  Created on: 5.8.2013
 *      Author: xgoldm03
 */

#include "rt_road_detection/detectors/lbp_training_ros.h"


using namespace rt_road_detection;

LBPTrainingRos::LBPTrainingRos(ros::NodeHandle private_nh) {

	nh_ = private_nh;

	string fileNameCSV,fileName;

	string svm_file="svm.xml";
	string csv_file="svm.csv";
	
	
	nh_.param("file_xml",fileName,svm_file);
	nh_.param("file_csv",fileNameCSV,csv_file);

	ifstream fin(fileNameCSV.c_str());
	
	if (!fin)  // check to see if file exists
	{
	  ROS_ERROR("File (%s) with information about training data doesnt exists",fileNameCSV.c_str());
	}
	
	fin.close();
	
	LBPDetector * detector=new LBPDetector();
	
	detector->train(fileNameCSV,fileName);
	
}

LBPTrainingRos::~LBPTrainingRos() {
  

}

