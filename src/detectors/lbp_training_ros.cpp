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


	int width_block,width_cell,height_block,height_cell;
	string fileNameCSV,fileName;

	nh_.param("width_cell",width_cell,16);
	nh_.param("width_block",width_block,32);
	nh_.param("height_cell",height_cell,16);
	nh_.param("height_block",height_block,32);

	
	//TO DO
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
	
	LBPDetector * detector=new LBPDetector(width_cell,height_cell,width_block,height_block,0.0,0.0,fileName);
	
	detector->trainLBP(fileNameCSV,fileName);
	
}

LBPTrainingRos::~LBPTrainingRos() {
  

}

