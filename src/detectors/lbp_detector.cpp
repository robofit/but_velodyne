/*
 *  LBP detector for ROS
 * 
 *  Note: value of the cell should never be greater than value of block
 *  lbp_detector.cpp
 *
 * 
 *  Created on: 15.8.2013
 *  Author: xgoldm03 <xgoldm03@stud.fit.vutbr.cz
 */


#include "rt_road_detection/detectors/lbp_detector.h"


#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;
using namespace rt_road_detection;


LBPDetector::LBPDetector(int _width_cell=32,int _height_cell=32, int _width_block=64, int _height_block=64, double _prob_min=0.3, double _prob_max=0.7, string svm_file=""):lbp(1,8,ROTARY_INVARIANT_OLBP)
{
	ifstream fin(svm_file.c_str());
	
	if (!fin)  // check to see if file exists
	{
	  cout << "LBP DETECTOR: File with SVM coeficients doesnt exists\n";
	}
	else
	{
	  svm.load(svm_file.c_str());
	  cout << "LBP DETECTOR: File with coeficients was load\n";
	}
	
	fin.close();

	
	prob_min=_prob_min;
	prob_max=_prob_max;
	
	
	width_cell=_width_cell;
	width_block=_width_block;
	height_cell=_height_cell;
	height_block=_height_block;
}



/**
 *  Detect road
 * 
 * 
 */

void LBPDetector::detect(InputArray input,float * probability)
{
	
	Mat src=input.getMat();
	Mat histogram;
	normalize(src,histogram);
	
	
	*probability=svm.predict(histogram);
}


//TO DO metho for dynamic change param
bool LBPDetector::setParams(int width_cell,int height_cell, int width_block, int height_block) {

	// TODO check param values


	return true;
}





/** 
* Read csv file with image list for training classificator
* 
*/
void LBPDetector::read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, vector<Rect>& rois, char separator ) {
    std::ifstream file(filename.c_str(), ifstream::in);

    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel,_width,_height,_x,_y;
    
    int width;
    int height;
    
    int x,y;
    
    char separator2 = ' ';
    
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
	getline(liness, classlabel, separator2);
	
	getline(liness, _x, separator2);
	getline(liness, _y, separator2);
	getline(liness, _width, separator2);
	getline(liness, _height);

	
	
	//conver string to int
	istringstream ( _width  ) >> width;
	istringstream ( _height  ) >> height;
	istringstream ( _x  ) >> x;
	istringstream ( _y  ) >> y;
	

	//cout << x << " " << y << " " << width  << " " << height << "\n";
	
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 1));
            labels.push_back(atoi(classlabel.c_str()));
	    rois.push_back(Rect(x,y,width,height));
        }
    }
}



/** 
* Train LBP classificator, this method create XML file with SVM coeficients
* 
*
*
*/

void LBPDetector::trainLBP(std::string csv_file,string output_file)
{	
      vector<Mat> images;
      vector<int> labels;
      vector<Rect> rois;

	//try load images from paths to vector
	try {
	  read_csv(csv_file, images, labels, rois);
	} catch (cv::Exception& e) {
	    cerr << "Error opening file \"" << csv_file << "\". Reason: " << e.msg << endl;
	    exit(1);
	}

	    if(images.size() <= 1) {
	    string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
	    CV_Error(CV_StsError, error_message);
	}

	cv::Mat train_data,train_data_label;
	train_data.create(0,lbp.getBins(),CV_32FC1);
	train_data_label.create(0,1,CV_32FC1);


	Mat histogram;
	histogram.create(1, lbp.getBins(), CV_32FC1);
	

	for(unsigned int i=0;i<images.size();i++)
	{
		Mat output;
		histogram.setTo(0);
		Mat image_hue;

		cv::cvtColor(images[i], image_hue, CV_BGR2GRAY);
		//cv::cvtColor(images[i], image_hue, CV_RGB2HSV);
		
		//TO DO
		Mat dst_roi = image_hue(rois[i]);
		vector<Mat> channels;

		split(dst_roi,channels);

		//lbp compute for one channel
		lbp.compute(channels[0],output);
		
		
		//compute normalize histogram
		lbp.histogram(output,histogram,0,0,output.cols,output.rows);
		normalize(histogram,histogram);
		train_data.push_back(histogram);
	
		
		//push image class
		Mat class_label = Mat::ones(1, 1, CV_32FC1);
		class_label= labels[i]; 
		train_data_label.push_back(class_label);
	}


	svm.train(train_data,train_data_label);
	
	//save SVM coeficients
	svm.save(output_file.c_str());
}



/*
* Fucntion for create probability map
*
*/

bool LBPDetector::map(cv_bridge::CvImageConstPtr in, cv_bridge::CvImagePtr out)
{

	cv::Mat input;
	
	if (in->encoding == "rgb8") cv::cvtColor( in->image, input, CV_RGB2GRAY);
	else if ((in->encoding == "bgr8")) cv::cvtColor( in->image, input, CV_BGR2GRAY);
	else {

	  ROS_WARN_THROTTLE(1,"Strange encoding!");
	  return false;
	}
  
	std::vector<cv::Mat> hsv_vec;
	cv::split(input,hsv_vec);

	Mat histogram,src,map;
	int bins = lbp.getBins();

	int height_block_ref=height_block;
	int width_block_ref=width_block;
	histogram.create(1, bins , CV_32F);
	
	lbp.compute(hsv_vec[0],src);
  
	//map.create(input.rows,input.cols,CV_8U);
	
	
	//if the output is float matrix
	
	map.create(input.rows,input.cols,CV_32F);
	
	map.setTo(0);
	
	//cout << "MAP: Properties src.cols: " << src.cols << "src.rows: " << src.rows <<  "height_block: " << height_block <<  "width_block: " << width_block <<  "width_cell: " << width_cell << " \n";

	float probability=0;
	for (int i = 0; i < src.cols; i+= width_cell) {

		if(i+width_block >=src.cols)
				width_block=src.cols-i;

		lbp.histogram(src,histogram,i, 0, width_block, height_block);
		
		int rotate=0;

		for (int j = 0; j < src.rows; j += height_cell) 
		{

			/********************************************************/
			/***************** save probability *********************/
			
			this->detect(histogram,&probability);
			
			if(probability>0)
			  probability=prob_min;
			else
			  probability=prob_max;

			for(int a=0;a <height_block;a++)
			{
				for(int b=0;b <width_block; b++)
				{
					//if cell is empty
					if(map.at<float> (j+a,i+b)==0.0)
					{
					  map.at<float> (j+a,i+b)=probability;
					}
						
					else
					{
					
					  //compute average of the previous and current value
					  map.at<float> (j+a,i+b)= (float) (((int) probability+(int) (map.at<float> (j+a,i+b)))/2);
					}
						
				}
			}

			
			/********************************************************/
			/***************** compute new histogram********************/


			//if will be last block in the cols update window size
			if(j+2*height_block >=src.rows)
				height_block=src.rows-j-height_block;

			//cout << histogram << " \n\n";
			
			if((j+height_cell < src.rows))
			{

				for(int l=0;l<height_cell;l++)
				{
					for(int q=0;q<width_block;q++)
					{
						histogram.at <float> (src.row(j+l).at<uchar> (i+q))--;
						histogram.at <float> (src.at<uchar> (j+height_block+l,q+i))++;	
					}
				}
			}

		}
	
		//reset height block values
		height_block=height_block_ref;
    }
    
      width_block=width_block_ref;
      
      //if the output is float matrix
      out->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      
      //out->encoding = sensor_msgs::image_encodings::MONO8;
      out->header = in->header;
      out->image = map;
    
      return true;
}


void LBPDetector::setCoeficients(string file)
{
	if(file != "")
	{
		svm.load(file.c_str());
	}
}