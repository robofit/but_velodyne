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


LBPDetector::LBPDetector(int _width_cell=32,int _height_cell=32, int _width_block=64, int _height_block=64, double _prob_min=0.3, double _prob_max=0.7,double _flat_surface_in_block=0.7, double _prob_overexposure=0.5, double _svm_threshold=0.0,bool _hue_channel=false, string svm_file=""):lbp(ROTARY_INVARIANT_LBP)
{
	ifstream fin(svm_file.c_str());
	
	if (!fin)  // check to see if file exists
	{
	  cout << "LBP DETECTOR: File with SVM coeficients doesn't exists\n";
	}
	else
	{
	    try {
	      svm.load(svm_file.c_str());
	      cout << "LBP DETECTOR: File with coeficients was load\n";
	    } catch (cv::Exception& e) {
	      
		 cout << "LBP DETECTOR: File with SVM coeficients doesn't exists\n";
	    }
	}
	
	fin.close();

	
	
	
	prob_min=_prob_min;
	prob_max=_prob_max;
	
	width_cell=_width_cell;
	width_block=_width_block;
	height_cell=_height_cell;
	height_block=_height_block;
	svm_threshold=_svm_threshold;
	
	hue_channel=_hue_channel;
	
	prob_overexposure= _prob_overexposure;
	flat_surface_in_block = _flat_surface_in_block;

}


LBPDetector::LBPDetector():lbp(ROTARY_INVARIANT_LBP)
{

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
	
	*probability=svm.predict(histogram,true);
	
	//DFVal
	if(*probability>svm_threshold)
	{
	  *probability=-1.0;
	}
	else
	  *probability=1.0;
}


/**
 *  Method for ROS dynamic reconfigure
 * 
 * 
 */
bool LBPDetector::setParams(int _width_cell,int _height_cell, int _width_block, int _height_block, double _svm_threshold, double _flat_surface_in_block) {

	
	width_cell=_width_cell;
	width_block=_width_block;
	height_cell=_height_cell;
	height_block=_height_block;
	svm_threshold=_svm_threshold;
	flat_surface_in_block = _flat_surface_in_block;

	return true;
}





/** 
* Read csv file with image list for training classificator
* 
*/
void LBPDetector::read_csv(const string& filename, vector<string>& images, vector<int>& labels, vector<Rect>& rois, string dir, char separator ) {
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
	
	
        getline(liness, classlabel);
	
        if(!path.empty() && !classlabel.empty()) {
	  
	    Mat img=imread(dir +path,1);
	    
	    if(y > 0 && x>0 && img.cols > (x+width) && img.rows > (y+height) && width !=0 && height !=0 && width>2 && height >2)
	    {
	      images.push_back(path);
	      labels.push_back(atoi(classlabel.c_str()));
	      rois.push_back(Rect(x,y,width,height));  
	    }
            
        }
    }
}


void LBPDetector::feature_extractor(InputArray input,OutputArray output,int channel)
{

	Mat src=input.getMat();
	
	output.create(src.rows-2, src.cols-2, CV_8UC1);
	
	Mat dst=output.getMat();
	dst.setTo(0);
	
	if(channel==-1)
	{
	  //lbp compute for one channel
	  lbp.compute(src,dst);
	}
	else
	{
	  vector<Mat> channels;
	  split(src,channels);
	  lbp.compute(channels[channel],dst);
	}
	
}



/** 
* Train LBP classificator, this method create XML file with SVM coeficients
* 
*
*
*/

void LBPDetector::train(std::string csv_file,string output_file,string dir, int type)
{	
	vector<string> images;
	vector<int> labels;
	vector<Rect> rois;
	
	
	int width_block=90;
	int height_block=90;
	int width_cell=48;
	int height_cell=48;
	int height_cell_ref=height_cell;
	int height_block_ref=height_block;
	int width_block_ref=width_block;
	
	int bins=lbp.getBins();

	//try load images from paths to vector
	try {
	    read_csv(csv_file, images, labels, rois, dir);
	} catch (cv::Exception& e) {
	  
	    cerr << "Error opening file \"" << csv_file << "\". Reason: " << e.msg << endl;
	    exit(1);
	}

	if(images.size() <= 1) {
	    string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
	    CV_Error(CV_StsError, error_message);
	}

	cv::Mat train_data,train_data_label;
	
	Mat histogram, histogram2;
	
	// !!! pocet sloupcu v histogramu, v pripade LBP si zjistim kolik sloupcu budu mit z objketu lbp
	// !!! train_data je matice, pricemz se na zacatku nastavi 0 radku a sirka pocet sloupcu histogramu
	
	
	//ok
	if(type==ONE_CHANNEL)
	{
	  histogram.create(1, bins , CV_32F);
	}
	else
	{
	  histogram.create(1, 2*bins , CV_32F);
	  histogram2.create(1, 2*bins , CV_32F);
	}
	
	
	train_data.create(0,2*lbp.getBins(),CV_32FC1);
	train_data_label.create(0,1,CV_32FC1);

	for(unsigned int k=0;k<images.size();k++)
	{
		Mat output, output2, histogram3,histogram4;
		histogram.setTo(0);
		Mat image_gray, image_rgb;

		Mat img=imread(dir +images[k],1);
		
		//feature extraction !!! v teto casti provedu vypocet LBP obrazu ze vstupniho snimku prevedeneho do gray
		cv::cvtColor(img, image_gray, CV_BGR2GRAY);
		cv::cvtColor(img, image_rgb, CV_BGR2HSV);
		
		feature_extractor(image_gray(rois[k]),output,-1);
		feature_extractor(image_rgb(rois[k]),output2,0);


		for (int i = 0; i < output.cols; i+= width_cell) {

		  
			if(i+width_block >=output.cols)
					width_block=output.cols-i;
			
			
			if(height_block > output.rows)
					height_block=output.rows;
			
			//!!! vypocet histogramu z dane oblasti, opet je to ve tride lbp
			histogram.setTo(0);
			
			lbp.histogram(output,histogram,i, 0, width_block, height_block);
			
			if(type==TWO_CHANNEL)
			{
			  histogram2.setTo(0);
			  lbp.histogram(output2,histogram2,i, 0, width_block, height_block, bins);  

			}
			
				
			for (int j = 0; j < output.rows; j += height_cell_ref) 
			{
			  
				//!!! vlozeni histogramu do vectoru 
				//compute normalize histogram
				
				normalize(histogram,histogram3);
				
				if(type==TWO_CHANNEL)
				{
				    normalize(histogram2,histogram4);
				    histogram3=histogram3+histogram4;
				}
				
				
				train_data.push_back(histogram3);
				

				//push image class
				Mat class_label = Mat::ones(1, 1, CV_32FC1);
				class_label.at <float> (0,0) = (float) labels[k]; 
				train_data_label.push_back(class_label);
				
				
				/********************************************************/
				/***************** compute new histogram********************/


				//if will be last block in the cols update window size
				if(j+height_block+height_cell >=output.rows && height_cell == height_cell_ref )
				{
				    height_cell=output.rows-j-height_block;
				}

				
				
				if((j+height_cell + height_block < output.rows))
				{
					for(int l=0;l<height_cell;l++)
					{
						for(int q=0;q<width_block;q++)
						{
							histogram.at <float> (output.row(j+l).at<uchar> (i+q))--;
							histogram.at <float> (output.at<uchar> (j+height_block_ref+l,q+i))++;
							
							if(type==TWO_CHANNEL)
							{
							  histogram2.at <float> (output2.row(j+l).at<uchar> (i+q)+bins)--;
							  histogram2.at <float> (output2.at<uchar> (j+height_block_ref+l,q+i)+bins)++;
							}
						}
					}
				}

			}

			//reset height block values
			height_block=height_block_ref;
			height_cell=height_cell_ref;
		}
	      
		width_block=width_block_ref;
		
		img.release();
	}

	//train
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
	cv::Mat input_rgb, input_gray, input;
	
	int type=ONE_CHANNEL;
	
	if(hue_channel==true)
	  type=TWO_CHANNEL;
	
	/** convert to gray **/
	if (in->encoding == "rgb8") cv::cvtColor( in->image, input_gray, CV_RGB2GRAY);
	else if ((in->encoding == "bgr8")) cv::cvtColor( in->image, input_gray, CV_BGR2GRAY);
	else {
	  return false;
	}
	
	
	/** convert to hsv **/
	if (in->encoding == "rgb8") { cv::cvtColor( in->image, input_rgb, CV_RGB2HSV); } // (in->image).copyTo(input_rgb);
	else if ((in->encoding == "bgr8")) { cv::cvtColor( in->image, input_rgb, CV_BGR2HSV);}
	else {
	  return false;
	}
  
	std::vector<cv::Mat> rgb_vec;
	Mat histogram,histogram2,src,src2,map;
	int bins = lbp.getBins();

	int height_block_ref=height_block;
	int width_block_ref=width_block;
	
	
	/** histogram create **/
	if(type==ONE_CHANNEL)
	  histogram.create(1, bins , CV_32F);
	else
	{
	  histogram.create(1, 2*bins , CV_32F);
	  histogram2.create(1, 2*bins , CV_32F);
	}
	
	/** compute lbp from gray channel**/
	lbp.compute(input_gray,src);
	
	
	/** compute lbp from one channel from HSV **/
	if(type==TWO_CHANNEL)
	{
	  cv::split(input_rgb,rgb_vec);
	  lbp.compute(rgb_vec[0],src2);
	}

	/** create output probability map **/
	//if the output is float matrix	
	map.create(input_gray.rows,input_gray.cols,CV_32F);
	map.setTo(0);
	

	float probability=0;

	Mat histogram3,histogram4;
	for (int i = 0; i < src.cols; i+= width_cell) {

		if(i+width_block >=src.cols)
				width_block=src.cols-i;

		
		/** compute LBP histograms **/
		histogram.setTo(0);
		lbp.histogram(src,histogram,i, 0, width_block, height_block,0);

		if(type==TWO_CHANNEL)
		{
		  histogram2.setTo(0);
		  lbp.histogram(src2,histogram2,i, 0, width_block, height_block, bins);
		}

		int flag=PROCESS;
		
		for (int j = 0; j < src.rows; j += height_cell) 
		{
			if(flag==END)
			  continue;
			
			
			 /********************************************************/
			 /***************** overexposure_detect *********************/
			
			Scalar result = sum(histogram);
			
			if((histogram.at<float>(0)/result[0]) > flat_surface_in_block)
			{
			      probability=prob_overexposure;

			}
			else
			{
			    /********************************************************/
			    /***************** texture classification *********************/
			    
			   // this->detect(histogram,&probability);
			    //compute normalize histogram
			    normalize(histogram,histogram3);
			    
			    if(type==TWO_CHANNEL)
			    {
			      normalize(histogram2,histogram4);
			      histogram3=histogram3+histogram4;
			    }

			    probability=svm.predict(histogram3,true);
			    
			    //DFVal
			    if(probability>svm_threshold)
			    {
			      probability=prob_max;
			    }
			    else
			      probability=prob_min;

			    
			}
			

			

			/********************************************************/
			/***************** save probability *********************/
			
			for(int a=0;a <height_block+2;a++)
			{
				for(int b=0;b <width_block+2; b++)
				{
					//if cell is empty
					if(map.at<float> (j+a,i+b)==0.0)
					{
					  map.at<float> (j+a,i+b)=probability;
					}
						
					else
					{
					
					  //compute average of the previous and current value
					  map.at<float> (j+a,i+b)= (float) ((probability+ (map.at<float> (j+a,i+b)))/2);
					}
						
				}
			}

			
			/********************************************************/
			/***************** update histogram********************/

			if(flag !=LAST_BLOCK)
			{
			
				//if will be last block in the cols update window size
				if(j+height_block + height_cell >src.rows)
				{
					height_block=src.rows-j-height_cell;
					flag=LAST_BLOCK;
				}
				
				for(int l=0;l<height_cell;l++)
				{
					for(int q=0;q<width_block;q++)
					{
						histogram.at <float> (src.row(j+l).at<uchar> (i+q))--;
						histogram.at <float> (src.at<uchar> (j+height_block+l,q+i))++;
						
						if(type!=ONE_CHANNEL)
						{
							histogram2.at <float> (src2.row(j+l).at<uchar> (i+q)+bins)--;
							histogram2.at <float> (src2.at<uchar> (j+height_block+l,q+i)+bins)++;
						}	
					}
				}
			}
			else
			{
			  flag=END;
			}

		}
	
		//reset height block values
		height_block=height_block_ref;
      }
    
      width_block=width_block_ref;
      
      //if the output is float matrix
      out->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      
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
