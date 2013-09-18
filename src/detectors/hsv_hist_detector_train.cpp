/**
 * Developed by dcgm-robotics@FIT group
 * Author: Vita Beran (beranv@fit.vutbr.cz)
 * Date: 20.08.2013 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *
 *------------------------------------------------------------------------------
 */


#include "rt_road_detection/detectors/hsv_hist_detector.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace rt_road_detection;
using namespace std;
using namespace cv;


int main( int argc, char** argv )
{
	int hbin     = 17;
	int sbin     = 17;
	int wnd_size = 45;
	int wnd_step = 15;
	int kernel_type = CvSVM::LINEAR;
	string cmds = "rbf train eval"; 		// rbf, train, eval, export_rois, browse

	string datasetDir = "/home/beranv/data/fit-park-unibrain-no-ir/";
	//string datasetDir = "/home/beranv/data/fit-park-iphone/";
	//string datasetDir = "/home/beranv/data/lodz-park/";


	if( argc >= 6 ) {
		hbin = atoi( argv[1] );
		sbin = atoi( argv[2] );
		wnd_size = atoi( argv[3] );
		wnd_step = atoi( argv[4] );
		datasetDir = string( argv[5] );
		if( argc >= 7 ) cmds = string( argv[6] );
	}

	if( cmds.find("rbf") != string::npos )
		kernel_type = CvSVM::RBF;

	char cfg_id_str[64];
	sprintf( cfg_id_str, "%d_%d_%d_%s", hbin, sbin, wnd_size, (kernel_type==CvSVM::LINEAR?"lin":"rbf") );

	cout << "HSV Detector Training" << endl;
	cout << "Configuration ID [" << cfg_id_str << "]" << endl;
	cout << "Dataset Path: " << datasetDir << endl;


	//////////////////////////////////////////////////////////////
	AnnotMeta annData;
	annData.loadCSV( datasetDir+"annotations.csv", ';' );

	// export ROIs if needed
	if( cmds.find("export_rois") != string::npos )
	{
		annData.exportROIs(
			datasetDir+"images/",
			datasetDir+"positives/",
			datasetDir+"negatives/" );
	}


	//////////////////////////////////////////////////////////////
	HSVHistDetector hsvDetector( 0.7, 0.3, hbin, sbin, wnd_size, wnd_step );

	//////////////////////////////////////////////////////////////
	HSVHistTrainData hsvHistTrainData( hsvDetector.featureExtractor() );
	if( cmds.find("train") != string::npos ||
		cmds.find("eval")  != string::npos )
	{
		// get training data
		hsvHistTrainData.extract( wnd_size, wnd_step, datasetDir+"images/", annData );
	}


	//////////////////////////////////////////////////////////////
	// Model type
	string fn_model = datasetDir+"models/hsvDetModel_"+cfg_id_str+".yaml";

	if( cmds.find("train") != string::npos )
	{
		hsvDetector.train( hsvHistTrainData.features(), hsvHistTrainData.labels() );
		hsvDetector.write( fn_model );
	}
	else
		hsvDetector.read( fn_model );

	if( hsvDetector.empty() )
	{
		cerr << "Error: No Model available." << endl;
		return -1;
	}

	//////////////////////////////////////////////////////////////
	// eval detector on training data
	if( cmds.find("eval") != string::npos )
	{
		float prob = 0;
		hsvDetector.eval( hsvHistTrainData.features(), hsvHistTrainData.labels(), &prob );
		cout << "Evaluation result: " << cfg_id_str << ": " << prob << endl;
	}


	//////////////////////////////////////////////////////////////
	// manual check
	if( cmds.find("browse") != string::npos && annData.size() > 0 )
	{
		int no = 0;
		while(1) {
			Mat img, hsv;
			Mat prob;

			if( annData.getImg( datasetDir+"images/", no, img ) )
			{
				cvtColor( img, hsv, CV_BGR2HSV );
				hsvDetector.detect( hsv, prob );

				Mat mask;
				prob.convertTo( mask, CV_8UC1 );
				img *= 0.7;
				add( img, Scalar(255,-30,-30), img, mask );
				annData.renderRois( img, no );

				imshow( "Input image", img );
			}

			char key = waitKey(0);
			if( key == 'q' || key == 27 )
				break;
			switch (key) {
			case '.' : no = MIN(no+1, annData.size() ); break;
			case ',' : no = MAX(no-1, 0); break;
			}
		}
	}

	return 0;
}


