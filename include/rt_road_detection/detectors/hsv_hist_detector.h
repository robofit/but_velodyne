/*
 * hsv_hist_detector.h
 *
 *  Created on: 3.7.2013
 *      Author: beranv
 */

#ifndef SAMPLE_ROAD_DETECTOR_H_
#define SAMPLE_ROAD_DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace rt_road_detection {


	class HSVHistFeature {
	public:
		HSVHistFeature( int hbins = 18, int sbins = 25 );
		~HSVHistFeature() {}

		void init( int hbins, int sbins );
		int  length() const;
		inline int  hbins() const { return hbins_;}
		inline int  sbins() const { return sbins_;}

		// extract one feature from entire HSV image
		bool extract( cv::Mat& hsv, cv::Mat& feature_vector ) const;

		// extract one feature from HSV image ROI
		bool extract( cv::Mat& hsv, cv::Rect roi, cv::Mat& feature_vector ) const;

		// extract more HSV image features using flowing window
		bool extract( cv::Mat& hsv, int wnd_size, int wnd_step, cv::Mat& feature_vector ) const;

	private:
		// histogram parameters
		int hbins_, sbins_;
		int histSize_[2];
		float hranges_[2], sranges_[2];
		const float * ranges_[2];
		int channels_[2];
	};



#define ROIinMat(roi,m) ( !(m).empty() && (roi).x >= 0 && (roi).y >= 0 && (roi).width >0 && (roi).height > 0 	&& (roi).x+(roi).width  < (m).cols && (roi).y+(roi).height < (m).rows )

	class iROI
	{
	public:
		iROI() : cid_(0) {}
		~iROI(){}

		std::string fname_;
		cv::Rect roi_;
		int cid_;
	};

	typedef cv::Ptr<iROI> iROI_ptr;
	typedef std::map<std::string, std::vector<iROI_ptr> >::iterator iROI_it;

	class AnnotMeta
	{
	public:
		AnnotMeta(){}
		~AnnotMeta(){}

		bool loadCSV( const std::string& csv_filename, char separator = ',' );
		bool exportROIs( const std::string& DataDir, const std::string& PositiveDir, const std::string& NegativeDir );

		inline int  size() const { return (int)irois_.size(); }
		bool getImg( const std::string& DataDir, int i, cv::Mat& img );
		void renderRois( cv::Mat& img, int i );

		std::map<std::string, std::vector<iROI_ptr> > irois_;
		std::vector<iROI_ptr> Prois_, Nrois_, Arois_;
	};


	class  HSVHistTrainData
	{
	public:
		HSVHistTrainData( const HSVHistFeature& featureExtractor ) :
			featureExtractor_(featureExtractor) {};
		~HSVHistTrainData() {};

		inline const cv::Mat& labels()   const { return labels_; }
		inline const cv::Mat& features() const { return features_; }

		bool extract( int wnd_size, int wnd_step, const std::string& DataDir, AnnotMeta& annData );

		bool read( const std::string& filename );
		bool write( const std::string& filename );

	private:
		cv::Mat features_;
		cv::Mat labels_;
		const HSVHistFeature& featureExtractor_;
	};


	class HSVHistDetector
	{
	public:
		HSVHistDetector( double hit, double miss, int hbins = 18, int sbins = 25, int wnd_size = 30, int wnd_step = 5);
		~HSVHistDetector();

		inline const HSVHistFeature& featureExtractor() const { return hsvftr_; }
		inline bool empty() const { return svm_.get_var_count() == 0; }

		// extract more HSV image features using flowing window
		bool detect( cv::Mat& hsv, int wnd_size, int wnd_step, cv::Mat& probability );

		bool predict( const cv::Mat& data, cv::Mat& result );
		bool eval( const cv::Mat& data, const cv::Mat& labels, float * precision );

		bool train( const cv::Mat& data, const cv::Mat& labels,	int kernel_type = CvSVM::LINEAR );

		bool read( const std::string& filename );
		bool write( const std::string& filename );

	private:
		CvSVM svm_;
		HSVHistFeature hsvftr_;
		cv::Mat sample_idx_;
		int wnd_size_;
		int wnd_step_;

		double prob_hit_;
		double prob_miss_;

	};


};




#endif /* SAMPLE_ROAD_DETECTOR_H_ */
