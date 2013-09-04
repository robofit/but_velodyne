#ifndef __LBP_H__
#define __LBP_H__


#include <opencv/cv.h>
#include <stdlib.h>
#include <cmath>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <emmintrin.h>

using namespace cv;

#define LBP_3X3 0
#define UNIFORM_LBP 1
#define ROTARY_INVARIANT_LBP 2



class LBP
{
  
	//old
	int _neighbors;
	int _radius;

	int type;
	std::vector<int> _uniform_lookup;
	int max_uniform_bin;
	int max_rotary_invariantion_bin;
	std::vector<int> _rotary_invariation_lookup;


	public:
		LBP(int type);
		void compute(InputArray src, OutputArray dst) ;
		void compute3CH(InputArray src, OutputArray dst);	
		void histogramUpdate(InputOutputArray hist,Mat newRow,Mat oldRow);
		void histogram(Mat src, OutputArray hist,int x,int y,int width,int height);
		int getBins();	
		
	private:
		void initUniformLookup();
		void initRotaryInvariantLookup();
		unsigned int rotateLeft( unsigned int i, unsigned int samples );

		
};

#endif