/**
* Class for computing LBP (3x3)
*
*
* - generate orto histogram from one or three channel
*
* Support: 
* - uniform LBP (UNIFORM_LBP)
* - rotary invariant LBP ( type ROTARY_INVARIANT_LBP)
*
* Note:
* 
* Source: 
* https://github.com/berak/uniform-lbp/blob/master/lbp2.cpp
*
*/


#include "rt_road_detection/detectors/lbp/lbp.h"


using namespace std;
using namespace cv;

#define M_PI       3.14159265358979323846


union opt {
	__m128i v;
	signed short ss[8];
};

typedef Vec<uchar,1>  Vec1b;



LBP::LBP(int _type=ROTARY_INVARIANT_LBP):type(_type),max_rotary_invariantion_bin(0) 
{
    _radius=1;
    _neighbors=8;
    
    
    if (type==UNIFORM_LBP) {
      
        initUniformLookup();
    }
    else if (type==ROTARY_INVARIANT_LBP  ) {
      
	initRotaryInvariantLookup();
    }
    else if (type==LBP_3X3) {
      
	//no mapping
    }
    else
    {
	string error_message = "Bad LBP type";
	CV_Error(CV_StsError, error_message);
    }
}


/**
* Create mapping lookup table for rotary invariant lbp version
*
* Computing the histogram of LBP codes normalizes for translations and normalization 
* for rotation is achieved by rotation invarianat mapping. In this mapping, each LBP binary code
* is circulary rotated into its minimum value.
*
*/

void LBP::initRotaryInvariantLookup()
{
	_rotary_invariation_lookup.clear();

	 unsigned long N = (int) pow( 2., (int) 8 );

	// Rotation Invariant
	int * tmpMap = new int[N];
	memset( tmpMap, -1, N );

	for( unsigned long i = 0; i < N; i++ ) {
		tmpMap[i] = -1;
    
		unsigned long rm = i;
		unsigned long r = i;
		for( int j = 1; j <= 8 - 1; j++ ) 
		{
			r = rotateLeft( r, 8 );

			if( r < rm )
				rm = r;
			}
			if( tmpMap[rm] < 0 ) {
				tmpMap[rm] = max_rotary_invariantion_bin;
				max_rotary_invariantion_bin = max_rotary_invariantion_bin + 1;
			}
			_rotary_invariation_lookup.push_back( tmpMap[rm] );
	}
}



/**
* RotateLeft
*
*/

unsigned int LBP::rotateLeft( unsigned int i, unsigned int samples ) {
    unsigned int bg = ((i & (1 << (samples - 1))) >> (samples - 1)); // bitget(r,samples)
    unsigned int bs = (i << 1) & ((int) pow( 2., (int) samples ) - 1); // bitshift(r, 1, samples)
    unsigned int j = (bs + bg) & ((int) pow( 2., (int) samples ) - 1); // bitset( bs, 1, bg )
    return j;
}



//------------------------------------------------------------------------------
// UNIFORM (from https://github.com/berak/uniform-lbp/blob/master/lbp2.cpp)
//------------------------------------------------------------------------------
//
// a bitmask is 'uniform' if the number of transitions <= 2.
//
// we precompute the possible values to a lookup(index) table for
// all possible lbp combinations of n bits(neighbours).
//
// check, if the 1st bit eq 2nd, 2nd eq 3rd, ..., last eq 1st,
// else add a transition for each bit.
//
// if there's no transition, it's solid
// 1 transition: we've found a solid edge.
// 2 transitions: we've found a line.
//
// since the radius of the lbp operator is quite small,
// we consider any larger number of transitions as noise,
// and 'discard' them from our histogram, by assinging all of them
// to a single noise bin
//
// this way, using uniform lbp features boils down to indexing into the lut
// instead of the original value, and adjusting the sizes for the histograms.
//
bool bit(unsigned b, unsigned i) {
    return ((b & (1 << i)) != 0);
}



void LBP::initUniformLookup() 
{
    int numSlots = 1 << _neighbors; // 2 ** _neighbours
    max_uniform_bin = 0;
    _uniform_lookup = std::vector<int>(numSlots);
    for ( int i=0; i<numSlots; i++ ) {
        int transitions = 0;
        for ( int j=0; j<_neighbors-1; j++ ) {
            transitions += (bit(i,j) != bit(i,j+1));
        }
        transitions += (bit(i,_neighbors-1) != bit(i,0));

        if ( transitions <= 2 ) {
            _uniform_lookup[i] = max_uniform_bin++;
        } else {
            _uniform_lookup[i] = -1; // mark all non-uniforms as noise channel
        }
    }

    // now run again through the lut, replace -1 with the 'noise' slot (numUniforms)
    for ( int i=0; i<numSlots; i++ ) {
        if ( _uniform_lookup[i] == -1 ) {
            _uniform_lookup[i] = max_uniform_bin;
        }
    }
}




template <typename _Tp> static
void computeOLBP(InputArray _src, OutputArray _dst, int type, std::vector<int> lookup,std::vector<int> rilookup) {

	
	  // get matrices
	  Mat src = _src.getMat();
	  // allocate memory for result
	      if(src.channels()==1)
		      _dst.create(src.rows-2, src.cols-2, CV_8UC1);
	      if(src.channels()==3)
		      _dst.create(src.rows-2, src.cols-2, CV_8UC3);

	  Mat dst = _dst.getMat();
	  // zero the result matrix
	  dst.setTo(0);
	  // calculate patterns
	      union opt a;


	//mask for LBP evaluation
	__m128i mask=_mm_set_epi32 (0x00080004,0x00200010, 0x00020040, 0x00800001);;
	__m128i zero=_mm_setzero_si128();

	vector<Mat> channels(3);
	split(src, channels);
	

	for(int q=0; q<src.channels(); q++)
	{
		int j=1;
		int i=1;
		int code = 0;
		int center =0;

		for(i=1;i<src.rows-1;i++) {

			a.ss[0]=channels[q].at<uchar>(i-1,0);
			a.ss[1]=channels[q].at<uchar>(i,0) ;
			a.ss[2]=channels[q].at<uchar>(i+1,0);
			a.ss[3]=channels[q].at<uchar>(i-1,1);

			a.ss[4]=channels[q].at<uchar>(i+1,2) ;
			a.ss[5]=channels[q].at<uchar>(i+1,1);
			a.ss[6]=channels[q].at<uchar>(i-1,2);
			a.ss[7]=channels[q].at<uchar>(i,2);

			for(j=1;j<src.cols-1;j++) {
				//center pixel 
				center =  channels[q].at<uchar>(i,j);

				//compute LBP
				union
				{
					__m128i q;
					signed short ss[8];
				} 
				result = {
				  _mm_sad_epu8(
					  _mm_and_si128(
						  mask,_mm_cmpgt_epi8(a.v,_mm_set1_epi8(center))),zero)
				};

				code =  result.ss[0]+result.ss[4];

				a.v=_mm_srli_si128(a.v, 6);

				//re-map values in vector 
				a.ss[1]=center;
				a.ss[5]=a.ss[4];

				a.ss[4]=channels[q].at<uchar>(i+1,j+1);
				a.ss[6]=channels[q].at<uchar>(i-1,j+1);
				a.ss[7]=channels[q].at<uchar>(i,j+1);


				//map value ith lookup table and save to new matrix
				if ( type==UNIFORM_LBP ) // replace content of dst with its resp. lookup value
						code= lookup[code];

				if ( type==ROTARY_INVARIANT_LBP ) // replace content of dst with its resp. lookup value
						code= rilookup[code];

				dst.at<_Tp>(i-1,j-1) [q] =  code;
			}       
		}
	}
}


void LBP::compute(InputArray src, OutputArray dst) 
{
	if(type==ROTARY_INVARIANT_LBP || type==LBP_3X3  || type==UNIFORM_LBP)
		computeOLBP< Vec1b >(src, dst, type, _uniform_lookup,_rotary_invariation_lookup);
}


void LBP::compute3CH(InputArray src, OutputArray dst) 
{
	if(type==ROTARY_INVARIANT_LBP || type==LBP_3X3 || type==UNIFORM_LBP)
		computeOLBP<Vec3b>(src, dst, type, _uniform_lookup,_rotary_invariation_lookup);
}


void LBP::histogram(Mat src, OutputArray _dst,int x,int y,int width,int height, int offset)
{
  
	Mat dst=_dst.getMat();
	

	for(int i=y; i < y+height;i++) {
	    for(int j=x;j < (x+width) ;j++) {

	      dst.at<float> (0,src.at<uchar>(i,j)+offset) ++;
	    }
	}
}


/**
*	Create orto LBP histogram for one or three channels
*
*
*/

void LBP::histogramUpdate(InputOutputArray _hist,Mat newRow,Mat oldRow)
{
	Mat hist=_hist.getMat();

	if(newRow.channels()==1)
		for(int i=0;i<oldRow.cols;i++)
		{
			hist.at <int> (newRow.at<uchar> (i))--;
			hist.at <int> (oldRow.at<uchar> (i))++;	
		}

	if(newRow.channels()==3)
		for(int i=0;i<oldRow.cols;i++)
		{
			hist.at <int> (newRow.at<Vec3b> (i) [0])--;
			hist.at <int> (oldRow.at<Vec3b> (i) [0])++;	

			hist.at <int> ((newRow.at<Vec3b> (i) [0])+256)--;
			hist.at <int> ((oldRow.at<Vec3b> (i) [0])+256)++;	

			hist.at <int> ((newRow.at<Vec3b> (i) [0])+512)--;
			hist.at <int> ((oldRow.at<Vec3b> (i) [0])+512)++;	
		}
}


int LBP::getBins()
{
	return max_rotary_invariantion_bin;
}