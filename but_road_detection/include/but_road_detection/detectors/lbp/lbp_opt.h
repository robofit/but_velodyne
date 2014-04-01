//
//  lbp.cpp
//  Roman Juranek, DCGM, FIT, BUT, Brno
//  ijuranek@fit.vutbr.cz
//
//  NOTES
//  Order of samples in LBP code
//  0 1 2
//  7 c 3
//  6 5 4
//
//  Evaluation function is
//  lbp(\vec v, c) = \sum_{i=0}^{7} (v_i > c) 2^i
//  Where v is vector of local samples ordered according to scheme above, and the c is the central sample.
//

#ifndef _LBP_H_
#define _LBP_H_

#include <cxcore.h>

#ifdef __cplusplus
extern "C" {
#endif

/// SSE optimized calculation of LBP image of the 'src' and store it in 'dst'
/// Both images should be one channel, IPL_DEPTH_8U and same size.
/// The function calculates 8 bit LBP from 3x3 px local area. No post processing is done.
/// \param src Source image
/// \param dst Result LBP image
  void calc_LBP11_sse(IplImage * src, IplImage * dst);

/// 'Stupid' calculation of LBP image of the 'src' and store it in 'dst'
/// Both images should be one channel, IPL_DEPTH_8U and same size.
/// The function calculates 8 bit LBP from 3x3 px local area. No post processing is done.
/// \param src Source image
/// \param dst Result LBP image
  void calc_LBP11_simple(IplImage * src, IplImage * dst);

  void calc_CSLBP11_simple(IplImage * src, IplImage * dst);

#ifdef __cplusplus
}
#endif


#endif

