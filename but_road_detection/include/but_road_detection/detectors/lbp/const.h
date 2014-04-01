/*
 *  const.h
 *  $Id: const.h 33 2012-01-30 15:18:29Z ijuranek $
 *
 *  Author
 *  Roman Juranek <ijuranek@fit.vutbr.cz>
 *
 *  Graph@FIT
 *  Department of Computer Graphics and Multimedia
 *  Faculty of Information Technology
 *  Brno University of Technology
 *
 */

#ifndef _ENGINE_CONST_
#define _ENGINE_CONST_

#include <pmmintrin.h>

/// SSE 128 bit integer.
typedef union
{
  signed char i8[16];     ///< 8 bit signed integer array
  unsigned char u8[16];   ///< 8 bit unsigned integer array
  int i32[4];
  unsigned u32[4];
  __m128i q;              ///< The SSE 128 bit type
} int128;

/// Table mapping
extern const unsigned char block_table[1024];

/// Mask mapping
extern const unsigned char mask_table[1024];

/// Rank index for different mask positions
extern const int rank_table[4][9];

/// Masks
extern const int128 masks[4];

/// Weights for LBP evaluation
extern const int128 lbp_weights[4];

/// Vector of ones (1,1,1...1)
extern const int128 ones;

/// Sign bit mask vector (0x80,0x80,...0x80)
extern const int128 sign_bit;

extern const int128 lbp_valid_mask;

extern const int lbp_bit_order[8];

extern const int lbp_bit_weight[9];

extern const float min_response;

#endif

