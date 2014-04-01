/*
 *  \file lbp.c
 *  $Id: lbp.c 11 2011-11-16 12:30:00Z ijuranek $
 *
 *  \brief Image space LBP operator calculation
 *
 *
 */

#include "lbp.h"
#include "const.h"

#include <cv.h>
#include <cxcore.h>
#include <mmintrin.h>
#include <pmmintrin.h>
#include <emmintrin.h>


static inline void calc_lbp_16_strip(IplImage * src, IplImage * dst, unsigned base)
{
  signed char * src_data = (signed char*)(src->imageData + base);
  unsigned char * dst_data = (unsigned char*)(dst->imageData + base);
  signed char * src_end = (signed char*)src->imageData + (src->height - 1) * src->widthStep;

  __m128i pixels[3];

  // Load first two rows
  //pixels[0] = *(__m128i*)src_data;//_mm_set_epi64(*(__m64*)(src_data+8), *(__m64*)(src_data));
  pixels[0] = _mm_set_epi64(*(__m64*)(src_data + 8), *(__m64*)(src_data));
  //pixels[0] = _mm_xor_si128(pixels[0], sign_bit.q); // conversion from unsigned to signed - invert sign bit
  src_data += src->widthStep;
  //pixels[1] = *(__m128i*)src_data;//_mm_set_epi64(*(__m64*)(src_data+8), *(__m64*)(src_data));
  pixels[1] = _mm_set_epi64(*(__m64*)(src_data + 8), *(__m64*)(src_data));
  //pixels[1] = _mm_xor_si128(pixels[1], sign_bit.q);
  src_data += src->widthStep;

  int phase = 2;

  __m128i * phase_map[3][3] =
  {
    {pixels + 1, pixels + 2, pixels},
    {pixels + 2, pixels, pixels + 1},
    {pixels, pixels + 1, pixels + 2},
  };

  while (src_data < src_end)
  {
    register __m128i weight = ones.q;
    register __m128i code = _mm_setzero_si128();

    //pixels[phase] = _mm_set_epi64(*(__m64*)(src_data+8), *(__m64*)(src_data));
    //pixels[phase] = _mm_xor_si128(pixels[phase], sign_bit.q);
    //pixels[phase] = _mm_xor_si128(_mm_lddqu_si128((__m128i*)src_data), sign_bit.q);
    pixels[phase] = _mm_lddqu_si128((__m128i*)src_data);

    src_data += src->widthStep;
    dst_data += dst->widthStep;

    _mm_prefetch(src_data, _MM_HINT_T0);

    register __m128i a = *(phase_map[phase][0]);
    register __m128i b = *(phase_map[phase][1]);
    register __m128i c = *(phase_map[phase][2]);

    phase++;
    phase = (phase == 3) ? 0 : phase;

    // X . .   A
    // . o .   B
    // . . .   C
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, _mm_slli_si128(a, 1)), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . X .
    // .   .
    // . . .
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, a), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . . X
    // .   .
    // . . .
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, _mm_srli_si128(a, 1)), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . . .
    // .   X
    // . . .
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, _mm_srli_si128(b, 1)), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . . .
    // .   .
    // . . X
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, _mm_srli_si128(c, 1)), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . . .
    // .   .
    // . X .
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, c), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . . .
    // .   .
    // X . .
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, _mm_slli_si128(c, 1)), weight));
    weight = _mm_slli_epi64(weight, 1);

    // . . .
    // X   .
    // . . .
    code = _mm_or_si128(code, _mm_and_si128(_mm_cmplt_epi8(b, _mm_slli_si128(b, 1)), weight));

    _mm_maskmoveu_si128(code, lbp_valid_mask.q, (char*)dst_data); // store the results - unaligned write
  }
}


void calc_LBP11_sse(IplImage * src, IplImage * dst)
{
  for (int x = 0; x < src->width; x += 14)
  {
    calc_lbp_16_strip(src, dst, x);
  }
  _mm_empty();
}


void calc_LBP11_simple(IplImage * src, IplImage * dst)
{
  unsigned char * src_row = (unsigned char*)src->imageData;
  unsigned char * dst_row = (unsigned char*)dst->imageData + dst->widthStep + 1;

  while (src_row < (unsigned char*)src->imageData + (src->height - 3) * src->widthStep)
  {
    for (int x = 0; x < src->width - 3; ++x)
    {
      int values[9];
      int * v_i = values;
      unsigned char * s_data = src_row + x;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j, ++v_i)
        {
          *v_i = *(s_data + (i * src->widthStep) + j);
        }
      int code = 0;

      for (int i = 0; i < 8; ++i)
        code |= (values[4] > values[lbp_bit_order[i]]) << i;

      *(dst_row + x) = code;
    }

    src_row += src->widthStep;
    dst_row += dst->widthStep;
  }
}

void calc_CSLBP11_simple(IplImage * src, IplImage * dst)
{
  unsigned char * src_row = (unsigned char*)src->imageData;
  unsigned char * dst_row = (unsigned char*)dst->imageData + dst->widthStep + 1;

  while (src_row < (unsigned char*)src->imageData + (src->height - 3) * src->widthStep)
  {
    for (int x = 0; x < src->width - 3; ++x)
    {
      int values[9];
      int * v_i = values;
      unsigned char * s_data = src_row + x;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j, ++v_i)
        {
          *v_i = *(s_data + (i * src->widthStep) + j);
        }
      int code = 0;

      for (int i = 0; i < 4; ++i)
      {
        code |= (abs(values[lbp_bit_order[i]] - values[lbp_bit_order[i + 4]]) > 64) << i;
      }

      *(dst_row + x) = code;
    }

    src_row += src->widthStep;
    dst_row += dst->widthStep;
  }
}

