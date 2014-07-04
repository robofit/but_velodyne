/*
 * Similarity.cpp
 *
 *  Created on: 8.1.2014
 *      Author: ivelas
 */

#include "but_calibration_camera_velodyne/Similarity.h"

using namespace std;
using namespace cv;

namespace but_calibration_camera_velodyne
{

void Similarity::computeEntropies()
{
  vector<float> histogram_X(INTENSITIES, 0);
  vector<float> histogram_Y(INTENSITIES, 0);
  map<pair<uchar, uchar>, float> joint_histogram;

  for (int row = 0; row < X.rows; row++)
  {
    for (int col = 0; col < X.cols; col++)
    {
      uchar x_val = X.at<uchar>(row, col);
      uchar y_val = Y.at<uchar>(row, col);

      histogram_X[x_val] += 1;
      histogram_Y[y_val] += 1;

      joint_histogram[pair<uchar, uchar>(x_val, y_val)] += 1;
    }
  }

  float p;
  float points_nm = X.rows * X.cols;
  H_X = H_Y = H_XY = 0;
  for (int i = 0; i < INTENSITIES; i++)
  {
    p = histogram_X[i] / points_nm;
    if (p > 0)
    {
      H_X += -p * log(p);
    }

    p = histogram_Y[i] / points_nm;
    if (p > 0)
    {
      H_Y += -p * log(p);
    }
  }

  float points_nm_pow = points_nm * points_nm;
  for (map<pair<uchar, uchar>, float>::iterator i = joint_histogram.begin(); i != joint_histogram.end(); i++)
  {
    p = i->second / points_nm_pow;
    H_XY = -p * log(p);
  }
}

}
