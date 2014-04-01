/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Zdenek Materna (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 28/06/2013
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "but_road_detection/detectors/sample_hue_detector.h"

using namespace but_road_detection;

SampleHueDetector::SampleHueDetector(int hsv_min, int hsv_max, int sat_min, int median_blur_ks, double hit, double miss)
{


  hue_min_ = hsv_min;
  hue_max_ = hsv_max;
  sat_min_ = sat_min;

  median_blur_ks_ = median_blur_ks;

  prob_hit_ = hit;
  prob_miss_ = miss;

}

SampleHueDetector::~SampleHueDetector()
{


}

bool SampleHueDetector::setProbs(double hit, double miss)
{

  prob_hit_ = hit;
  prob_miss_ = miss;

  return true;

}

bool SampleHueDetector::setParams(int hue_min, int hue_max, int median_blur_ks)
{

  // TODO check param values
  hue_min_ = hue_min;
  hue_max_ = hue_max;
  median_blur_ks_ = median_blur_ks;

  return true;

}



bool SampleHueDetector::detect(cv_bridge::CvImageConstPtr in, cv_bridge::CvImagePtr out)
{

  cv::Mat hsv;

  // we don't need high level of details
  cv::GaussianBlur(in->image, hsv, cv::Size(11, 11), 0);

  if (in->encoding == "rgb8") cv::cvtColor(hsv, hsv, CV_RGB2HSV);   // Hue in range 0-360
  else if ((in->encoding == "bgr8")) cv::cvtColor(hsv, hsv, CV_BGR2HSV);
  else
  {

    ROS_WARN_THROTTLE(1, "Strange encoding!");
    return false;
  }

  std::vector<cv::Mat> hsv_vec;
  cv::split(hsv, hsv_vec);

  cv::Mat_<float> bin_mask(hsv_vec[0]);

  bin_mask.setTo(0.5);

  for (int row = 0; row < hsv_vec[0].rows; row++)
    for (int col = 0; col < hsv_vec[0].cols; col++)
    {

      int overexposed = 0;

      if (in->image.at<cv::Vec3b>(row, col)[0] > 247) overexposed++;
      if (in->image.at<cv::Vec3b>(row, col)[1] > 247) overexposed++;
      if (in->image.at<cv::Vec3b>(row, col)[2] > 247) overexposed++;

      // deal with overexposed areas
      if (overexposed > 1)
      {

        bin_mask.at<float>(row, col) = 0.5;

      }
      else
      {

        if (hsv_vec[0].at<uint8_t>(row, col) > hue_min_ && hsv_vec[0].at<uint8_t>(row, col) < hue_max_ && hsv_vec[1].at<uint8_t>(row, col) > 28) bin_mask.at<float>(row, col) = prob_hit_;
        else bin_mask.at<float>(row, col) = prob_miss_;


      }


    };

  // post-filter some small mess in mask
  cv::medianBlur(bin_mask, bin_mask, median_blur_ks_);

  out->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  //out->encoding = sensor_msgs::image_encodings::MONO8;
  out->header = in->header;
  out->image = bin_mask;

  return true;

}
