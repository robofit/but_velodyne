/*
 * Image.h
 *
 *  Created on: 13.11.2013
 *      Author: ivelas
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace but_calibration_camera_velodyne
{

namespace Image
{

typedef enum
{
  IDT_EDGES, NONE
} Processing;

class Image
{
public:
  Image(cv::Mat _img);

  cv::Mat computeEdgeImage();

  // Inverse Distance Tranform
  cv::Mat computeIDTEdgeImage();
  cv::Mat computeIDTEdgeImage(cv::Mat &edge_img);
  bool detect4Circles(float canny_thresh, float center_thresh, std::vector<cv::Point2f> &centers,
                      std::vector<float> &radiuses);
  void threshold(int threshold)
  {
    cv::threshold(img, img, threshold, 255, cv::THRESH_TOZERO);
  }

  void show(const char *name = "Image", int wait = 0)
  {
    cv::imshow(name, img);
    cv::waitKey(wait);
  }

  static cv::Vec3b atf(cv::Mat rgb, cv::Point2f xy_f)
  {
    cv::Vec3i color_i;
    color_i.val[0] = color_i.val[1] = color_i.val[2] = 0;

    int x = xy_f.x;
    int y = xy_f.y;

    for (int row = 0; row <= 1; row++)
    {
      for (int col = 0; col <= 1; col++)
      {
        cv::Vec3b c = rgb.at<cv::Vec3b>(cv::Point(x + col, y + row));
        for (int i = 0; i < 3; i++)
        {
          color_i.val[i] += c.val[i];
        }
      }
    }

    cv::Vec3b color;
    for (int i = 0; i < 3; i++)
    {
      color.val[i] = color_i.val[i] / 4;
    }
    return color;
  }

  cv::Size size()
  {
    return img.size();
  }

  uchar at(cv::Point xy)
  {
    return img.at<uchar>(xy);
  }

  // CV_32SC1 Mat with segment indexes
  cv::Mat segmentation(int segments);

protected:
  cv::Mat img;

  static float const gamma;
  static float const alpha;
  static cv::Mat distance_weights;
};
/* CLASS Image */

} /* NAMESPACE Image*/
;

};

#endif /* IMAGE_H_ */
