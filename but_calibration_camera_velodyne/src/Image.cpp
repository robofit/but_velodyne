/*
 * Image.cpp
 *
 *  Created on: 13.11.2013
 *      Author: ivelas
 */

#include "but_calibration_camera_velodyne/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <algorithm>

using namespace std;
using namespace cv;

namespace but_calibration_camera_velodyne
{

namespace Image
{

cv::Mat Image::distance_weights;

float const Image::gamma = 0.98;
float const Image::alpha = 0.33;

Image::Image(cv::Mat _img) :
    img(_img)
{

  // computing Lookup Table of weights for inverse distance transform
  int middle = MAX(img.cols, img.rows);
  int width = middle * 2 + 1;
  if (Image::distance_weights.cols != width)
  {
    Image::distance_weights = Mat(1, width, CV_32FC1);
    for (int i = 0; i <= middle; i++)
    {
      float weight = pow(gamma, i);
      Image::distance_weights.at<float>(middle + i) = weight;
      Image::distance_weights.at<float>(middle - i) = weight;
    }
//		cerr << "weights: " << distance_weights << endl << endl;
  }
}

// outputs grayscle CV_8UC1 matrix
Mat Image::computeEdgeImage()
{

  Mat grayImg;
  if (this->img.channels() > 1)
  {
    grayImg = Mat(this->img.size(), CV_8UC1);
    cvtColor(this->img, grayImg, CV_BGR2GRAY);
  }
  else
  {
    this->img.copyTo(grayImg);
  }

  Mat edges;
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;
  /// Gradient X
  Sobel(grayImg, grad_x, CV_16S, 1, 0, 3);
  convertScaleAbs(grad_x, abs_grad_x);

  /// Gradient Y
  Sobel(grayImg, grad_y, CV_16S, 0, 1, 3);
  convertScaleAbs(grad_y, abs_grad_y);

  /// Total Gradient (approximate)
  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);
  return edges;
}

Mat Image::computeIDTEdgeImage(Mat &edge_img)
{
  Mat edges;
  edge_img.convertTo(edges, CV_32FC1);

  Mat default_edges = edges;

  Mat weights = Mat::ones(edges.size(), CV_32FC1);
  Mat new_weights = Mat::ones(edges.size(), CV_32FC1);
  Mat new_edges = Mat::zeros(edges.size(), CV_32FC1);

  int weights_middle = distance_weights.cols / 2;
  for (int rows_cols = 0; rows_cols < 2; rows_cols++)
  {
    // first time through rows, second by cols (is transposed)
    int width = edges.cols;
    for (int row = 0; row < edges.rows; row++)
    {
      for (int x = 0; x < width; x++)
      {
        Mat sub_weights;
        min(distance_weights(Rect(weights_middle - x, 0, width, 1)), weights.row(row), sub_weights);

        Mat edges_row = edges.row(row);

        Mat sub_res = sub_weights.mul(edges_row);

        double max;
        Point max_loc;
        minMaxLoc(sub_res, NULL, &max, NULL, &max_loc);

        new_edges.at<float>(row, x) = max;
        new_weights.at<float>(row, x) = sub_weights.at<float>(max_loc.x);
      }
    }

    new_edges = new_edges.t();
    new_weights = new_weights.t();
    edges = new_edges;
    weights = new_weights;
  }

  Mat idt_edge_img = new_edges.mul(new_weights);

  addWeighted(default_edges, alpha, idt_edge_img, (1.0 - alpha), 0.0, idt_edge_img);

  normalize(idt_edge_img, idt_edge_img, 0.0, 1.0, NORM_MINMAX);

  return idt_edge_img;
}

Mat Image::computeIDTEdgeImage()
{
  Mat edge_img = this->computeEdgeImage();
  Mat grayscale_idt_edge_img;
  Mat idt_edge_img = this->computeIDTEdgeImage(edge_img).mul(cv::Scalar::all(255.0));
  idt_edge_img.convertTo(grayscale_idt_edge_img, CV_8UC1);
  return grayscale_idt_edge_img;
}

bool order_X(const Vec3f &p1, const Vec3f &p2)
{
  return p1.val[0] < p2.val[0];
}
bool order_Y(const Vec3f &p1, const Vec3f &p2)
{
  return p1.val[1] < p2.val[1];
}
bool Image::detect4Circles(float canny_thresh, float center_thresh, vector<Point2f> &centers, vector<float> &radiuses)
{
  vector<Vec3f> circles;
  radiuses.clear();
  centers.clear();

  Mat src_gray;
  this->img.copyTo(src_gray);

  for (int thresh = center_thresh; circles.size() < 4 && thresh > 10; thresh -= 5)
  {
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, canny_thresh, thresh, 0, 0);
  }

  if (circles.size() != 4)
  {
    return false;
  }

  sort(circles.begin(), circles.end(), order_Y);
  sort(circles.begin(), circles.begin() + 2, order_X);
  sort(circles.begin() + 2, circles.begin() + 4, order_X);

  for (size_t i = 0; i < circles.size(); i++)
  {
    centers.push_back(Point2f(circles[i][0], circles[i][1]));
    radiuses.push_back(cvRound(circles[i][2]));
  }

  /// Draw the circles detected
  /*
   Mat src_rgb;
   cvtColor(img, src_rgb, CV_GRAY2BGR );
   vector<Scalar> colors;
   colors.push_back(Scalar(255,0,0));
   colors.push_back(Scalar(0,255,0));
   colors.push_back(Scalar(0,0,255));
   colors.push_back(Scalar(255,255,255));
   for (size_t i = 0; i < circles.size(); i++) {
   centers.push_back(Point2f(circles[i][0], circles[i][1]));

   Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
   int radius = cvRound(circles[i][2]);
   // circle center
   circle(src_rgb, center, 3, Scalar(0, 255, 0), -1, 8, 0);
   // circle outline
   circle(src_rgb, center, radius, colors[i], 3, 8, 0);
   cerr << i+1 << ". circle S("<<center.x<<","<<center.y<<"); r="<<radius << endl;
   }
   namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
   imshow("Hough Circle Transform Demo", src_rgb);
   waitKey(0);
   */

  return true;
}

Mat Image::segmentation(int segment_count)
{
  Mat src_gray;
  if (img.channels() > 1)
  {
    cvtColor(img, src_gray, CV_BGR2GRAY);
  }
  else
  {
    src_gray = img;
  }
  src_gray = src_gray.reshape(src_gray.channels(), src_gray.rows * src_gray.cols); // to single row

  Mat segmentation;
  adaptiveThreshold(img, segmentation, 1, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 401, 2);
  return segmentation;
}

}/* NAMESPACE Image */

}
