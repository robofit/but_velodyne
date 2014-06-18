/*
 * Similarity.h
 *
 *  Created on: 8.1.2014
 *      Author: ivelas
 */

#ifndef SIMILARITY_H_
#define SIMILARITY_H_

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <map>
#include <utility>
#include <cassert>
#include <string>
#include <sstream>

#include <ros/assert.h>

#include <but_calibration_camera_velodyne/Exceptions.h>
#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

namespace but_calibration_camera_velodyne {

class Similarity
{
public:
  Similarity(cv::Mat _X, cv::Mat _Y) :
      X(_X), Y(_Y), H_X(-1.0), H_Y(-1.0), H_XY(-1.0)
  {
    ROS_ASSERT(X.type() == CV_8UC1);
    ROS_ASSERT(Y.type() == CV_8UC1);
    ROS_ASSERT(X.size() == Y.size());
  }

  // cross-corelation
  float getCrossCorelation()
  {
    return sum(X.mul(Y))[0];
  }

  // mutual information
  float getMutualInformation()
  {
    if (H_X < 0)
    {
      computeEntropies();
    }
    return H_X + H_Y - H_XY;
  }

  // normalized mutual information
  float getNormalizedMutualInformation()
  {
    if (H_X < 0)
    {
      computeEntropies();
    }
    return (H_X + H_Y) / H_XY;
  }

  enum Criteria
  {
    MI, NMI, CC
  };

  static Criteria getCriteria(std::string s)
  {
    if (s == "MI")
      return MI;
    if (s == "NMI")
      return NMI;
    if (s == "CC")
      return CC;
    throw new NotImplementedException("Unknown criteria " + s);
  }

  float getSimilarity(Criteria crit)
  {
    switch (crit)
    {
      case MI:
        return getMutualInformation();
      case NMI:
        return getNormalizedMutualInformation();
      case CC:
        return getCrossCorelation();
      default:
        std::stringstream ss;
        ss << "Similarity criteria " << crit << ".";
        throw new NotImplementedException(ss.str());
    }
  }

  float static projectionError(cv::Mat &segmentation, std::vector<Velodyne::Velodyne> &segments, cv::Mat P,
                               bool verbose = false)
  {

    cv::Rect frame(cv::Point(0, 0), segmentation.size());
    int total_miss = 0;
    int total = 0;
    for (int i = 0; i < 2; i++)
    {
      int fire = 0;
      int miss = 0;
      for (::pcl::PointCloud<Velodyne::Point>::iterator pt = segments[i].begin(); pt < segments[i].end(); pt++)
      {
        cv::Point xy = Velodyne::Velodyne::project(*pt, P);

        if (pt->z > 0 && xy.inside(frame))
        {
          if (segmentation.at<uchar>(xy) == i)
          {
            fire++;
          }
          else
          {
            miss++;
          }
          total++;
        }
      }
      total_miss += miss;
      if (verbose)
      {
        cout << "segment: " << i << ";\t ok: " << fire << ";\t missed: " << miss << endl;
      }
    }

    return total_miss / (float)(segments[0].size() + segments[1].size());
  }

  float static projectionError(Image::Image img, Velodyne::Velodyne scan, cv::Mat P, bool verbose = false)
  {

    cv::Mat segmentation = img.segmentation(2);
    std::vector<Velodyne::Velodyne> segments = scan.depthSegmentation(2);
    return projectionError(segmentation, segments, P, verbose);
  }

  float static edgeSimilarity(Image::Image &img, Velodyne::Velodyne &scan, cv::Mat &P)
  {
    cv::Rect frame(cv::Point(0, 0), img.size());
    float CC = 0;
    for (::pcl::PointCloud<Velodyne::Point>::iterator pt = scan.begin(); pt < scan.end(); pt++)
    {
      cv::Point xy = Velodyne::Velodyne::project(*pt, P);

      if (pt->z > 0 && xy.inside(frame))
      {
        CC += img.at(xy) * pt->intensity;
      }
    }
    return CC;
  }

protected:
  void computeEntropies();

protected:
  cv::Mat X, Y; // compared images

  float H_X, H_Y; // entropy of image X, Y
  float H_XY; // joint entropy

  static const int INTENSITIES = 256;
};

};

#endif /* SIMILARITY_H_ */
