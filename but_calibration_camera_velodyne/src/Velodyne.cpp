/*
 * Velodyne.cpp
 *
 *  Created on: 26.11.2013
 *      Author: ivelas
 */

#include <vector>
#include <cmath>

#include "but_calibration_camera_velodyne/Velodyne.h"
#include "but_calibration_camera_velodyne/Exceptions.h"
#include "but_calibration_camera_velodyne/Image.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/assert.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;

namespace but_calibration_camera_velodyne
{

Velodyne::Velodyne::Velodyne(PointCloud<Point> _point_cloud) :
    point_cloud(_point_cloud)
{
  getRings(); // range computation
}

Velodyne::Velodyne Velodyne::Velodyne::transform(float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
  Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
  PointCloud<Point> new_cloud;
  transformPointCloud(point_cloud, new_cloud, transf);
  return Velodyne(new_cloud);
}

Velodyne::Velodyne Velodyne::Velodyne::transform(vector<float> DoF)
{
  ROS_ASSERT(DoF.size() == 6);
  return transform(DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);
}

Mat Velodyne::Velodyne::project(Mat projection_matrix, Rect frame, PointCloud<Point> *visible_points)
{
  Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {

    // behind the camera
    if (pt->z < 0)
    {
      continue;
    }

    float intensity = pt->intensity;
    cv::Point xy = Velodyne::project(*pt, projection_matrix);
    if (xy.inside(frame))
    {
      if (visible_points != NULL)
      {
        visible_points->push_back(*pt);
      }

      //cv::circle(plane, xy, 3, intensity, -1);
      plane.at<float>(xy) = intensity;
    }
  }

  Mat plane_gray;
  cv::normalize(plane, plane_gray, 0, 255, NORM_MINMAX, CV_8UC1);
  dilate(plane_gray, plane_gray, Mat());
  //Image::Image plane_img(plane_gray);
  //return plane_img.computeIDTEdgeImage();

  return plane_gray;
}

Mat Velodyne::Velodyne::project(Mat projection_matrix, Rect frame, Mat image)
{
  Mat plane = this->project(projection_matrix, frame, NULL);
  //equalizeHist(plane, plane);
  //equalizeHist(image, image);

  ROS_ASSERT(frame.width == image.cols && frame.height == image.rows);
  Mat empty = Mat::zeros(frame.size(), CV_8UC1);

  Mat result_channel(frame.size(), CV_8UC3);
  Mat in[] = {image, empty, plane};
  int from_to[] = {0, 0, 1, 1, 2, 2};
  mixChannels(in, 3, &result_channel, 1, from_to, 3);
  return result_channel;
}

vector<vector<Velodyne::Point*> > Velodyne::Velodyne::getRings()
{
  vector<vector<Point*> > rings(Velodyne::Velodyne::RINGS_COUNT);
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    ROS_ASSERT(pt->ring < RINGS_COUNT);
    pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);

    rings[pt->ring].push_back(&(*pt));
  }
  return rings;
}

void Velodyne::Velodyne::intensityByRangeDiff()
{
  intensityByDiff(Processing::DISTORTIONS);
}
void Velodyne::Velodyne::intensityByIntensityDiff()
{
  intensityByDiff(Processing::INTENSITY_EDGES);
}

void Velodyne::Velodyne::intensityByDiff(Processing processing)
{
  vector<vector<Point*> > rings = this->getRings();

  for (vector<vector<Point*> >::iterator ring = rings.begin(); ring < rings.end(); ring++)
  {
    Point* prev, *succ;
    if (ring->empty())
    {
      continue;
    }
    float last_intensity = (*ring->begin())->intensity;
    float new_intensity;
    (*ring->begin())->intensity = 0;
    (*(ring->end() - 1))->intensity = 0;
    for (vector<Point*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++)
    {
      prev = *(pt - 1);
      succ = *(pt + 1);

      switch (processing)
      {
        case Processing::DISTORTIONS:
          (*pt)->intensity = MAX( MAX( prev->range-(*pt)->range, succ->range-(*pt)->range), 0) * 10;
          break;
        case Processing::INTENSITY_EDGES:
          new_intensity = MAX( MAX( last_intensity-(*pt)->intensity, succ->intensity-(*pt)->intensity), 0) * 10;
          last_intensity = (*pt)->intensity;
          (*pt)->intensity = new_intensity;
          break;
        case Processing::NONE:
          break;
        default:
          throw NotImplementedException("Velodyne processing unknown.");
      }
    }
  }
  normalizeIntensity(0.0, 1.0);
}

PointCloud<PointXYZ> *Velodyne::Velodyne::toPointsXYZ()
{
  PointCloud<PointXYZ> *new_cloud = new PointCloud<PointXYZ>();
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    new_cloud->push_back(PointXYZ(pt->x, pt->y, pt->z));
  }
  return new_cloud;
}

// all intensities to range min-max
void Velodyne::Velodyne::normalizeIntensity(float min, float max)
{
  float min_found = INFINITY;
  float max_found = -INFINITY;

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    max_found = MAX(max_found, pt->intensity);
    min_found = MIN(min_found, pt->intensity);
  }

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (max - min) + min;
//		cerr << pt->intensity << " ";
  }

//	cerr << endl;
}

Velodyne::Velodyne Velodyne::Velodyne::threshold(float thresh)
{
  PointCloud<Point> new_cloud;
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    if (pt->intensity > thresh)
    {
      new_cloud.push_back(*pt);
    }
  }
  return Velodyne(new_cloud);
}

void Velodyne::Velodyne::detectPlanes(cv::Mat projection)
{
  PointCloud<Point> visible_points;
  this->project(projection, Rect(0, 0, 640, 480), &visible_points);
  // ...
}

vector<Velodyne::Velodyne> Velodyne::Velodyne::depthSegmentation(int segment_counts)
{
  vector<Velodyne> segments(segment_counts);

  Mat ranges(point_cloud.size(), 1, CV_32FC1);
//	Mat indicies(point_cloud.size(), 1, CV_32SC1);
  for (int i = 0; i < point_cloud.size(); i++)
  {
    ranges.at<float>(i) = point_cloud[i].range;
  }
  //kmeans(ranges, segment_counts, indicies, TermCriteria(TermCriteria::MAX_ITER, 3, 0), 3, KMEANS_PP_CENTERS);

  Mat ranges_uchar;
  normalize(ranges, ranges_uchar, 0, 255, NORM_MINMAX, CV_8UC1);
  Mat indicies(point_cloud.size(), 1, CV_8UC1);
  cv::threshold(ranges_uchar, indicies, 0, 1, THRESH_BINARY + THRESH_OTSU);

  for (int i = 0; i < point_cloud.size(); i++)
  {
    segments[indicies.at<uchar>(i)].push_back(point_cloud[i]);
  }

  return segments;
}

PointCloud<PointXYZRGB> Velodyne::Velodyne::colour(cv::Mat frame_rgb, cv::Mat P)
{
  PointCloud<PointXYZRGB> color_cloud;
  for (PointCloud<Point>::iterator pt = this->point_cloud.begin(); pt < this->point_cloud.end(); pt++)
  {
    Point2f xy = Velodyne::Velodyne::projectf(*pt, P);

    Vec3b rgb = Image::Image::atf(frame_rgb, xy);
    PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
    pt_rgb.x = pt->x;
    pt_rgb.y = pt->y;
    pt_rgb.z = pt->z;

    color_cloud.push_back(pt_rgb);
  }
  return color_cloud;
}

}
