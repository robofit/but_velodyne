/*
 * Calibration3DMarker.h
 *
 *  Created on: 2.4.2014
 *      Author: ivelas
 */

#ifndef CALIBRATION3DMARKER_H_
#define CALIBRATION3DMARKER_H_

#include <iostream>
#include <algorithm>
#include <cmath>

#include "opencv2/opencv.hpp"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Calibration.h>

namespace but_calibration_camera_velodyne
{

class Calibration3DMarker
{

public:
  Calibration3DMarker(cv::Mat _frame_gray, cv::Mat _P, ::pcl::PointCloud<Velodyne::Point> _pc, float _circ_distance,
                      float _radius);

  bool detectCirclesInImage(std::vector<cv::Point2f> &centers, std::vector<float> &radiuses);

  bool detectCirclesInPointCloud(std::vector<cv::Point3f> &centers, std::vector<float> &radiuses);

protected:
  template<typename PointT>
    void remove_inliers(const ::pcl::PointCloud<PointT> &cloud_in, std::vector<int> inliers_indices,
                        ::pcl::PointCloud<PointT> &cloud_out)
    {

      std::vector<int> outliers_indicies;
      for (size_t i = 0; i < cloud_in.size(); i++)
      {
        if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
        {
          outliers_indicies.push_back(i);
        }
      }
      ::pcl::copyPointCloud< ::pcl::PointXYZ >(cloud_in, outliers_indicies, cloud_out);
    }

  std::vector< ::pcl::PointXYZ > detect4spheres(::pcl::PointCloud< ::pcl::PointXYZ >::Ptr plane, std::vector<float> &radiuses);
  /*
   * Indexes of circles in marker:
   * 0 1
   * 2 3
   */
  void order4spheres(std::vector< ::pcl::PointXYZ > &spheres_centers);

  bool verify4spheres(const std::vector< ::pcl::PointXYZ > &spheres_centers, float straight_distance, float delta);
  /*
   * All points around the all found centers:
   * x x x
   * x   x
   * x x x
   */
  std::vector< ::pcl::PointXYZ > generate_possible_centers(const std::vector< ::pcl::PointXYZ > &spheres_centers,
                                                       float straight_distance);

  void generate_possible_points(::pcl::PointCloud< ::pcl::PointXYZ > &plane,
                                ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr detection_cloud,
                                const std::vector< ::pcl::PointXYZ > &possible_centers, float radius, float tolerance);

  std::vector< ::pcl::PointXYZ > refine4centers(std::vector< ::pcl::PointXYZ > centers,
                                            ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr detection_cloud);

protected:
  cv::Mat frame_gray, P;
  ::pcl::PointCloud<Velodyne::Point> pc;
  float circ_distance, radius;
  ::pcl::PointCloud< ::pcl::PointXYZ > plane;

  static const int CANNY_THRESH = 150;
  static const int CENTER_THRESH_DISTANCE = 80;
};

};

#endif /* CALIBRATION3DMARKER_H_ */
