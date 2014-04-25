/*
 * Velodyne.h
 *
 *  Created on: 26.11.2013
 *      Author: ivelas
 */

#ifndef VELODYNE_H_
#define VELODYNE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

#include "opencv2/opencv.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace but_calibration_camera_velodyne {

namespace Velodyne
{

typedef enum
{
  DISTORTIONS, INTENSITY_EDGES, NONE
} Processing;

// Euclidean Velodyne coordinate, including intensity, ring number and range information
struct Point
{
  PCL_ADD_POINT4D
  ; // quad-word XYZ
  float intensity; ///< laser intensity reading
  uint16_t ring; ///< laser ring number
  float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
}EIGEN_ALIGN16;

class Velodyne
{
public:
  Velodyne()
  {
  }
  Velodyne(::pcl::PointCloud<Point> point_cloud);
  Velodyne transform(float x, float y, float z, float rot_x, float rot_y, float rot_z);
  Velodyne transform(std::vector<float> DoF);

  static cv::Point2f projectf(const Point &pt, const cv::Mat &projection_matrix)
  {
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f; // is homogenious coords. the point's 4. coord is 1

    cv::Mat pt_2D = projection_matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;

    return cv::Point2f(x, y);
  }

  static cv::Point project(const Point &pt, const cv::Mat &projection_matrix)
  {

    cv::Point2f xy = projectf(pt, projection_matrix);
    return cv::Point(xy.x, xy.y);
  }

  cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, ::pcl::PointCloud<Point> *visible_points = NULL);
  cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, cv::Mat plane);
  void intensityByDiff(Processing processing);
  void intensityByRangeDiff();
  void intensityByIntensityDiff();
  std::vector<Velodyne> depthSegmentation(int segments);

  bool isEmpty()
  {
    return point_cloud.empty();
  }

  size_t size()
  {
    return point_cloud.size();
  }

  bool empty()
  {
    return point_cloud.empty();
  }

  void push_back(Point pt)
  {
    point_cloud.push_back(pt);
  }

  void save(std::string filename)
  {
    ::pcl::io::savePCDFile(filename, point_cloud);
  }

  ::pcl::PointCloud<Point>::iterator begin()
  {
    return point_cloud.begin();
  }

  ::pcl::PointCloud<Point>::iterator end()
  {
    return point_cloud.end();
  }

  ::pcl::PointCloud<Point> getPointCloud()
  {
    return point_cloud;
  }

  static void view(::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud_ptr)
  {
    boost::shared_ptr< ::pcl::visualization::PCLVisualizer > viewer(new ::pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud< ::pcl::PointXYZ >(cloud_ptr, "sample cloud");
    viewer->setPointCloudRenderingProperties( ::pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem(0.3);
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  static void view( ::pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
  {
    boost::shared_ptr< ::pcl::visualization::PCLVisualizer > viewer(
        new ::pcl::visualization::PCLVisualizer("Color 3D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);
    ::pcl::visualization::PointCloudColorHandlerRGBField< ::pcl::PointXYZRGB > rgb(cloud_ptr);

    viewer->addPointCloud< ::pcl::PointXYZRGB>(cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties( ::pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem(0.3);
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  ::pcl::PointCloud<pcl::PointXYZRGB> colour(cv::Mat frame_rgb, cv::Mat P);

  void detectPlanes(cv::Mat projection);
  Velodyne threshold(float thresh);
  void normalizeIntensity(float min = 0.0, float max = 1.0);
  ::pcl::PointCloud<pcl::PointXYZ> *toPointsXYZ();

  static const unsigned RINGS_COUNT = 32;
  std::vector<std::vector<Point*> > getRings();

protected:
  ::pcl::PointCloud<Point> point_cloud;
};

} /* NAMESPACE Velodyne */

} /* NAMESPACE but_calibration_camera_velodyne */

POINT_CLOUD_REGISTER_POINT_STRUCT(
    but_calibration_camera_velodyne::Velodyne::Point, (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (uint16_t, ring, ring))

#endif /* VELODYNE_H_ */
