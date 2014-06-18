/*
 * manual_calibration.cpp
 *
 *  Created on: 27.2.2014
 *      Author: ivelas
 */

#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;
string VELODYNE_COLOR_TOPIC;

ros::Publisher pub;
cv::Mat projection_matrix;

cv::Mat frame_rgb;
vector<float> DoF;

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg->P.begin(); i != msg->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;

  }

  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

  // if no rgb frame for coloring:
  if (frame_rgb.data == NULL)
  {
    return;
  }

  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg, pc);

  // x := x, y := -z, z := y,
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

  Image::Image img(frame_rgb);
  Velodyne::Velodyne transformed = pointcloud.transform(DoF);
  PointCloud<Velodyne::Point> visible_points;
  transformed.project(projection_matrix, Rect(0, 0, frame_rgb.cols, frame_rgb.rows), &visible_points);

  Velodyne::Velodyne visible_scan(visible_points);

  PointCloud<PointXYZRGB> color_cloud = visible_scan.colour(frame_rgb, projection_matrix);

  // reverse axix switching:
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  transformPointCloud(color_cloud, color_cloud, transf);

  sensor_msgs::PointCloud2 color_cloud2;
  toROSMsg(color_cloud, color_cloud2);
  color_cloud2.header = msg->header;

  pub.publish(color_cloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coloring_node");

  ros::NodeHandle n;
  n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_color_topic", VELODYNE_COLOR_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/6DoF", DoF);

  pub = n.advertise<sensor_msgs::PointCloud2>(VELODYNE_COLOR_TOPIC, 1);

  // Subscribe input camera image
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe(CAMERA_FRAME_TOPIC, 10, imageCallback);

  ros::Subscriber info_sub = n.subscribe(CAMERA_INFO_TOPIC, 10, cameraInfoCallback);

  ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>(VELODYNE_TOPIC, 1, pointCloudCallback);

  ros::spin();

  return EXIT_SUCCESS;
}
