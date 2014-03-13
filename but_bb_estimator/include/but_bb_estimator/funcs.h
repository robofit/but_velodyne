/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 25.4.2012 (version 6.0)
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

#pragma once
#ifndef BB_ESTIMATOR_FUNCS_H
#define BB_ESTIMATOR_FUNCS_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


namespace but_bb_estimator
{

/*
 * Global variables
 */
static const float PI = 3.1415926535;
const bool DEBUG = false; // If true, verbose outputs are written to console.
//const bool DEBUG = true; // If true, verbose outputs are written to console.

/**
 * Cache size
 */
const int CACHE_SIZE = 10;

/**
 * Size of waiting queue for messages to arrive and complete their "set"
 */
const int QUEUE_SIZE = 10;

/**
 * Subscription variants
 */
enum subVariantsEnum { SV_NONE = 0, SV_1, SV_2 };

/**
 * Modes of bounding box estimation
 * They differ in interpretation of the specified 2D region of interest (ROI).
 *
 * MODE1 = The ROI corresponds to projection of BB front face and the BB is
 *         rotated to fit the viewing frustum (representing the back-projection
 *         of the ROI) in such way, that the BB front face is perpendicular
 *         to the frustum's center axis.
 *         (BB can be non-parallel with all axis.)
 *
 * MODE2 = In the ROI is contained the whole projection of BB.
 *         (BB is parallel with all axis.)
 *
 * MODE3 = The ROI corresponds to projection of BB front face.
 *         (BB is parallel with all axis.)
 */
enum estimationModeEnum { MODE1 = 1, MODE2, MODE3 };

/**
 * Point in 2D
 */
typedef boost::array<int16_t, 2> point2_t;


/**
 * Back perspective projection of a 2D point with a known depth:
 * 2D point in image coords + known depth => 3D point in world (camera) coords
 * (Result of back projection of an image point is typically a line. However,
 * in our case we know also its depth (from the depth map) and thus we can
 * determine its location on that line and so get a single 3D point.)
 *
 * The world coordinate system is in our case identical with the camera
 * coordinate system => Tx = Ty = 0 and R is identity => P[1:3,1:3] = K,
 * where P is the camera matrix and K is the intrinsic camera matrix.
 * (http://www.ros.org/wiki/image_pipeline/CameraInfo)
 *
 * This function implements these formulas:
 * X = x * Z / f_x
 * Y = y * Z / f_y
 * where [x,y] is an image point (with (0,0) in the middle of the image)
 * and [X,Y,Z] its perspective back projection in the given depth.
 *
 * @param p  2D image point.
 * @param z  Depth of the back projected 3D point.
 * @param fx  Focal length w.r.t. axis X.
 * @param fy  Focal length w.r.t. axis Y.
 * @return  The perspective back projection in the given depth.
 */
cv::Point3f backProject(cv::Point2i p, float z, float fx, float fy);


/**
 * Perspective projection of a 3D point to the image plane.
 * Intrinsic camera matrix for the raw (distorted) images is used:
 *     [fx  0 cx]
 * K = [ 0 fy cy]
 *     [ 0  0  1]
 * [u v w] = K * [X Y Z]
 *       x = u / w = fx * X / Z + cx
 *       y = v / w = fy * Y / Z + cy
 * Projects 3D points in the camera coordinate frame to 2D pixel
 * coordinates using the focal lengths (fx, fy) and principal point
 * (cx, cy).
 */
cv::Point2i fwdProject(cv::Point3f p, float fx, float fy, float cx, float cy);


/**
 * Calculation of statistics (mean, standard deviation, min and max).
 *
 * @param m  The matrix with depth information from which the statistics will
 *           be calculated (it is assumed that the unknown values are represented
 *           by zero).
 * @param mean  The calculated mean of m.
 * @param stdDev  The calculated standard deviation of m.
 * @return  True if the statistics was calculated. False if the statistics could
 *          not be calculated, because there is no depth information available
 *          in the specified ROI.
 */
bool calcStats(cv::Mat &m, float *mean, float *stdDev);


/**
 * Calculation of distances from origin to the BB front and back face vertices.
 * (It is used in MODE1.)
 *
 * @param m  The matrix with distance information (distance from origin).
 * @param fx  Focal length w.r.t. X axis.
 * @param fy  Focal length w.r.t. Y axis.
 * @param roiLB  Left-bottom corner of ROI.
 * @param roiRT  Right-top corner of ROI.
 * @param d1  Caclulated distance from origin to the BB front face.
 * @param d2  Caclulated distance from origin to the BB back face.
 * @return  True if the distance values were calculated. False if not (due to
 *          missing depth information in function calcStats).
 */
bool calcNearAndFarFaceDistance(
		cv::Mat &m, float fx, float fy, cv::Point2i roiLB,
		cv::Point2i roiRT, float *d1, float *d2
		);


/**
 * Calculation of depth of near and far face of BB (perpendicular with all axis).
 * (It is used in MODE2 and MODE3.)
 *
 * @param m  The matrix with depth information.
 * @param f  Focal length.
 * @param z1  Caclulated depth of the near BB face.
 * @param z2  Caclulated depth of the far BB face.
 * @return  True if the depth values were calculated. False if not (due to
 *          missing depth information in function calcStats).
 */
bool calcNearAndFarFaceDepth(cv::Mat &m, float f, float *z1, float *z2);


/**
 * Bounding box estimation.
 *
 * @param stamp     time stamp obtained from message header.
 * @param p1,p2     input 2D rectangle.
 * @param mode      estimation mode.
 * @param bbXYZ     resulting bounding box corners.
 */
bool estimateBB(const ros::Time& stamp,
                const point2_t& p1, const point2_t& p2,
                int mode,
                cv::Point3f& bbLBF, cv::Point3f& bbRBF,
                cv::Point3f& bbRTF, cv::Point3f& bbLTF,
                cv::Point3f& bbLBB, cv::Point3f& bbRBB,
                cv::Point3f& bbRTB, cv::Point3f& bbLTB
                );


/**
 * Bounding box pose estimation.
 *
 * @param bbXYZ         input bounding box corners.
 * @param position      calculated BB position (i.e. position of its center).
 * @param orientation   calculated BB orientation (i.e. quaternion).
 * @param scale         BB dimensions.
 */
bool estimateBBPose(const cv::Point3f& bbLBF, const cv::Point3f& bbRBF,
                    const cv::Point3f& bbRTF, const cv::Point3f& bbLTF,
                    const cv::Point3f& bbLBB, const cv::Point3f& bbRBB,
                    const cv::Point3f& bbRTB, const cv::Point3f& bbLTB,
                    cv::Point3f& position,
                    tf::Quaternion& orientation,
                    cv::Point3f& scale
                    );


/**
 * Image rectangle estimation.
 *
 * @param stamp     time stamp obtained from message header.
 * @param bbXYZ     input bounding box corners.
 * @param p1,p2     resulting 2D image rectangle.
 */
bool estimateRect(const ros::Time& stamp,
                  const cv::Point3f& bbLBF, const cv::Point3f& bbRBF,
                  const cv::Point3f& bbRTF, const cv::Point3f& bbLTF,
                  const cv::Point3f& bbLBB, const cv::Point3f& bbRBB,
                  const cv::Point3f& bbRTB, const cv::Point3f& bbLTB,
                  point2_t& p1, point2_t& p2
                  );

/**
 * 2D convex hull estimation.
 *
 * @param stamp       time stamp obtained from message header.
 * @param points      input 3D points (in world coordinates).
 * @param convexHull  resulting 2D convex hull (in image plane coordinates) of the input points.
 */
bool estimate2DConvexHull(const ros::Time& stamp,
                          const std::vector<cv::Point3f> points,
                          std::vector<cv::Point2i> &convexHull
                         );

}

#endif // BB_ESTIMATOR_FUNCS_H

