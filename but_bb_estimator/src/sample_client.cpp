/******************************************************************************
 * \file
 *
 * $Id: bb_estimator_client.cpp 742 2012-04-25 15:18:28Z spanel $
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

/**
 * Description:
 * This client demonstrates the function of service "/bb_estimator/estimate_bb"
 * performing bounding box estimation.
 *
 * There are two variants of subscription - to be the service able to work also
 * with a simulation of Care-O-bot (subscription variant #2 is meant to be used
 * with simulation, which does not produce a depth map so it must be created from
 * a point cloud).
 *
 * Required topics:
 *  Subscription variant #1:
 *   rgb_image_in - topic with Image messages containing RGB information
 *   depth_image_in - topic with Image messages containing depth information
 *   camera_info_in - topic with CameraInfo messages
 *  Subscription variant #2:
 *   rgb_image_in - topic with Image messages containing RGB information
 *   points_in - topic with PointCloud2 messages containing Point Cloud
 *   camera_info_in - topic with CameraInfo messages
 *
 * Node parameters:
 *  global_frame - Frame Id in which the BB coordinates are returned
 *
 * Manual:
 * - Use your mouse to specify a region of interest (ROI) in the window with
 *   input video. You will get another window with the visualization of the
 *   resulting bounding box.
 * - Press "D" key to switch between display modes (RGB or DEPTH data is displayed)
 * - Press "E" key to switch between estimation modes:
 *      MODE1 = The ROI corresponds to projection of BB front face and the BB is 
 *         rotated to fit the viewing frustum (representing the back-projection
 *         of the ROI) in such way, that the BB front face is perpendicular
 *         to the frustum's center axis.
 *         (BB can be non-parallel with all axis.)
 *
 *      MODE2 = In the ROI is contained the whole projection of BB.
 *         (BB is parallel with all axis.)
 *
 *      MODE3 = The ROI corresponds to projection of BB front face.
 *         (BB is parallel with all axis.)
 *------------------------------------------------------------------------------
 */


#include <but_bb_estimator/bb_estimator.h>
#include <but_bb_estimator/funcs.h>


// Definition of the service for BB estimation
#include "but_bb_estimator/EstimateBB.h"
#include "but_bb_estimator/EstimateBBAlt.h"
#include "but_bb_estimator/EstimateRect.h"
#include "but_bb_estimator/EstimateRectAlt.h"
//#include "but_bb_estimator/Estimate2DHullMesh.h"

#include <but_interaction_primitives/AddBoundingBox.h>
#include <but_interaction_primitives/ChangePose.h>
#include <but_interaction_primitives/ChangeScale.h>
#include <but_interaction_primitives/PoseType.h>

#include <algorithm>
#include <cstdlib>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

//#include <arm_navigation_msgs/Shape.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


namespace but_bb_estimator
{

// Cache
message_filters::Cache<Image> depthCache; // Cache of Image messages
message_filters::Cache<PointCloud2> pointCloudCache; // Cache of PointCloud2 messages
message_filters::Cache<CameraInfo> camInfoCache; // Cache of CameraInfo messages

// Frame IDs
std::string camFrameId; // Camera frame id (will be obtained automatically)
std::string sceneFrameId; // Scene (world) frame id

// Subscription variants
int subVariant = SV_NONE;

// TF listener
tf::TransformListener *tfListener;

// Percentage of farthest points from mean considered as outliers when
// calculating statistics of ROI
int outliersPercent = OUTLIERS_PERCENT_DEFAULT;

// The required maximum ratio of sides length (the longer side is at maximum
// sidesRatio times longer than the shorter one)
double sidesRatio = SIDES_RATIO_DEFAULT;

// Modes of bounding box estimation
int estimationMode = 1;

bool isWaitingForResponse = false; // Client is waiting for response from server
bool isBBCalculated = false; // Indicates, if there is any BB already calculated
bool isBBPrimitiveCreated = false; // Indicates, if there is any BB primitive already created
                                   // (for visualization using interactive markers)
Point2i mouseDown; // Position of mouse click
Point2i roiP1, roiP2; // Two diagonally opposite corners of ROI
vector<Point3f> bbVertices(8); // Corners of the estimated BB
geometry_msgs::Pose bbPose; // Pose of the estimated BB
geometry_msgs::Vector3 bbScale; // Scale of the estimated BB
vector<Point2i> rectPoints(2); // Estimated rectangle after backward projection

// Names of windows
string inputVideoWinName("Input Video (use your mouse to specify a ROI)");
string bbWinName("Bounding Box");
string hullWinName("Convex Hull");

// Bounding box colors
Scalar bbColorFront(0, 0, 255); // Color of the front side edges of the box
Scalar bbColor(255, 0, 0); // Color of the rest of the edges
Scalar rectColor(0, 255, 255); // Color of the rectangle

// Transformation from world to camera coordinates
tf::StampedTransform worldToSensorTf;

// Display mode enumeration
enum displayModesEnum {RGB, DEPTH};
int displayMode = RGB;

// Clients for communication with bb_estimator_server
ros::ServiceClient bbEstimateClient, rectEstimateClient, Hull2DMeshEstimateClient;

// Client for the add_bounding_box, change_pose and change_scale service
ros::ServiceClient bbAddClient, bbChangePoseClient, bbChangeScaleClient;

// Current and request images
// currentX = the last received image of type X
// requestX = the image of type X, which was current in the time of request
// (currentImage and requestImage are references to corresponding Rgb or Depth
// images - according to the selected display mode)
Mat currentImage, requestImage;
Mat currentRgb, requestRgb, currentDepth, requestDepth;
Mat currentCamK, requestCamK;


/*==============================================================================
 * Visualization of bounding box in a special window.
 */
void visualizeInImage()
{    
    // If the image does not have 3 channels => convert it (we want to visualize
    // the bounding box in color, thus we need 3 channels)
    Mat img3ch;
    if(requestImage.channels() != 3) {
        cvtColor(requestImage, img3ch, CV_GRAY2RGB, 3);
    }
    else {
        requestImage.copyTo(img3ch);
    }
    
    // Transform the resulting BB vertices (in world coordinates)
    // to camera coordinates
    //--------------------------------------------------------------------------
    vector<Point3f> bbVerticesCam(8); // Corners of estimated BB in camera coords
    
    tf::Transformer t;
    t.setTransform(worldToSensorTf);
    
    for(int i = 0; i < (int)bbVertices.size(); i++) {
    
        tf::Stamped<tf::Point> vertex;
        vertex.frame_id_ = sceneFrameId;
    
        vertex.setX(bbVertices[i].x);
        vertex.setY(bbVertices[i].y);
        vertex.setZ(bbVertices[i].z);
        t.transformPoint(camFrameId, vertex, vertex);
        bbVerticesCam[i].x = vertex.getX();
        bbVerticesCam[i].y = vertex.getY();
        bbVerticesCam[i].z = vertex.getZ();
    }
    
    // Perspective projection of the bounding box on the image plane:
    // world (camera) coords (3D) => image coords (2D)
    //
    // The world coordinate system is identical with the camera coordinate system
    // => Tx = Ty = 0 and R is identity => P[1:3,1:3] = K, where P is the camera
    // matrix and K is the intrinsic camera matrix.
    // (http://www.ros.org/wiki/image_pipeline/CameraInfo)
    //
    // The lines below implement these formulas:
    // x = X / Z * f_x + c_x
    // y = Y / Z * f_y + c_y
    // where [X,Y,Z] is a 3D point and [x,y] its perspective projection.
    //--------------------------------------------------------------------------
    double bb3DVerticesArray[8][3];
    for(int i = 0; i < (int)bbVertices.size(); i++) {
        bb3DVerticesArray[i][0] = bbVerticesCam[i].x / (double)bbVerticesCam[i].z;
        bb3DVerticesArray[i][1] = bbVerticesCam[i].y / (double)bbVerticesCam[i].z;
        bb3DVerticesArray[i][2] = 1;
    }
    Mat bb3DVertices = Mat(8, 3, CV_64F, bb3DVerticesArray);

    // requestCamK is the intrinsic camera matrix (containing f_x, f_y, c_x, c_y)
    Mat bb2DVertices = requestCamK * bb3DVertices.t();
    bb2DVertices = bb2DVertices.t();
    
    // Define edges of the bounding box
    //--------------------------------------------------------------------------
    // Edges are represented by connections between vertices calculated above
    int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5},
        {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}
    };
    
    // Draw the bounding box
    //--------------------------------------------------------------------------
    for(int i = 11; i >= 0; i--) {
        int j = edges[i][0];
        int k = edges[i][1];
        line(img3ch,
            Point((int)bb2DVertices.at<double>(j, 0), (int)bb2DVertices.at<double>(j, 1)),
            Point((int)bb2DVertices.at<double>(k, 0), (int)bb2DVertices.at<double>(k, 1)),
            (i < 4) ? bbColorFront : bbColor, 2);
    }

    // Draw the rectangle
    //--------------------------------------------------------------------------
    line(img3ch,
         Point(rectPoints[0].x, rectPoints[0].y),
         Point(rectPoints[0].x, rectPoints[1].y),
         rectColor, 1);
    line(img3ch,
         Point(rectPoints[0].x, rectPoints[1].y),
         Point(rectPoints[1].x, rectPoints[1].y),
         rectColor, 1);
    line(img3ch,
         Point(rectPoints[1].x, rectPoints[1].y),
         Point(rectPoints[1].x, rectPoints[0].y),
         rectColor, 1);
    line(img3ch,
         Point(rectPoints[1].x, rectPoints[0].y),
         Point(rectPoints[0].x, rectPoints[0].y),
         rectColor, 1);
    
    // Show the image with visualized bounding box
    imshow(bbWinName, img3ch);
}


/*==============================================================================
 * Visualization of bounding box using interactive markers.
 */
void visualizeWithMarkers()
{   
    std::string bbPrimitiveName("estimated_bb");

    // No BB primitive has been created yet => create it
    //--------------------------------------------------------------------------
    if(!isBBPrimitiveCreated) {

        // Set parameters to the new bounding box
        but_interaction_primitives::AddBoundingBox bbAddSrv;
        
        bbAddSrv.request.name = bbPrimitiveName;
        bbAddSrv.request.frame_id = sceneFrameId;
        bbAddSrv.request.description = "Bounding Box";
        
        bbAddSrv.request.pose = bbPose;
        bbAddSrv.request.scale = bbScale;
        bbAddSrv.request.pose_type = but_interaction_primitives::PoseType::POSE_CENTER;
        
        bbAddSrv.request.color.r = 1.0;
        bbAddSrv.request.color.g = 0.0;
        bbAddSrv.request.color.b = 1.0;
        bbAddSrv.request.color.a = 1.0;

        // Call service with specified parameters
        bbAddClient.call(bbAddSrv);
        
        isBBPrimitiveCreated = true;
    }
    
    // The BB primitive has already been created => modify it
    //--------------------------------------------------------------------------
    else {
        // Change pose of the BB
        //--------------------
        but_interaction_primitives::ChangePose bbChangePoseSrv;
        bbChangePoseSrv.request.name = bbPrimitiveName;
        bbChangePoseSrv.request.pose = bbPose;

        // Call service with specified parameters
        bbChangePoseClient.call(bbChangePoseSrv);
        
        // Change scale of the BB
        //--------------------
        but_interaction_primitives::ChangeScale bbChangeScaleSrv;
        bbChangeScaleSrv.request.name = bbPrimitiveName;
        bbChangeScaleSrv.request.scale = bbScale;

        // Call service with specified parameters
        bbChangeScaleClient.call(bbChangeScaleSrv);
    }
}


/*==============================================================================
 * Visualization of bounding box.
 */
void visualize()
{
    visualizeInImage();
    visualizeWithMarkers();
}


/*==============================================================================
 * Sets the images (current and request one) based on the selected display mode.
 */
void setImages()
{  
    // RGB mode
    if(displayMode == RGB) {
        currentImage = currentRgb;
        requestImage = requestRgb;
    }
    
    // DEPTH mode
    else {
        currentImage = currentDepth;
        requestImage = requestDepth;
    }
}


/*==============================================================================
 * Redraw windows according to the selected display mode.
 */
void redrawWindows()
{
    // Set the current and request images based on the selected display mode
    setImages();
    
    // Show the current image
    imshow(inputVideoWinName, currentImage);
    
    // Show the bounding box (if there is any calculated already)
    if(isBBCalculated) {
        visualize();
    }
}


/*==============================================================================
 * Send request for bounding box calculation.
 *
 * @param p1  A corner of the region of interest.
 * @param p1  A corner of the region of interest - diagonally opposite to p1.
 */
void sendRequest(Point2i p1, Point2i p2)
{
    // Return if the input points are empty
    if(p1 == Point2i() || p2 == Point2i()) {
        return;
    }

    isWaitingForResponse = true;

    // Create the request
    //--------------------------------------------------------------------------
    // Instantiate the autogenerated service class, and assign values into
    // its request member
    but_bb_estimator::EstimateBB srv;
    srv.request.p1[0] = p1.x;
    srv.request.p1[1] = p1.y;
    srv.request.p2[0] = p2.x;
    srv.request.p2[1] = p2.y;
    srv.request.mode = estimationMode;
    
    // When using simulated Clock time, now() returns time 0 until first message
    // has been received on /clock topic => wait for that.
    ros::Time reqTime;
    do {
        reqTime = ros::Time::now();
    } while(reqTime.sec == 0);
    srv.request.header.stamp = reqTime;
    
    // Obtain the corresponding transformation from world to camera coordinate system
    // (for later visualization in an image)
    //--------------------------------------------------------------------------
    try {
        tfListener->waitForTransform(sceneFrameId, camFrameId, reqTime, ros::Duration(0.2));
        tfListener->lookupTransform(sceneFrameId, camFrameId, reqTime, worldToSensorTf);
    }
    catch(tf::TransformException& ex) {
        string errorMsg = String("Transform error: ") + ex.what();
        ROS_ERROR("%s", errorMsg.c_str());
        return;
    }
    
    // Send request and obtain response (the bounding box coordinates)
    //--------------------------------------------------------------------------    
    // Call the service (calls are blocking, it will return once the call is done)
    if(bbEstimateClient.call(srv)) 
    {
        isWaitingForResponse = false;
        isBBCalculated = true;
        
        but_bb_estimator::EstimateBB::Response res = srv.response;
        
        // Save the response (in world coordinates)
        //----------------------------------------------------------------------
        bbVertices[0] = Point3f(res.p1[0], res.p1[1], res.p1[2]);
        bbVertices[1] = Point3f(res.p2[0], res.p2[1], res.p2[2]);
        bbVertices[2] = Point3f(res.p3[0], res.p3[1], res.p3[2]);
        bbVertices[3] = Point3f(res.p4[0], res.p4[1], res.p4[2]);
        bbVertices[4] = Point3f(res.p5[0], res.p5[1], res.p5[2]);
        bbVertices[5] = Point3f(res.p6[0], res.p6[1], res.p6[2]);
        bbVertices[6] = Point3f(res.p7[0], res.p7[1], res.p7[2]);
        bbVertices[7] = Point3f(res.p8[0], res.p8[1], res.p8[2]);
        
        bbPose = res.pose;
        bbScale = res.scale;

        // Create the request for backward projection to the image plane test
        //--------------------------------------------------------------------------
        // Instantiate the autogenerated service class, and assign values into
        // its request member
        but_bb_estimator::EstimateRect srv2;
        srv2.request.pose = bbPose;
        srv2.request.scale = bbScale;
        srv2.request.header.stamp = reqTime;

        // Call the service (calls are blocking, it will return once the call is done)
        if( rectEstimateClient.call(srv2) )
        {
            but_bb_estimator::EstimateRect::Response res2 = srv2.response;

            rectPoints[0] = Point2i(res2.p1[0], res2.p1[1]);
            rectPoints[1] = Point2i(res2.p2[0], res2.p2[1]);
        }
        else
        {
            rectPoints[0] = Point2i(0, 0);
            rectPoints[1] = Point2i(0, 0);
            std::string errMsg = "Failed to call estimation service.";
            ROS_ERROR("%s", errMsg.c_str());
        }
        
        // Show the resulting bounding box (and rectangle)
        visualize();
        
        // Log request
        //----------------------------------------------------------------------
        std::cout << "----------" << std::endl;
        ROS_INFO("ROI (request): [%ld, %ld], [%ld, %ld]",
                (long int)srv.request.p1[0], (long int)srv.request.p1[1],
                (long int)srv.request.p2[0], (long int)srv.request.p2[1]);
        
        // Log response
        //----------------------------------------------------------------------
        std::stringstream ss;
        ss << "Bounding box, mode = " << estimationMode << " (response):\n";
        for(int i = 0; i < (int)bbVertices.size(); i++) {
            ss << "[" << bbVertices[i].x << ","
                      << bbVertices[i].y << ","
                      << bbVertices[i].z << "] ";
            if(i == 3) ss << "\n";
        }
        ss << std::endl;
        ss << "Rectangle (response):\n";
        for(int i = 0; i < (int)rectPoints.size(); i++) {
            ss << "[" << rectPoints[i].x << ","
                      << rectPoints[i].y << "] ";
        }
        ss << std::endl;
        ROS_INFO("%s", ss.str().c_str());
    }
    else {
        isWaitingForResponse = false;
        std::string errMsg = "Failed to call estimation service.";
        ROS_ERROR("%s", errMsg.c_str());
    }
}

/*==============================================================================
 * Test estimation service for 2D convex hull (in image plane coordinates) of a mesh
 * (in world coordinates).
 */
/*void estimate2DHullMesh()
{
    // Create the request
    //--------------------------------------------------------------------------
    but_bb_estimator::Estimate2DHullMesh srv;
    
    // Create a test mesh (in camera coordinates)
    //--------------------------------------------------------------------------
    arm_navigation_msgs::Shape mC;
    mC.type = mC.MESH;
    mC.vertices.resize(4);
    mC.vertices[0].x = 0; mC.vertices[0].y = 0; mC.vertices[0].z = 1;
    mC.vertices[1].x = 0; mC.vertices[1].y = 0.2; mC.vertices[1].z = 1;
    mC.vertices[2].x = 0.2; mC.vertices[2].y = 0.2; mC.vertices[2].z = 1;
    mC.vertices[3].x = -0.2; mC.vertices[3].y = 0.3; mC.vertices[3].z = 2;
    
    // triangle k is defined by tre vertices located at indices
    // triangles[3k], triangles[3k+1], triangles[3k+2]
    mC.triangles.resize(6);
    mC.triangles[0] = 0; mC.triangles[1] = 1; mC.triangles[2] = 2;
    mC.triangles[3] = 1; mC.triangles[4] = 2; mC.triangles[5] = 3;
    
    // When using simulated Clock time, now() returns time 0 until first message
    // has been received on /clock topic => wait for that.
    ros::Time reqTime;
    do {
        reqTime = ros::Time::now();
    } while(reqTime.sec == 0);
    srv.request.header.stamp = reqTime;
    
    // Read the messages from cache (the latest ones before the request timestamp)
    //--------------------------------------------------------------------------
    sensor_msgs::CameraInfoConstPtr camInfo = camInfoCache.getElemBeforeTime(reqTime);
    if(camInfo == 0) {
        ROS_ERROR("No camera info was obtained before the request time.");
        return;
    }
    
    // Get the intrinsic camera parameters
    //--------------------------------------------------------------------------
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    float cx = (float)K.at<double>(0, 2);
    float cy = (float)K.at<double>(1, 2);
    float fx = (float)K.at<double>(0, 0);
    float fy = (float)K.at<double>(1, 1);
    //float f = (fx + fy) / 2.0;
    
    if(fx == 0 || fy == 0 || cx == 0 || cy == 0) {
        ROS_ERROR("Intrinsic camera parameters are undefined.");
    }
    
    // Obtain the corresponding transformation from world to camera coordinate system
    //--------------------------------------------------------------------------
    try {
        tfListener->waitForTransform(sceneFrameId, camFrameId, reqTime, ros::Duration(0.2));
        tfListener->lookupTransform(sceneFrameId, camFrameId, reqTime, worldToSensorTf);
    }
    catch(tf::TransformException& ex) {
        string errorMsg = String("Transform error: ") + ex.what();
        ROS_ERROR("%s", errorMsg.c_str());
        return;
    }
    
    // Transform the test mesh to world coordinates
    //--------------------------------------------------------------------------
    tf::Transformer t;
    t.setTransform(worldToSensorTf);
    arm_navigation_msgs::Shape mW;
    mW.type = mW.MESH;
    mW.vertices.resize(mC.vertices.size());
    for( int i = 0; i < (int)mC.vertices.size(); i++ )
    {
        // Transformation to the world coordinates
        tf::Stamped<tf::Point> vertex;
        vertex.frame_id_ = camFrameId;
        vertex.setX(mC.vertices[i].x);
        vertex.setY(mC.vertices[i].y);
        vertex.setZ(mC.vertices[i].z);
        t.transformPoint(sceneFrameId, vertex, vertex);
        
        mW.vertices[i].x = vertex.getX();
        mW.vertices[i].y = vertex.getY();
        mW.vertices[i].z = vertex.getZ();
    }
    srv.request.mesh = mW;
  
    // Call the service (calls are blocking, it will return once the call is done)
    //--------------------------------------------------------------------------
    if( Hull2DMeshEstimateClient.call(srv) )
    {
        but_bb_estimator::Estimate2DHullMesh::Response res = srv.response;
        
        // If the image does not have 3 channels => convert it (we want to visualize
        // the bounding box in color, thus we need 3 channels)
        Mat img3ch;
        if(currentRgb.channels() != 3) {
            cvtColor(currentRgb, img3ch, CV_GRAY2RGB, 3);
        }
        else {
            currentRgb.copyTo(img3ch);
        }
        
        // Draw the projected test mesh
        //----------------------------------------------------------------------
        // Project the vertices onto image plane
        vector<Point2i> trPoints(mC.vertices.size());
        for( int i = 0; i < (int)mC.vertices.size(); i++ ) {
            Point3f p(mC.vertices[i].x, mC.vertices[i].y, mC.vertices[i].z);
            trPoints[i] = Point2i(int(p.x * fx / p.z + cx + 0.5), int(p.y * fy / p.z + cy + 0.5));
            //trPoints[i] = fwdProject(Point3f(mC.vertices[i].x, mC.vertices[i].y, mC.vertices[i].z), fx, fy, cx, cy);
        }
        
        // Draw the triangles
        for( int i = 2; i < (int)mC.triangles.size(); i += 3 ) {
            line(img3ch,
                Point(trPoints[mC.triangles[i - 2]].x, trPoints[mC.triangles[i - 2]].y),
                Point(trPoints[mC.triangles[i - 1]].x, trPoints[mC.triangles[i - 1]].y), bbColor, 3);
            line(img3ch,
                Point(trPoints[mC.triangles[i - 1]].x, trPoints[mC.triangles[i - 1]].y),
                Point(trPoints[mC.triangles[i]].x, trPoints[mC.triangles[i]].y), bbColor, 3);
            line(img3ch,
                Point(trPoints[mC.triangles[i - 2]].x, trPoints[mC.triangles[i - 2]].y),
                Point(trPoints[mC.triangles[i]].x, trPoints[mC.triangles[i]].y), bbColor, 3);
        }
        
        // Draw the resulting convex hull
        //----------------------------------------------------------------------
        for(int i = 0; i < (int)res.convexHull.points.size(); i++) {
            int j = (i + 1) % res.convexHull.points.size();
            
            line(img3ch,
                Point(res.convexHull.points[i].x, res.convexHull.points[i].y),
                Point(res.convexHull.points[j].x, res.convexHull.points[j].y), bbColorFront, 1);
        }
        
        // Show the image with visualized bounding box
        imshow(hullWinName, img3ch);
    }
    else
    {
        std::string errMsg = "Failed to call service " + Estimate2DHullMesh_SRV + ".";
        ROS_ERROR("%s", errMsg.c_str());
    }
}*/


/*==============================================================================
 * Scales depth values (for visualization purposes).
 *
 * The provided depth is typically in CV_16SC1 datatype.
 * Because we do not expect to have any negative depth value, we
 * can convert the datatype to unsigned integers, which is sufficient
 * for visualization purposes + more OpenCV functions can work with this
 * datatype (e.g. cvtColor used in visualize function).
 * The depth values are scaled to fit the range <0, 255>.
 */
void scaleDepth()
{
    // Do not consider large ("infinity") values
    Mat infMask;
    if(subVariant == SV_1) {
        infMask = currentDepth > 20000; // Pixels
    }
    else if(subVariant == SV_2) {
        infMask = currentDepth > 20; // Meters
    }
    
    // Get mask of known values
    Mat knownMask = (infMask == 0);

    double maxDepth;
    minMaxLoc(currentDepth, NULL, &maxDepth, NULL, NULL, knownMask);
    currentDepth.convertTo(currentDepth, CV_8U, 255.0 / maxDepth);
    
    // Set color of "infinity" values
    currentDepth.setTo(255, infMask);
}

/*==============================================================================
 * Processes messages from synchronized subscription variant #1.
 *
 * @param rgb  Message with rgb image.
 * @param depth  Message with depth image.
 * @param camInfo  Message with camera information.
 */
void sv1_processSubMsgs(const sensor_msgs::ImageConstPtr &rgb,
                    const sensor_msgs::ImageConstPtr &depth,
                    const sensor_msgs::CameraInfoConstPtr &camInfo)
{
    if(subVariant == SV_NONE) {
        subVariant = SV_1;
        
        // Set size of cache
        camInfoCache.setCacheSize(CACHE_SIZE);
    }
    else if(subVariant != SV_1) {
        return;
    }
    
    camInfoCache.add(camInfo);

    // Get the intrinsic camera matrix of our camera
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    K.copyTo(currentCamK);
    
    // Save frame id
    camFrameId = camInfo->header.frame_id;

    // Get rgb and depth image from the messages
    //--------------------------------------------------------------------------
    try {
        currentRgb = cv_bridge::toCvCopy(rgb)->image;
        currentDepth = cv_bridge::toCvCopy(depth)->image;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Scales depth values (for visualization purposes).
    scaleDepth();
    
    // Set the current and request images based on the selected display mode
    setImages();
    
    // Show the current image
    //--------------------------------------------------------------------------
    imshow(inputVideoWinName, currentImage);
    
    if(DEBUG) {
        // Prints the timestamp of the received frame
        ROS_INFO("Received frame timestamp: %d.%d",
                 rgb->header.stamp.sec, rgb->header.stamp.nsec);
    }
}


/*==============================================================================
 * Processes messages from synchronized subscription variant #2.
 *
 * @param rgb  Message with rgb image.
 * @param pointCloud  Message with point cloud.
 * @param camInfo  Message with camera information.
 */
void sv2_processSubMsgs(const sensor_msgs::ImageConstPtr &rgb,
                    const sensor_msgs::PointCloud2ConstPtr &pointCloud,
                    const sensor_msgs::CameraInfoConstPtr &camInfo)
{
    if(subVariant == SV_NONE) {
        subVariant = SV_2;
        
        // Set size of cache
        camInfoCache.setCacheSize(CACHE_SIZE);
    }
    else if(subVariant != SV_2) {
        return;
    }
    
    camInfoCache.add(camInfo);

    // Get the intrinsic camera matrix of our camera
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    K.copyTo(currentCamK);
    
    // Save frame id
    camFrameId = camInfo->header.frame_id;

    // Convert the sensor_msgs/PointCloud2 data to depth map
    //--------------------------------------------------------------------------
    // At first convert to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*pointCloud, cloud);
    
    currentDepth = Mat(cloud.height, cloud.width, CV_32F);
    
    for(int y = 0; y < (int)cloud.height; y++) {
        for(int x = 0; x < (int)cloud.width; x++) {
            currentDepth.at<float>(y, x) = cloud.points[y * cloud.width + x].z;
        }
    }

    // Get rgb and depth image from the Image messages
    //--------------------------------------------------------------------------
    try {
        currentRgb = cv_bridge::toCvCopy(rgb)->image;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // The image / point cloud coming from COB is flipped around X and Y axis
    //flip(currentRgb, currentRgb, -1);
    //flip(currentDepth, currentDepth, -1);
    
    // Scales depth values (for visualization purposes).
    scaleDepth();
    
    // Set the current and request images based on the selected display mode
    setImages();
    
    // Show the current image
    //--------------------------------------------------------------------------
    imshow(inputVideoWinName, currentImage);
    
    if(DEBUG) {
        // Prints the timestamp of the received frame
        ROS_INFO("Received frame timestamp: %d.%d",
                 rgb->header.stamp.sec, rgb->header.stamp.nsec);
    }
}


/*==============================================================================
 * Mouse event handler.
 *
 * @param event  Mouse event to be handled.
 * @param x  x-coordinate of the event.
 * @param y  y-coordinate of the event.
 */
void onMouse(int event, int x, int y, int flags, void *param)
{
    switch(event) {
    
        // Mouse DOWN (start-point of the ROI)
        case CV_EVENT_LBUTTONDOWN:
            mouseDown.x = x;
            mouseDown.y = y;
            break;
            
        // Mouse UP (end-point of the ROI)
        case CV_EVENT_LBUTTONUP:
            // Save the current images as the request ones
            currentImage.copyTo(requestImage);
            currentRgb.copyTo(requestRgb);
            currentDepth.copyTo(requestDepth);
            currentCamK.copyTo(requestCamK);
            
            roiP1 = Point2i(mouseDown.x, mouseDown.y);
            roiP2 = Point2i(x, y);
  
            // Send request for bounding box calculation
            // (if there is no pending one already)
            if(!isWaitingForResponse) {
                sendRequest(roiP1, roiP2);
            }
            break;
    }
}

}

/*==============================================================================
 * Main function
 */
int main(int argc, char **argv)
{
	using namespace but_bb_estimator;

	// ROS initialization (the last argument is the name of the node)
    ros::init(argc, argv, "bb_estimator_client");
    
    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    
    // Create a client for the /bb_estimator/estimate_bb service
    bbEstimateClient = private_nh.serviceClient<but_bb_estimator::EstimateBB>("est_bb");

    // Create a client for the /bb_estimator/estimate_rect service
    rectEstimateClient = private_nh.serviceClient<but_bb_estimator::EstimateRect>("est_rect");
    
    // Create a client for the /estimate_2D_hull_mesh service
    //Hull2DMeshEstimateClient = n.serviceClient<but_bb_estimator::Estimate2DHullMesh>(Estimate2DHullMesh_SRV);
    
    // Create clients for the add_bounding_box, change_pose and change_scale services
    // (for manipulation with a visualization of BB)
    bbAddClient = private_nh.serviceClient<but_interaction_primitives::AddBoundingBox>("bb_add"); // TODO remap in launch file?
    bbChangePoseClient = private_nh.serviceClient<but_interaction_primitives::ChangePose>("bb_change_pose");
    bbChangeScaleClient = private_nh.serviceClient<but_interaction_primitives::ChangeScale>("bb_change_scale");
    
    // Create a TF listener
    tfListener = new tf::TransformListener();
    
    // Get private parameters from the parameter server
    // (the third parameter of function param is the default value)
    //--------------------------------------------------------------------------
    
    private_nh.param("global_frame", sceneFrameId, "/map");

    ROS_INFO_STREAM("global_frame" << " = " << sceneFrameId);

    // Subscription and synchronization of messages
    // TODO: Create the subscribers dynamically and unsubscribe the unused
    // subscription variant.
    //--------------------------------------------------------------------------
    // Subscription variant #1
    message_filters::Subscriber<Image> sv1_rgb_sub(private_nh, "rgb_in", 1);
    message_filters::Subscriber<Image> sv1_depth_sub(private_nh, "depth_in", 1);
    message_filters::Subscriber<CameraInfo> sv1_camInfo_sub(private_nh, "cam_info_in", 1);
        
    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> sv1_MySyncPolicy;
	Synchronizer<sv1_MySyncPolicy> sv1_sync(sv1_MySyncPolicy(QUEUE_SIZE), sv1_rgb_sub, sv1_depth_sub, sv1_camInfo_sub);
	sv1_sync.registerCallback(boost::bind(&sv1_processSubMsgs, _1, _2, _3));
	
	// Subscription variant #2
	message_filters::Subscriber<Image> sv2_rgb_sub(n, "rgb_in", 1);
    message_filters::Subscriber<PointCloud2> sv2_pointCloud_sub(n, "cloud_in", 1);
    message_filters::Subscriber<CameraInfo> sv2_camInfo_sub(n, "cam_info_in", 1);
    
    typedef sync_policies::ApproximateTime<Image, PointCloud2, CameraInfo> sv2_MySyncPolicy;
	Synchronizer<sv2_MySyncPolicy> sv2_sync(sv2_MySyncPolicy(QUEUE_SIZE), sv2_rgb_sub, sv2_pointCloud_sub, sv2_camInfo_sub);
	sv2_sync.registerCallback(boost::bind(&sv2_processSubMsgs, _1, _2, _3));

    // Create a window to show the incoming video and set its mouse event handler
    //--------------------------------------------------------------------------
    namedWindow(inputVideoWinName.c_str(), CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback(inputVideoWinName.c_str(), onMouse);

    ROS_INFO("Ready");

    // Enters a loop
    //--------------------------------------------------------------------------
    while(ros::ok()) {
        char key = waitKey(10); // Process window events (i.e. also mouse events)
        
        // If the key D was pressed -> change the display mode
        if(key == 'd' || key == 'D') {
            displayMode = (displayMode == RGB) ? DEPTH : RGB;
            redrawWindows(); // Redraw windows (with input video and bounding box)
        }
        // If the key E was pressed -> change the estimation mode
        else if(key == 'e' || key == 'E') {
            estimationMode = (estimationMode == 3) ? 1 : (estimationMode + 1);
            ROS_INFO("Estimation mode %d activated.", estimationMode);
            sendRequest(roiP1, roiP2);
        }
        // If the key M was pressed -> test estimation service for 2D convex hull of a mesh
        /*else if(key == 'm' || key == 'M') {
            estimate2DHullMesh();
        }*/
        
        ros::spinOnce(); // Call all the message callbacks waiting to be called
    }

    return 0;
}

