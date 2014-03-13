/******************************************************************************
 * \file
 *
 * $Id: bb_estimator_server.cpp 742 2012-04-25 15:18:28Z spanel $
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

#include <but_bb_estimator/funcs.h>

#include <algorithm>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>

#include <tf/transform_listener.h>

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


namespace but_bb_estimator
{

// Global variables
//------------------------------------------------------------------------------

// Cache
extern message_filters::Cache<Image> depthCache; // Cache of Image messages
extern message_filters::Cache<PointCloud2> pointCloudCache; // Cache of PointCloud2 messages
extern message_filters::Cache<CameraInfo> camInfoCache; // Cache of CameraInfo messages

// Frame IDs
extern std::string camFrameId; // Camera frame id (will be obtained automatically)
extern std::string sceneFrameId; // Scene (world) frame id

// Subscription variants
extern int subVariant;

// TF listener
extern tf::TransformListener *tfListener;

// Percentage of furthest points from mean considered as outliers when
// calculating statistics of ROI
extern int outliersPercent;

// Percentage of rows and columns considered for sampling (for calculation of statistics)
extern int samplingPercent;

// The required maximum ratio of sides length (the longer side is at maximum
// sidesRatio times longer than the shorter one)
extern double sidesRatio;

// Modes of bounding box estimation
extern int estimationMode;


/******************************************************************************
 * Back perspective projection of a 2D point with a known depth.
 */

Point3f backProject(Point2i p, float z, float fx, float fy)
{
    return Point3f((p.x * z) / fx, (p.y * z) / fy, z);
}


/******************************************************************************
 * Perspective projection of a 3D point to the image plane.
 */

Point2i fwdProject(Point3f p, float fx, float fy, float cx, float cy)
{
    return Point2i(int(p.x * fx / p.z + cx + 0.5), int(p.y * fy / p.z + cy + 0.5));
}


/******************************************************************************
 * Calculation of statistics (mean, standard deviation, min and max).
 */

bool calcStats(Mat &m, float *mean, float *stdDev)
{

    // Get the mask of known values (the unknown are represented by 0, the
    // known by 255)
    Mat negMask = m <= 0; // Negative values
    Mat infMask = m > 20; // "Infinite" values (more than 20 meters)
    Mat knownMask = ((negMask + infMask) == 0);
    
    // Sampling (to speed up the calculation we consider only pixels on a sampling
    // grid, whose density is given by the specified parameter samplingPercent,
    // which says how many percent of rows and columns should be considered)
    int rowSamples = knownMask.rows * (samplingPercent / 100.0);
    int columnSamples = knownMask.cols * (samplingPercent / 100.0);
    int samplingStepY = max(rowSamples, 1);
    int samplingStepX = max(columnSamples, 1);
    
    // Keep only pixels on a sampling grid
    for(int y = 0; y < knownMask.rows; y++) {
        if(y % samplingStepY != 0) {
            for(int x = 0; x < knownMask.cols; x++) {
                if(x % samplingStepX != 0) {
                    knownMask.at<uchar>(y, x) = 0;
                }
            }
        }
    }

    // Mean and standard deviation
    Scalar meanS, stdDevS;
    float meanValue, stdDevValue;
    meanStdDev(m, meanS, stdDevS, knownMask);
    meanValue = meanS[0];
    stdDevValue = stdDevS[0];
    
    // When all the knownMask elements are zeros, the function meanStdDev returns
    // roiMean == roiStdDev == 0
    if(meanValue == 0 && stdDevValue == 0) {
        ROS_WARN("No depth information available in the region of interest");
        return false;
    }

    // Number of known values and outliers to be ignored
    int knownCount = sum(knownMask * (1.0/255.0))[0];
    int outliersCount = (knownCount * outliersPercent) / 100;

    // Just for sure check if there will be some known values left after removal
    // of outliers.
    if((knownCount - outliersCount) > 0) {
    
        // Values of m relative to the mean value
        Mat mMeanRel = abs(m - meanValue);

        // Find and ignore the given percentage of outliers (the furthest points
        // from the mean value)
        //----------------------------------------------------------------------
        vector<Point3f> outliers; // A vector to store a given number of the furthest
                                  // points from the mean value (z of each Point3f
                                  // stores an outlier value)
        float currMin = 0; // Current min value included in the vector of outliers
        int currMinIndex = 0;
        
        // Go through all the samples and find outliersCount of the furthest points
        // from the mean value (outliers)
        for(int y = 0; y < mMeanRel.rows; y++) {
            for(int x = 0; x < mMeanRel.cols; x++) {
                if(knownMask.at<uchar>(y, x)
                   && (mMeanRel.at<float>(y, x) > currMin || (int)outliers.size() < outliersCount)) {
                   
                    Point3f outlier(x, y, mMeanRel.at<float>(y, x));
                    
                    // If the vector of outliers is not full yet
                    if((int)outliers.size() < outliersCount) {
                        outliers.push_back(outlier);
                    }
                    
                    // If the vector of outliers is full already, replace the outlier with minimum value
                    else {
                        outliers.erase(outliers.begin() + currMinIndex);
                        outliers.insert(outliers.begin() + currMinIndex, outlier);
                    }
                    
                    // Update the current min value
                    float min = 100;
                    int minIndex = 0;
                    for(unsigned int i = 0; i < outliers.size(); i++) {
                        if(outliers[i].z < min) {
                            min = outliers[i].z;
                            minIndex = i;
                        }
                    }
                    currMin = min;
                    currMinIndex = minIndex;
                }
            }
        }
        
        // Mask out all the outliers
        for(unsigned int i = 0; i < outliers.size(); i++) {
            knownMask.at<uchar>(outliers[i].y, outliers[i].x) = 0;
        }

        // Mean and standard deviation (now ignoring the outliers)
        meanStdDev(m, meanS, stdDevS, knownMask);
        meanValue = meanS[0];
        stdDevValue = stdDevS[0];
    }

    *mean = meanValue;
    *stdDev = stdDevValue;


    if(DEBUG) {
        // Min and max
        double min, max;
        minMaxLoc(m, &min, &max, 0, 0, knownMask);

        // Print the calculated statistics
        std::cout << "DEPTH STATISTICS "
                  << "- Mean: " << *mean << ", StdDev: " << *stdDev
                  << ", Min: " << min << ", Max: " << max << std::endl;
    }
    
    return true;
}


/******************************************************************************
 * Calculation of distances from origin to the BB front and back face vertices.
 */

bool calcNearAndFarFaceDistance(Mat &m, float fx, float fy, Point2i roiLB,
                                Point2i roiRT, float *d1, float *d2)
{
	// Get mean and standard deviation
	float mean, stdDev;
	if(!calcStats(m, &mean, &stdDev)) {
		return false;
	}

	// Distance from origin to the front and back face vertices
	*d1 = mean - stdDev;
	*d2 = mean + stdDev;

    // TODO: The following check can be done only after conversion the focal
    // length to the same units as the distances have (= meters)
    // ----------
	// Check if all BB vertices are in the front of the image plane (only the
	// vertices of the front face can be behind (= closer to origin),
	// so it is enough to check them). A vector from origin to the depth = f
	// in direction of each of the front face vertices is constructed.
	// If the length l of such vector is bigger than the distance d1
	// (= distance from origin to the front face vertices), it means that
	// the corresponding vertex is behind the image plane, thus set d1 to l
	// (the vertex is moved to depth = f). This is done for all front face
	// vertices...
	/*
    float f = (fx + fy) / 2.0;
	
	Point3f vecLBFf = backProject(roiLB, f, fx, fy);
	float vecLBFlen = norm(vecLBFf);
	if(vecLBFlen > *d1 || *d1 < 0) *d1 = vecLBFlen;

	Point3f vecRBFf = backProject(Point2i(roiRT.x, roiLB.y), f, fx, fy);
	float vecRBFlen = norm(vecRBFf);
	if(vecRBFlen > *d1) *d1 = vecRBFlen;

	Point3f vecRTFf = backProject(roiRT, f, fx, fy);
	float vecRTFlen = norm(vecRTFf);
	if(vecRTFlen > *d1) *d1 = vecRTFlen;

	Point3f vecLTFf = backProject(Point2i(roiLB.x, roiRT.y), f, fx, fy);
	float vecLTFlen = norm(vecLTFf);
	if(vecLTFlen > *d1) *d1 = vecLTFlen;
	*/
	
	return true;
}


/******************************************************************************
 * Calculation of depth of near and far face of BB (perpendicular with all axis).
 */

bool calcNearAndFarFaceDepth(Mat &m, float f, float *z1, float *z2)
{    
    // Get statistics of depth in ROI
    float mean, stdDev;
    if(!calcStats(m, &mean, &stdDev)) {
       return false;
    }
    
    // The near and far faces of BB are positioned in the same distance
    // (given by the depth standard deviation) from the depth mean.
    *z1 = mean - stdDev; // Depth of the near BB face
    *z2 = mean + stdDev; // Depth of the far BB face
    
    // TODO: The following check can be done only after conversion the focal
    // length to the same units as the depth values have (= meters)
    // ----------
    // There cannot be any object whose depth is smaller than the focal length
    // => if we have obtained such value, set the value to be equal to f.
    /*
    *z1 = (*z1 > f) ? *z1 : f;
    */
    
    return true;
}


/******************************************************************************
 * Bounding box estimation.
 */

bool estimateBB(const ros::Time& stamp,
                const point2_t& p1, const point2_t& p2, int mode,
                Point3f& bbLBF, Point3f& bbRBF,
                Point3f& bbRTF, Point3f& bbLTF,
                Point3f& bbLBB, Point3f& bbRBB,
                Point3f& bbRTB, Point3f& bbLTB
                )
{
    // Set estimation mode. If it is not specified in the request or the value
    // is not valid => set MODE1 (== 1) as default estimation mode.
    estimationMode = mode;
    if(estimationMode <= 0 || estimationMode > 3) {
        estimationMode = 1;
    }
    
    Mat depthMap;

    // Read the messages from cache (the latest ones before the request timestamp)
    //--------------------------------------------------------------------------
    sensor_msgs::CameraInfoConstPtr camInfo = camInfoCache.getElemBeforeTime(stamp);
    if(camInfo == 0) {
        ROS_ERROR("Cannot calculate the bounding box. "
                  "No frames were obtained before the request time.");
        return false;
    }
    
    // Subscription variant #1 - there is an Image message with a depth map
    //----------------------
    if(subVariant == SV_1) {
        sensor_msgs::ImageConstPtr depth = depthCache.getElemBeforeTime(stamp);

        // Get depth from the message
        try {
            cv_bridge::CvImagePtr cvDepth;
            cvDepth = cv_bridge::toCvCopy(depth);
            cvDepth->image.convertTo(depthMap, CV_32F);
        }
        catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return false;
        }
        
        // Convert the depth coming in milimeters to meters
        depthMap *= 1.0/1000.0;
    }
    
    // Subscription variant #2 - the depth map must be created from a point clound
    //----------------------
    else if(subVariant == SV_2) {
        sensor_msgs::PointCloud2ConstPtr pointCloud = pointCloudCache.getElemBeforeTime(stamp);
    
        // Convert the sensor_msgs/PointCloud2 data to depth map
        // At first convert to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (*pointCloud, cloud);
        
        depthMap = Mat(cloud.height, cloud.width, CV_32F);
        
        for(int y = 0; y < (int)cloud.height; y++) {
            for(int x = 0; x < (int)cloud.width; x++) {
                float z = cloud.points[y * cloud.width + x].z;
                if(cvIsNaN(z)) z = 0;
                depthMap.at<float>(y, x) = z;
            }
        }
    }
    else {
        ROS_ERROR("Unknown subscription variant!");
        return false;
    }
    
    // Obtain the corresponding transformation from camera to world coordinate system
    //--------------------------------------------------------------------------
    tf::StampedTransform sensorToWorldTf;
    camFrameId = camInfo->header.frame_id;
    try {
        tfListener->waitForTransform(camFrameId, sceneFrameId,
                                     camInfo->header.stamp, ros::Duration(2.0));
//                                   camInfo->header.stamp, ros::Duration(0.2));
        tfListener->lookupTransform(camFrameId, sceneFrameId,
                                    camInfo->header.stamp, sensorToWorldTf);
    }
    catch(tf::TransformException& ex) {
        string errorMsg = String("Transform error: ") + ex.what();
        ROS_ERROR("%s", errorMsg.c_str());
        return false;
    }
    
    // Width and height of the depth image
    int width = depthMap.cols;
    int height = depthMap.rows;

    // Get the coordinates of the specified ROI (Region of Interest)
    //--------------------------------------------------------------------------
    // Determine the left-bottom and the right-top ROI corner.
    // It is assumed that the origin (0,0) is in the top-left image corner.
    Point2i roiLBi; // Left-bottom corner of ROI (in image coordinates)
    Point2i roiRTi; // Right-top corner of ROI (in image coordinates)
    
    if(p1[0] < p2[0]) {roiLBi.x = p1[0]; roiRTi.x = p2[0];}
    else {roiLBi.x = p2[0]; roiRTi.x = p1[0];}
    
    if(p1[1] < p2[1]) {roiRTi.y = p1[1]; roiLBi.y = p2[1];}
    else {roiRTi.y = p2[1]; roiLBi.y = p1[1];}
    
    // Consider only the part of the ROI which is within the image
    roiLBi.x = min(max(roiLBi.x, 0), width);
    roiLBi.y = min(max(roiLBi.y, 0), height);
    roiRTi.x = min(max(roiRTi.x, 0), width);
    roiRTi.y = min(max(roiRTi.y, 0), height);
    
    // Get depth information in the ROI
    Mat roi = Mat(depthMap, Rect(roiLBi.x, roiRTi.y,
                                 roiRTi.x - roiLBi.x, roiLBi.y - roiRTi.y));

    // Get the intrinsic camera parameters (needed for back-projection)
    //--------------------------------------------------------------------------
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    float cx = (float)K.at<double>(0, 2);
    float cy = (float)K.at<double>(1, 2);
    float fx = (float)K.at<double>(0, 0);
    float fy = (float)K.at<double>(1, 1);
    float f = (fx + fy) / 2.0;
    
    if(fx == 0 || fy == 0 || cx == 0 || cy == 0) {
        ROS_ERROR("Intrinsic camera parameters are undefined.");
    }
    
    // Transform the corners of ROI from image coordinates to coordinates
    // with center in the middle of the image
    // (Notice that the Y axis is flipped! = going downwards)
    //--------------------------------------------------------------------------
    Point2i roiLB(roiLBi.x - cx, roiLBi.y - cy);
    Point2i roiRT(roiRTi.x - cx, roiRTi.y - cy);
    
    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
//    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;
    
    // Vector with pointers to the BB vertices (to be able to iterate over them)
    vector<Point3f *> bbVertices(8);
    bbVertices[0] = &bbLBF;
    bbVertices[1] = &bbRBF;
    bbVertices[2] = &bbRTF;
    bbVertices[3] = &bbLTF;
    bbVertices[4] = &bbLBB;
    bbVertices[5] = &bbRBB;
    bbVertices[6] = &bbRTB;
    bbVertices[7] = &bbLTB;
    
    // MODE #1
    ////////////////////////////////////////////////////////////////////////////
    if(estimationMode == MODE1) {

        // Convert depth to distance from origin (0,0,0) (= optical center).
        Mat roiD(roi.size(), CV_32F);
        for(int i = 0; i < roi.rows; i++) {
            for(int j = 0; j < roi.cols; j++) {
                float z = roi.at<float>(i, j);
                Point3f P = backProject(Point2i(roiLB.x + j, roiRT.y + i), z, fx, fy);
                
                // Get the distance from the origin
                roiD.at<float>(i, j) = norm(P);
            }
        }

        // Get distance from origin to the front and back face vertices
        float d1, d2;
        if(!calcNearAndFarFaceDistance(roiD, fx, fy, roiLB, roiRT, &d1, &d2)) {
            return false;
        }

        // Front-face vertices of BB
        // Each of them is obtained by construction of a vector from origin with
        // length = d1 in direction of the corresponding ROI corner.
        //----------------------------------------------------------------------
        Point3f vecLBF = backProject(roiLB, 1, fx, fy);
        bbLBF = vecLBF * (d1 / (float)norm(vecLBF));
        
        Point3f vecRBF = backProject(Point2i(roiRT.x, roiLB.y), 1, fx, fy);
        bbRBF = vecRBF * (d1 / (float)norm(vecRBF));
        
        Point3f vecRTF = backProject(roiRT, 1, fx, fy);
        bbRTF = vecRTF * (d1 / (float)norm(vecRTF));
        
        Point3f vecLTF = backProject(Point2i(roiLB.x, roiRT.y), 1, fx, fy);
        bbLTF = vecLTF * (d1 / (float)norm(vecLTF));
        
        // Back-face vertices of BB
        // These vertices are obtained by translation of the corresponding
        // front face vertices by vector vecFFtoBF, which is the vector from the
        // intersection of the viewing frustum axis (A) and the circle with radius
        // d1 to the intersection of A and the circle with radius d2.
        //----------------------------------------------------------------------
        // Calculate an unit vector pointing to the center of the front face of BB
        Point3f vecMid = bbLBF + bbRBF + bbRTF + bbLTF;
        vecMid *= 1.0 / (float)norm(vecMid);
        
        // Calculate a vector from the front face to the back face (perpendicular
        // to them). We know that this vector must have the same direction as
        // vecMid, we only don't know its magnitude. We can express the magnitude
        // (k) from this equation: d2 = |vecLBF + k*vecMid|, which is describing
        // the geometry of our problem. After expressing k we get a quadratic
        // equation: a*k^2 + b*k + c = 0, where:
        // a = vecMid.x*vecMid.x + vecMid.y*vecMid.y + vecMid.z*vecMid.z = 1
        // b = 2*(bbLBF.x*vecMid.x + bbLBF.y*vecMid.y + bbLBF.z*vecMid.z)
        // c = d1*d1 - d2*d2
        // Discriminant D is always positive, because c <= 0, since d2 >= d1, so
        // we know that the equation has at least one solution. We do not
        // want to flip the direction of vecMid, thus we can observe that we are
        // interested only in the positiove solution (the bigger one).
        float b = 2.0*(bbLBF.x*vecMid.x + bbLBF.y*vecMid.y + bbLBF.z*vecMid.z);
        float c = d1*d1 - d2*d2;
        float D = b*b - 4.0*c;
        float k = (-b + sqrt(D)) / 2.0;
        Point3f vecFFtoBF = k * vecMid;

        // Back-face vertices
        bbLBB = bbLBF + vecFFtoBF;
        bbRBB = bbRBF + vecFFtoBF;
        bbRTB = bbRTF + vecFFtoBF;
        bbLTB = bbLTF + vecFFtoBF;
    }
    
    // MODE #2
    ////////////////////////////////////////////////////////////////////////////
    // Only bbLBF and bbRTF are calculated in this mode. The rest of the BB
    // vertices is expressed using these points (it is possible because the
    // resulting BB is parallel with all axis in this mode).
    else if(estimationMode == MODE2) {
        
        // Get depth of near and far face of BB
        float z1, z2;
        if(!calcNearAndFarFaceDepth(roi, f, &z1, &z2)) {
            return false;
        }

        // If necessary, flip axis (this is done to unify the calculation in
        // different situations). There can appear these situations (#1 and #2
        // can appear simultaneously):
        // 1) The X coordinates of both specified ROI corners are positive
        // => we flip them to be negative (the resulting 3D points will be then
        // flipped back). We want that, because we want bbLBF to be on the border
        // of viewing frustum - calculation for this case is implemented.
        // 2) The Y coordinates of both specified ROI corners are negative
        // => we flip them to be positive (the resulting 3D points will be then
        // flipped back). We want that, because we want bbRTB to be on the border
        // of viewing frustum - calculation for this case is implemented.
        // 3) If one X (Y) coordinate is negative and one positive, we know
        // that the X (Y) coordinates of the back BB face projection won't
        // be outside the range given by the X (Y) coordinates of the front BB
        // face projection, thus the calculation is the same as in MODE #3 in
        // this situation.
        //-----------------------------
        // Copy the corners of ROI (because they may be modified to unify the
        // calculation)
        Point2i roiLBm = roiLB;
        Point2i roiRTm = roiRT;
        
        bool flippedX = false;
        bool flippedY = false;
        
        // If both corners of ROI have positive X-coordinate => flip X-axis.
        if(roiLBm.x > 0) {
            roiLBm.x = -roiRT.x;
            roiRTm.x = -roiLB.x;
            flippedX = true;
        }
        
        // If both corners of ROI have negative Y-coordinate => flip Y-axis.
        if(roiLBm.y < 0) {
            roiLBm.y = -roiRT.y;
            roiRTm.y = -roiLB.y;
            flippedY = true;
        }
        
        // X coordinates of BB vertices
        //-----------------------------
        if(roiLBm.x <= 0 && roiRTm.x >= 0) {
            bbLBF.x = (roiLBm.x * z1) / fx;
            bbRTB.x = (roiRTm.x * z1) / fx;
        }
        else {
            bbLBF.x = (roiLBm.x * z1) / fx;
            bbRTB.x = (roiRTm.x * z2) / fx;
        
            // Degenerate case #1 - the back-projection of the left ROI corner
            // is on the right of the back-projection of the right ROI corner
            // => set X-coord of both to the middle value.
            if(bbLBF.x > bbRTB.x) {
                float midX = (bbLBF.x + bbRTB.x) / 2.0;
                bbLBF.x = midX;
                bbRTB.x = midX;
                z1 = (midX * fx) / roiLBm.x;
                z2 = (midX * fx) / roiRTm.x;
            }
        }
        
        // Y coordinates of BB vertices
        //-----------------------------
        if(roiLBm.y >= 0 && roiRTm.y <= 0) {
            bbLBF.y = (roiLBm.y * z1) / fy;
            bbRTB.y = (roiRTm.y * z1) / fy;
        }
        else {
            bbLBF.y = (roiLBm.y * z1) / fy;
            bbRTB.y = (roiRTm.y * z2) / fy;
            
            // Degenerate case #2 - the back-projection of the bottom ROI corner
            // is on the top of the back-projection of the top ROI corner.
            // => set Y-coord of both to the middle value
            if(bbLBF.y < bbRTB.y) {
                float midY = (bbLBF.y + bbRTB.y) / 2.0;
                bbLBF.y = midY;
                bbRTB.y = midY;
                z1 = (midY * fy) / roiLBm.y;
                z2 = (midY * fy) / roiRTm.y;
                
                // Adjust X coordinates to the new depth
                if(roiLBm.x <= 0 && roiRTm.x >= 0) {
                    bbLBF.x = (roiLBm.x * z1) / fx;
                    bbRTB.x = (roiRTm.x * z1) / fx;
                }
                else {
                    bbLBF.x = (roiLBm.x * z1) / fx;
                    bbRTB.x = (roiRTm.x * z2) / fx;
                }
            }
        }
        
        // Z coordinates of BB vertices
        //-----------------------------
        bbLBF.z = z1;
        bbRTB.z = z2;
        
        // If any axis was flipped => flip back
        if(flippedX) {
            float bbLBFx = bbLBF.x;
            bbLBF.x = -bbRTB.x;
            bbRTB.x = -bbLBFx;
        }
        if(flippedY) {
            float bbLBFy = bbLBF.y;
            bbLBF.y = -bbRTB.y;
            bbRTB.y = -bbLBFy;
        }
        
        // Get the remaining vertices (in this mode the BB is perpendicular
        // with all axis, so we can use the coordinates which have already
        // been calculated).
        bbRBF = Point3f(bbRTB.x, bbLBF.y, bbLBF.z);
        bbRTF = Point3f(bbRTB.x, bbRTB.y, bbLBF.z);
        bbLTF = Point3f(bbLBF.x, bbRTB.y, bbLBF.z);
        bbLBB = Point3f(bbLBF.x, bbLBF.y, bbRTB.z);
        bbRBB = Point3f(bbRTB.x, bbLBF.y, bbRTB.z);
        bbLTB = Point3f(bbLBF.x, bbRTB.y, bbRTB.z);
    }
    
    // MODE #3
    ////////////////////////////////////////////////////////////////////////////
    // Only bbLBF and bbRTF are calculated in this mode. The rest of the BB
    // vertices is expressed using these points (it is possible because the
    // resulting BB is parallel with all axis in this mode).
    else if(estimationMode == MODE3) {
            
        // Get depth of near and far face of BB
        float z1, z2;
        if(!calcNearAndFarFaceDepth(roi, f, &z1, &z2)) {
            return false;
        }
    
        // Left-bottom-front vertex of BB
        bbLBF = Point3f((roiLB.x * z1) / fx, (roiLB.y * z1) / fy, z1);
        
        // Right-top-back vertex of BB
        // X and Y coordinates of the back projected point is obtained using
        // the near depth, but its Z coordinate is set to the further
        // depth (this is so to ensure perpendicularity of the bounding box).
        bbRTB = Point3f((roiRT.x * z1) / fx, (roiRT.y * z1) / fy, z2);
        
        // Get the remaining vertices (in this mode the BB is perpendicular
        // with all axis, so we can use the coordinates which have already
        // been calculated).
        bbRBF = Point3f(bbRTB.x, bbLBF.y, bbLBF.z);
        bbRTF = Point3f(bbRTB.x, bbRTB.y, bbLBF.z);
        bbLTF = Point3f(bbLBF.x, bbRTB.y, bbLBF.z);
        bbLBB = Point3f(bbLBF.x, bbLBF.y, bbRTB.z);
        bbRBB = Point3f(bbRTB.x, bbLBF.y, bbRTB.z);
        bbLTB = Point3f(bbLBF.x, bbRTB.y, bbRTB.z);
    }

    // Transform the resulting BB vertices to the world coordinates
    //--------------------------------------------------------------------------
    tf::Transformer t;
    t.setTransform(sensorToWorldTf);
    
    for(int i = 0; i < (int)bbVertices.size(); i++) {
        tf::Stamped<tf::Point> vertex;
        vertex.frame_id_ = camFrameId;
    
        vertex.setX(bbVertices[i]->x);
        vertex.setY(bbVertices[i]->y);
        vertex.setZ(bbVertices[i]->z);
        t.transformPoint(sceneFrameId, vertex, vertex);
        bbVertices[i]->x = vertex.getX();
        bbVertices[i]->y = vertex.getY();
        bbVertices[i]->z = vertex.getZ();
    }
    
    
    // The resulting BB cannot go under the floor (z = 0). If it does so, then try
    // to raise the bottom face of BB so the lowest BB corner is on the floor.
    // This process is feasible only if the whole BB top face is above the floor.
    //--------------------------------------------------------------------------
    Point3f lowestCorner(0, 0, 0);
    Point3f vec(0, 0, 0);
    bool feasible = true;
    bool isSensorFlipped = false; // Indicates if the camera sensor is flipped
                                  // (if the top face in the sensor coordinates is also
                                  // the top one in the world coordinates or not).
    
    // The top face in the sensor coordinates is also the top one in the world coordinates
    if(bbLBF.z < lowestCorner.z) {
        if(bbLTF.z <= 0) feasible = false;
        vec = bbLTF - bbLBF;
        lowestCorner = bbLBF;
    }
    if(bbRBF.z < lowestCorner.z && feasible) {
        if(bbRTF.z <= 0) feasible = false;
        vec = bbRTF - bbRBF;
        lowestCorner = bbRBF;
    }
    if(bbLBB.z < lowestCorner.z && feasible) {
        if(bbLTB.z <= 0) feasible = false;
        vec = bbLTB - bbLBB;
        lowestCorner = bbLBB;
    }
    if(bbRBB.z < lowestCorner.z && feasible) {
        if(bbRTB.z <= 0) feasible = false;
        vec = bbRTB - bbRBB;
        lowestCorner = bbRBB;
    }
    
    // The top face in the sensor coordinates is the bottom one in the world coordinates
    // (sensor is flipped)
    if(bbLTF.z < lowestCorner.z) {
        if(bbLBF.z <= 0) feasible = false;
        isSensorFlipped = true;
        vec = bbLBF - bbLTF;
        lowestCorner = bbLTF;
    }
    if(bbRTF.z < lowestCorner.z && feasible) {
        if(bbRBF.z <= 0) feasible = false;
        isSensorFlipped = true;
        vec = bbRBF - bbRTF;
        lowestCorner = bbRTF;
    }
    if(bbLTB.z < lowestCorner.z && feasible) {
        if(bbLBB.z <= 0) feasible = false;
        isSensorFlipped = true;
        vec = bbLBB - bbLTB;
        lowestCorner = bbLTB;
    }
    if(bbRTB.z < lowestCorner.z && feasible) {
        if(bbRBB.z <= 0) feasible = false;
        isSensorFlipped = true;
        vec = bbRBB - bbRTB;
        lowestCorner = bbRTB;
    }
    
    if(lowestCorner.z < 0) {
        if(!feasible) {
            return false;
        }
        else {
            // Compute a vector moving the bottom corners along the BB edges so the
            // lowest corner is on the floor.
            float scale = (-lowestCorner.z) / vec.z;
            vec = vec * scale;
            
            // Raise the bottom/top corners along the BB edges
            if(!isSensorFlipped) {
                bbLBF += vec;
                bbRBF += vec;
                bbLBB += vec;
                bbRBB += vec;
            }
            else {
                bbLTF += vec;
                bbRTF += vec;
                bbLTB += vec;
                bbRTB += vec;
            }
        }
    }
    

    return true;
}


/******************************************************************************
 * Bounding box pose estimation.
 */

bool estimateBBPose(const Point3f& bbLBF, const Point3f& bbRBF,
                    const Point3f& bbRTF, const Point3f& bbLTF,
                    const Point3f& bbLBB, const Point3f& bbRBB,
                    const Point3f& bbRTB, const Point3f& bbLTB,
                    Point3f& position,
                    tf::Quaternion& orientation,
                    Point3f& scale
                    )
{
    // Calculate also a representation of the BB given by position, orientation
    // and scale
    //--------------------------------------------------------------------------
    // Save vectors representing the 3 edges emanating from bbLBF. We will align
    // them to X, Y and Z axis by a rotation (roll, pitch, yaw), from which is
    // then created a quaternion representing the BB orientation. All permutations
    // from set of these three edges to set of X, Y and Z axis are considered. The
    // one with the smallest sum of roll + pitch + yaw (= rotation which yields
    // the BB to be axis-aligned) is selected - we want to describe the BB
    // orientation by minimal rotation.
    vector<Point3f> edges(3), edgesNorm(3);
    edges[0] = bbRBF - bbLBF;
    edges[1] = bbLTF - bbLBF;
    edges[2] = bbLBB - bbLBF;
    
    // Normalization of edge vectors
    for(int i = 0; i < 3; i++) edgesNorm[i] = edges[i] * (1.0 / norm(edges[i]));
    
    Point3f rotBest; // The best rotation yielding to axis-aligned BB (roll, pitch and yaw)
    float anglesMinSum = -1; // Minimal sum of roll, pitch and yaw
    Point3f bbSize; // Size (scale) of the BB, w.r.t. X, Y and Z axis

    // For each permutation (edges -> axis) the smallest rotation which yields
    // to the axis-aligned BB is obtained.
    // At first, an angle between an edge (the one which is currently selected
    // as the one to be aligned with X axis) and X-axis is found. The edges are
    // then rotated by this angle. Similar steps are then applied for Y and Z axis.
    // These angles then define the BB orientation.
    //
    // Here are detailed steps to calculation an angle of rotation around X axis
    // (the steps for Y and Z axis are analogous):
    // 1) Project edges (the ones to be aligned with Y and Z axis; the one to be
    //    aligned with X axis is not considered here, because its rotation around
    //    X axis won't get us any closer tbtQuaterniono the axis-aligned BB) onto YZ plane
    //    and normalize them.
    // 2) Angle = arcus cosinus of the dot product of a normalized
    //    projected edge and a unit vector describing the corresponding axis.
    // 3) The direction correspondence of the vectors is not important,
    //    so convert the angle to the range from 0 to PI/2
    //    (=> the normalized projected vector can have, after rotation
    //    by this angle, the opposite direction than the axis vector).
    // 4) Set the direction of rotation (note: rotation is going
    //    counter-clockwise around given axis, when the coordinate system is
    //    viewed as this axis is going the the eye).
    // 5) Take the smaller angle (from the angle aligning one edge to Y axis and
    //    the angle aligning the second edge to Z axis).
    for(int xi = 0; xi < 3; xi++) {
        for(int yi = 0; yi < 3; yi++) {
            if(xi == yi) continue;
            for(int zi = 0; zi < 3; zi++) {
                if(xi == zi || yi == zi) continue;
                
                Point3f rot; // Current rotation
                vector<Point3f> edgesTmp = edgesNorm; // Local copy of edges

                // Roll - rotation around X axis
                //----------
                // Step 1:
                Point3f edgeYyz(0, edgesTmp[yi].y, edgesTmp[yi].z);
                Point3f edgeZyz(0, edgesTmp[zi].y, edgesTmp[zi].z);
                edgeYyz *= (1.0 / norm(edgeYyz));
                edgeZyz *= (1.0 / norm(edgeZyz));
                // Step 2:
                float rollY = acos(edgeYyz.y);
                float rollZ = acos(edgeZyz.z);
                // Step 3:
                if(rollY > PI/2.0) rollY = -PI + rollY;
                if(rollZ > PI/2.0) rollZ = -PI + rollZ;
                // Step 4:
                if(edgeYyz.z < 0) rollY = -rollY;
                if(edgeZyz.y > 0) rollZ = -rollZ;
                // Step 5:
                rot.x = (fabs(rollY) < fabs(rollZ)) ? -rollY : -rollZ;

                // Rotate edges around X axis
                for(int m = 0; m < 3; m++) {
	                edgesTmp[m] = Point3f(
					    edgesTmp[m].x,
					    cos(rot.x)*edgesTmp[m].y - sin(rot.x)*edgesTmp[m].z,
			            sin(rot.x)*edgesTmp[m].y + cos(rot.x)*edgesTmp[m].z);
                }

                // Pitch - rotation around Y axis
                //----------
                // Step 1:
                // Project edges (the ones to be aligned with X and Z axis) onto XZ plane
                Point3f edgeXxz(edgesTmp[xi].x, 0, edgesTmp[xi].z);
                Point3f edgeZxz(edgesTmp[zi].x, 0, edgesTmp[zi].z);
                edgeXxz *= (1.0 / norm(edgeXxz));
                edgeZxz *= (1.0 / norm(edgeZxz));
                // Step 2:
                float pitchX = acos(edgeXxz.x);
                float pitchZ = acos(edgeZxz.z);
                // Step 3:
                if(pitchX > PI/2.0) pitchX = -PI + pitchX;
                if(pitchZ > PI/2.0) pitchZ = -PI + pitchZ;
                // Step 4:
                if(edgeXxz.z > 0) pitchX = -pitchX;
                if(edgeZxz.x < 0) pitchZ = -pitchZ;
                // Step 5:
                rot.y = (fabs(pitchX) < fabs(pitchZ)) ? -pitchX : -pitchZ;

                // Rotate edges around Y axis
                for(int m = 0; m < 3; m++) {
	                edgesTmp[m] = Point3f(
					    cos(rot.y)*edgesTmp[m].x + sin(rot.y)*edgesTmp[m].z,
					    edgesTmp[m].y,
					    -sin(rot.y)*edgesTmp[m].x + cos(rot.y)*edgesTmp[m].z);
                }

                // Yaw - rotation around Z axis
                //----------
                // Step 1:
                // Project edges (the ones to be aligned with X and Y axis) onto XY plane
                Point3f edgeXxy(edgesTmp[xi].x, edgesTmp[xi].y, 0);
                Point3f edgeYxy(edgesTmp[yi].x, edgesTmp[yi].y, 0);
                edgeXxy *= (1.0 / norm(edgeXxy));
                edgeYxy *= (1.0 / norm(edgeYxy));
                // Step 2:
                float yawX = acos(edgeXxy.x);
                float yawY = acos(edgeYxy.y);
                // Step 3:
                if(yawX > PI/2.0) yawX = -PI + yawX;
                if(yawY > PI/2.0) yawY = -PI + yawY;
                // Step 4:
                if(edgeXxy.y < 0) yawX = -yawX;
                if(edgeYxy.x > 0) yawY = -yawY;
                // Step 5:
                rot.z = (fabs(yawX) < fabs(yawY)) ? -yawX : -yawY;

                // Evaluate the actual rotation - the smaller sum of angles, the better
                //--------------------------------------------------------------
                float actualSum = fabs(rot.x) + fabs(rot.y) + fabs(rot.z);
                if(anglesMinSum == -1 || anglesMinSum > actualSum) {
	                anglesMinSum = actualSum;
	                rotBest = rot;

                    // Calculate size (scale) of tbtQuaternionhe BB, w.r.t. X, Y and Z axis
                    // (using the current permutation)
	                bbSize.x = norm(edges[xi]);
	                bbSize.y = norm(edges[yi]);
	                bbSize.z = norm(edges[zi]);
                }
            }
        }
    }
    
    // Create a quaternion representing the calculated rotation
    //--------------------------------------------------------------------------
    orientation = tf::createQuaternionFromRPY(-rotBest.x, -rotBest.y, -rotBest.z);
    
    // Get the center of mass of the BB (in world coordinates)
    //--------------------------------------------------------------------------
//    position = Point3f(0, 0, 0);
//    for(int i = 0; i < (int)bbVertices.size(); i++) {
//        position += *(bbVertices[i]);
//    }
    position = bbLBF;
    position += bbRBF;
    position += bbRTF;
    position += bbLTF;
    position += bbLBB;
    position += bbRBB;
    position += bbRTB;
    position += bbLTB;
    position *= 1.0/8.0;

    // Scale
    //--------------------------------------------------------------------------
    scale.x = bbSize.x;
    scale.y = bbSize.y;
    scale.z = bbSize.z;
    
    return true;
}


/******************************************************************************
 * Image rectangle estimation.
 */

bool estimateRect(const ros::Time& stamp,
                  const Point3f& bbLBF, const Point3f& bbRBF,
                  const Point3f& bbRTF, const Point3f& bbLTF,
                  const Point3f& bbLBB, const Point3f& bbRBB,
                  const Point3f& bbRTB, const Point3f& bbLTB,
                  point2_t& p1, point2_t& p2
                  )
{
    // Read the messages from cache (the latest ones before the request timestamp)
    //--------------------------------------------------------------------------
    sensor_msgs::CameraInfoConstPtr camInfo = camInfoCache.getElemBeforeTime(stamp);
    if(camInfo == 0) {
        ROS_ERROR("Cannot calculate the image rectangle. "
                  "No camera info was obtained before the request time.");
        return false;
    }
    
    // Obtain the corresponding transformation from camera to world coordinate system
    //--------------------------------------------------------------------------
    tf::StampedTransform worldToSensorTf;
    camFrameId = camInfo->header.frame_id;
    try {
        tfListener->waitForTransform(sceneFrameId, camFrameId,
                                     camInfo->header.stamp, ros::Duration(2.0));

        tfListener->lookupTransform(sceneFrameId, camFrameId,
                                    camInfo->header.stamp, worldToSensorTf);
    }
    catch(tf::TransformException& ex) {
        string errorMsg = String("Transform error: ") + ex.what();
        ROS_ERROR("%s", errorMsg.c_str());
        return false;
    }
    
    // Width and height of the image
    int width = camInfo->width;
    int height = camInfo->height;

    // Get the intrinsic camera parameters (needed for back-projection)
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

    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
//    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;
    
    // Vector with pointers to the BB vertices (to be able to iterate over them)
    vector<const Point3f *> bbVertices(8);
    bbVertices[0] = &bbLBF;
    bbVertices[1] = &bbRBF;
    bbVertices[2] = &bbRTF;
    bbVertices[3] = &bbLTF;
    bbVertices[4] = &bbLBB;
    bbVertices[5] = &bbRBB;
    bbVertices[6] = &bbRTB;
    bbVertices[7] = &bbLTB;

    // 2D points after projection to the image plane
    vector<Point2i> trVertices(8);

    // Transform the BB vertices to the image plane
    //--------------------------------------------------------------------------
    tf::Transformer t;
    t.setTransform(worldToSensorTf);
    
    for( int i = 0; i < (int)bbVertices.size(); i++ )
    {
        // Transformation to the camera coordinates
        tf::Stamped<tf::Point> vertex;
        vertex.frame_id_ = sceneFrameId;
        vertex.setX(bbVertices[i]->x);
        vertex.setY(bbVertices[i]->y);
        vertex.setZ(bbVertices[i]->z);
        t.transformPoint(camFrameId, vertex, vertex);
        
        // Projection to the image plane
        trVertices[i] = fwdProject(Point3f(vertex.getX(), vertex.getY(), vertex.getZ()), fx, fy, cx, cy);
    }

    // Get the coordinates of the ROI (Region of Interest)
    //--------------------------------------------------------------------------

    // Determine the left-bottom and the right-top ROI corner.
    // It is assumed that the origin (0,0) is in the top-left image corner.
    Point2i ap1(trVertices[0].x, trVertices[0].y);
    Point2i ap2(trVertices[0].x, trVertices[0].y);
    for( int i = 1; i < (int)bbVertices.size(); i++ )
    {
        ap1.x = min(trVertices[i].x, ap1.x);
        ap1.y = min(trVertices[i].y, ap1.y);
        ap2.x = max(trVertices[i].x, ap2.x);
        ap2.y = max(trVertices[i].y, ap2.y);
    }
    
    // Consider only the part of the ROI which is within the image
    p1[0] = min(max(ap1.x, 0), width);
    p1[1] = min(max(ap1.y, 0), height);
    p2[0] = min(max(ap2.x, 0), width);
    p2[1] = min(max(ap2.y, 0), height);

    return true;
}


/******************************************************************************
 * 2D convex hull estimation.
 */
bool estimate2DConvexHull(const ros::Time& stamp,
                          const std::vector<cv::Point3f> points,
                          std::vector<cv::Point2i> &convexHull
                         )
{
    // Read the messages from cache (the latest ones before the request timestamp)
    //--------------------------------------------------------------------------
    sensor_msgs::CameraInfoConstPtr camInfo = camInfoCache.getElemBeforeTime(stamp);
    if(camInfo == 0) {
        ROS_ERROR("Cannot calculate the image rectangle. "
                  "No camera info was obtained before the request time.");
        return false;
    }
    
    // Obtain the corresponding transformation from camera to world coordinate system
    //--------------------------------------------------------------------------
    tf::StampedTransform worldToSensorTf;
    camFrameId = camInfo->header.frame_id;
    try {
        tfListener->waitForTransform(sceneFrameId, camFrameId,
                                     camInfo->header.stamp, ros::Duration(2.0));

        tfListener->lookupTransform(sceneFrameId, camFrameId,
                                    camInfo->header.stamp, worldToSensorTf);
    }
    catch(tf::TransformException& ex) {
        string errorMsg = String("Transform error: ") + ex.what();
        ROS_ERROR("%s", errorMsg.c_str());
        return false;
    }

    // Get the intrinsic camera parameters (needed for back-projection)
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
    
    // Transform the BB vertices to the image plane
    //--------------------------------------------------------------------------
    tf::Transformer t;
    t.setTransform(worldToSensorTf);
    
    // 2D points after projection to the image plane
    vector<Point2i> trPoints(points.size());
    
    for( int i = 0; i < (int)points.size(); i++ )
    {
        // Transformation to the camera coordinates
        tf::Stamped<tf::Point> vertex;
        vertex.frame_id_ = sceneFrameId;
        vertex.setX(points[i].x);
        vertex.setY(points[i].y);
        vertex.setZ(points[i].z);
        t.transformPoint(camFrameId, vertex, vertex);
        
        // Projection to the image plane
        trPoints[i] = fwdProject(Point3f(vertex.getX(), vertex.getY(), vertex.getZ()), fx, fy, cx, cy);
    }

    // Get a convex hull
    cv::convexHull(trPoints, convexHull);
    
    return true;
}

}
