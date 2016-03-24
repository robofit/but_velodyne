/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Shohei Fujii (fujii.shohei@gmail.com)
 * Date: 03/16/2016
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
#include <but_velodyne_odom/collar_line_odom.h>

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <libgen.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <cv.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

namespace but_velodyne_odom {
    
CollarLineOdomNode::CollarLineOdomNode(ros::NodeHandle& private_nh) :
    cumulated_transformation_(Eigen::Matrix4f::Identity()), b_save_file_(false), msg_count_(0) {
    but_velodyne::CollarLinesRegistration::Parameters registration_parameters;
    but_velodyne::CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
    bool use_kalman = false;
    int linear_estimator = 3;
    //"How the value of line matching threshold is estimated (mean/median/... of line pairs distance). Possible values: MEDIAN_THRESHOLD|MEAN_THRESHOLD|NO_THRESHOLD")
    int distance_threshold;
    private_nh.param("matching_threshold", 
            distance_threshold, static_cast<int>(registration_parameters.distance_threshold));
    registration_parameters.distance_threshold = static_cast<but_velodyne::CollarLinesRegistration::Threshold>(distance_threshold);
    // "How the weights are assigned to the line matches - prefer vertical lines, close or treat matches as equal. Possible values: DISTANCE_WEIGHTS|VERTICAL_ANGLE_WEIGHTS|NO_WEIGHTS"
    int weighting;
    private_nh.param("line_weightning", 
            weighting, static_cast<int>(registration_parameters.weighting));
    registration_parameters.weighting = static_cast<but_velodyne::CollarLinesRegistration::Weights>(weighting);
    //"[Experimental] How many shift vectors (for SVD) are generated per line match - each is amended by small noise")
    private_nh.param("shifts_per_match", 
            registration_parameters.correnspPerLineMatch, registration_parameters.correnspPerLineMatch);
    //"[Experimental] Deviation of noise generated for shift vectors (see above)"
    private_nh.param("shifts_noise_sigma", 
            registration_parameters.lineCorrenspSigma, static_cast<float>(registration_parameters.lineCorrenspSigma));
    //"How many collar lines are generated per single polar bin"
    private_nh.param("lines_per_bin_generated", pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellGenerated);
    //"How many collar lines are preserved per single polar bin after filtering"
    private_nh.param("lines_per_bin_preserved", pipeline_parameters.linesPerCellPreserved, pipeline_parameters.linesPerCellPreserved);
    //"Discard part of the generated lines based on the vertical population of polar bin (on/off). Possible values: ANGLE_WITH_GROUND|NONE"
    int preservedFactorOfLinesBy;
    private_nh.param("lines_preserved_factor_by",
            preservedFactorOfLinesBy, static_cast<int>(pipeline_parameters.preservedFactorOfLinesBy));
    pipeline_parameters.preservedFactorOfLinesBy = static_cast<but_velodyne::LineCloud::PreservedFactorBy>(preservedFactorOfLinesBy);
    //"Minimal number of registration iterations (similar to ICP iterations)"
    private_nh.param("min_iterations", pipeline_parameters.minIterations, pipeline_parameters.minIterations);
    //"Maximal number of registration iterations"
    private_nh.param("max_iterations", pipeline_parameters.maxIterations, pipeline_parameters.maxIterations);
    //"Maximal time for registration [sec]"
    private_nh.param("max_time_for_registration", pipeline_parameters.maxTimeSpent, pipeline_parameters.maxTimeSpent);
    //"After how many iterations the cloud should be re-sampled by the new collar line segments"
    private_nh.param("iterations_per_sampling", pipeline_parameters.iterationsPerSampling, pipeline_parameters.iterationsPerSampling);
    //"Minimal error (average distance of line matches) causing termination of registration"
    private_nh.param("target_error",
            pipeline_parameters.targetError, static_cast<float>(pipeline_parameters.targetError));
    //"If standard deviation of error from last N=min_iterations iterations if below this value - registration is terminated"
    private_nh.param("significant_error_deviation",
            pipeline_parameters.significantErrorDeviation, static_cast<float>(pipeline_parameters.significantErrorDeviation));
    //"How many previous frames are used for registration (multi-view CLS-M approach described in the paper)")
    private_nh.param("history_size", pipeline_parameters.historySize, pipeline_parameters.historySize);
    //"Use last N frames for linear odometry prediction - can not be combined with kalman_estimator switch")
    private_nh.param("linear_estimator", linear_estimator, linear_estimator);
    //"Use Kalman filter instead of linear predictor for estimation of odometry")
    private_nh.param("kalman_estimator", use_kalman, use_kalman);

    private_nh.param("save_file", b_save_file_, b_save_file_);

    pcl_input_cloud_.reset(new but_velodyne::VelodynePointCloud());

    if(use_kalman) {
        if(linear_estimator > 0) {
            ROS_WARN("Unable to use both linear predictor and Kalman filter!");
        }
        estimator_.reset(new but_velodyne::KalmanMoveEstimator(1e-5, 1e-4, 1.0));
    } else {
        estimator_.reset(new but_velodyne::LinearMoveEstimator(linear_estimator));
    }
    std::string output_path =  boost::filesystem::current_path().string();
    std::string graph_filename = output_path + "/poses.graph";
    graph_file_.reset(new std::ofstream(graph_filename.c_str()));
    if (!graph_file_->is_open()) {
        ROS_WARN_STREAM("failed to open file " << graph_filename.c_str());
        ros::shutdown();
        //exit(1);
    } else {
        ROS_INFO_STREAM("opened file " << graph_filename.c_str());
    }
    registration_.reset(new but_velodyne::CollarLinesRegistrationPipeline(*estimator_, *graph_file_, pipeline_parameters, registration_parameters));
    cloud_sub_ = private_nh.subscribe("input/cloud", 5, &CollarLineOdomNode::msgCallback, this);
    odom_pub_ = private_nh.advertise<nav_msgs::Odometry>("output/odom", 1);
}

void CollarLineOdomNode::msgCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {
    cv::Mat covariance;
    pcl::fromROSMsg<velodyne_pointcloud::PointXYZIR>(*input_cloud, *pcl_input_cloud_);
    Eigen::Matrix4f t = registration_->runRegistration(*boost::dynamic_pointer_cast<but_velodyne::VelodynePointCloud>(pcl_input_cloud_), covariance);
    cumulated_transformation_ = cumulated_transformation_ * t;
    nav_msgs::Odometry odom_msg;
    odom_msg.header = input_cloud->header;
    Eigen::Affine3d t_aff(cumulated_transformation_.cast<double>());
    tf::poseEigenToMsg(t_aff, odom_msg.pose.pose);
    odom_pub_.publish(odom_msg);
    if (b_save_file_) {
        std::stringstream ss;
        ss << boost::filesystem::current_path().string() << "/points_" << std::setfill('0') << std::setw(8) << msg_count_ << ".pcd";
        pcl::io::savePCDFileBinary<velodyne_pointcloud::PointXYZIR>(ss.str(), *pcl_input_cloud_);
        std::ofstream ofs(std::string(boost::filesystem::current_path().string() + "/estimated.poses").c_str(), std::ios::app);
        Eigen::Matrix4f::Scalar *pose = cumulated_transformation_.data();
        ofs       << pose[0] << " " << pose[4] << " " << pose[8]  << " " << pose[12] <<
              " " << pose[1] << " " << pose[5] << " " << pose[9]  << " " << pose[13] <<
              " " << pose[2] << " " << pose[6] << " " << pose[10] << " " << pose[14] << std::endl;
        msg_count_++;
    }
}

CollarLineOdomNode::~CollarLineOdomNode() {
}

} // namespace but_velodyne_odom

