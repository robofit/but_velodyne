#ifndef WAYPOINT_CORRECTOR_H
#define WAYPOINT_CORRECTOR_H

#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include "robotour_waypoint_corrector/getCorrectedWaypoint.h"

#define CV_VISUALIZE 1

namespace rt_road_detection
{

class WaypointCorrector
{
	public:
		WaypointCorrector(void);	//default constructor
		void init();

		std::string map_frame_, robot_frame_, map_topic_;
		double path_min_width_, path_max_width_, obstacle_bloat_;

		ros::NodeHandle nh; // NodeHandle is the main access point for communication with ROS system
		float _x, _y, _t;	//robot coordinates in meters
		float robo_x, robo_y, robo_t, norm_t;		//robot coordinates in pixels
		//origin point of robot
		cv::Point3f robot_origin;
		cv::Point2f robot_heading;
		cv::Point robot_origin2d;
		bool origin_set, got_map;
		//
		int width, height;	//dimensions of map
		float resolution, originX, originY; //map metadata

		//subscribers / publishers
		ros::Subscriber map_subscriber;
		//ros::Subscriber map_image_subscriber;
		ros::ServiceServer wp_service;

		//public methods
		void getCorrectedPose();
		std::vector<std::pair<cv::Point3f, cv::Point2f> > getSegments(float radius);
		std::vector<std::pair<cv::Point3f, cv::Point2f> > processSegments(std::vector<std::pair<cv::Point3f, cv::Point2f> > &segments, float min_width, float max_width, float angle_tolerance);
		bool correctWaypoint(cv::Point2i wp, cv::Point2f &result);
		bool serviceCallback(robotour_waypoint_corrector::getCorrectedWaypoint::Request &reqest,
				robotour_waypoint_corrector::getCorrectedWaypoint::Response &response);
		void getRoboPos();
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

	private:
		tf::TransformListener *tfl;
		geometry_msgs::Vector3 getAsEuler(geometry_msgs::Quaternion quat);
		geometry_msgs::Quaternion getAsQuaternion(geometry_msgs::Vector3 vec);
		float angle_difference(float angle1, float angle2);

		cv::Mat cv_map;			//local map of the environment
		cv::Point2f robot_position;	//position of robot in pixels
		cv::Point2f destination;	//destination point of robot
};

}

#endif // WAYPOINT_CORRECTOR_H
