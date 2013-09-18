#include "rt_road_detection/WaypointCorrector.h"

#define OBSTACLE_THRESH 10
#define UNKNOWN_THRESH 100

using namespace cv;
using namespace std;

namespace rt_road_detection
{

	WaypointCorrector::WaypointCorrector(void)
	{
		init();
	}

	void WaypointCorrector::init()
	{
		//load parameters
		//if(!nh.getParam("/but_aligner/map_topic", map_topic))
		//		map_topic = "map";
		//if(!nh.getParam("/but_aligner/angle_threshold", angle_thresh))
		//	angle_thresh = (CV_PI / 180) * 20;

		//read parameters
		ros::param::param<string>("~map_frame",map_frame_,"odom");
		ros::param::param<string>("~robot_frame",robot_frame_,"base_link");
		ros::param::param<string>("~map_topic",map_topic_,"/move_base/local_costmap/costmap");

		ros::param::param<double>("~obstacle_bloat",obstacle_bloat_,1.0);
		ros::param::param<int>("~algorithm_type",algorithm_type_,1);

		//initialize other variables
		origin_set = false;
		got_map = false;

		tfl = new tf::TransformListener();

		map_subscriber = nh.subscribe(map_topic_, 1, &WaypointCorrector::mapCallback, this);
		//map_image_subscriber = nh.subscribe("/det2costmap/occ_map_img", 1, &WaypointCorrector::mapImageCallback, this);

		//make service
		wp_service = nh.advertiseService("wp_corrector", &WaypointCorrector::serviceCallback, this);
	}

	bool WaypointCorrector::serviceCallback(rt_road_detection::getCorrectedWaypoint::Request &reqest,
			rt_road_detection::getCorrectedWaypoint::Response &response) {

		//extract destiation point
		float dest_x = originX + reqest.wp_in.x / resolution;
		float dest_y = originY - reqest.wp_in.y / resolution;

		Point2f res;
		bool found = correctWaypoint(Point2i(dest_x, dest_y), res);

		if(found) {
			response.wp_out.x = res.x;
			response.wp_out.y = res.y;

			return true;
		}

		return false;
	}

	void WaypointCorrector::getRoboPos()
	{
		ros::Time now = ros::Time::now();
		geometry_msgs::PoseStamped source;
		source.header.stamp = now;
		source.header.frame_id = robot_frame_;	//source frame
		source.pose.position.x = 0;
		source.pose.position.y = 0;
		source.pose.position.z = 0;
		source.pose.orientation.x = 0.0;
		source.pose.orientation.y = 0.0;
		source.pose.orientation.z = 0.0;
		source.pose.orientation.w = 1.0;

		geometry_msgs::PoseStamped mpose;

		std::string target = map_frame_;	//destination frame

		try {

			if (tfl->waitForTransform(target, source.header.frame_id, now, ros::Duration(1.0))) {	//asking for the transformation
				tfl->transformPose(target,source,mpose);	//actual transformation
			} else {
				source.header.stamp = ros::Time(0);		//this part is not necessary, just in case
				tfl->transformPose(target,source,mpose);
				ROS_WARN("Using latest transform available, may be wrong.");
			}
		} catch(tf::TransformException& ex){
			std::cerr << "Transform error: " << ex.what() << std::endl;
		}

		_x = mpose.pose.position.x;
		_y = mpose.pose.position.y;
		geometry_msgs::Vector3 vec = getAsEuler(mpose.pose.orientation);
		_t = 2*CV_PI - vec.z;
		//cout << "x: " << _x << " y: " << _t << endl;

		robo_x 	= originX + _x / resolution;
		robo_y 	= originY - _y / resolution;
		robo_t	= _t;
		//cout << "rx: " << robo_x << " ry: " << robo_y << endl;
		//find heading point
		robot_heading.x = cos(robo_t);
		robot_heading.y = sin(robo_t);

		//remember first robot position
		if(!origin_set)
		{
			robot_origin.x = mpose.pose.position.x;
			robot_origin.y = mpose.pose.position.y;
			robot_origin.z = vec.z;
			robot_origin2d.x = robo_x;
			robot_origin2d.y = robo_y;

			origin_set = true;
		}
	}

	void WaypointCorrector::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
	{
		//cout << "map map" << endl;
		got_map = true;
		//now create Opencv image from map
		width 		= (*msg).info.width;
		height 		= (*msg).info.height;
		resolution 	= (*msg).info.resolution;
		originX		= -(*msg).info.origin.position.x / resolution;
		//originY		= -(*msg).info.origin.position.y / resolution;
		originY		= height + (*msg).info.origin.position.y / resolution - 1;
		//cout << "ox: " << (*msg).info.origin.position.x << " oy: " << (*msg).info.origin.position.y << endl;

		//get MAP
		cv_map	= Mat::zeros(height, width, CV_8U);

		//set XandY robot vals
		getRoboPos();

		for(int a = 0; a < height*width; a++)
		{
			if( (*msg).data[a] != -1 && (*msg).data[a] < 50 )
			{
				cv_map.at<unsigned char>(a / width, a % width) = 255;
			}
		}
		//flip
		flip(cv_map, cv_map, 0);

		/*cv::Mat element = (cv::Mat_<uchar>(3,3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
		erode(cv_map, cv_map, element, Point(-1,-1), 3);
		dilate(cv_map, cv_map, element, Point(-1,-1), 3);*/

		//call processing
		// 1. get segments around the robot
		destination.x = robo_x + 6 / resolution;
		destination.y = robo_y;
		float radius = sqrt( pow(robo_x - destination.x, 2) + pow(robo_y - destination.y, 2) );

		vector<pair<Point3f, Point2f> > segments = getSegments(radius);

		// 2. process the segments
		float min_width = path_min_width_;
		float max_width = path_max_width_;
		float angle_tolerance = CV_PI / 2;
		processSegments(segments, min_width, max_width, angle_tolerance);
	}

	bool WaypointCorrector::correctWaypoint(Point2i wp, Point2f &result) {
#ifdef CV_VISUALIZE
	//drawing purposes
	Mat draw = cv_map.clone();
	//draw what we got
	circle(draw, Point(robo_x, robo_y), 2, Scalar(150), 3);
	//heading
	circle(draw, Point(robo_x + robot_heading.x*5, robo_y + robot_heading.y*5), 1, Scalar(150), 2);
	line(draw, Point(robo_x, robo_y), Point(robo_x + robot_heading.x*5, robo_y + robot_heading.y*5), Scalar(200), 1, 8, 0);
#endif

	// CASE 1. - Point is on path - find middle of the road
	if( cv_map.at<unsigned char>(wp.y, wp.x) == 255 ) {
		//find vector perpendicular to roboline
		Point2f line1(wp.y - robo_y, -(wp.x - robo_x));
		float line_size = sqrt(line1.x*line1.x + line1.y*line1.y);
		line1.x = line1.x / line_size;
		line1.y = line1.y / line_size;

		//now in both directions, find corner points
		bool got1 = false, got2 = false;
		Point2f pt1, pt2;
		for(unsigned int i = 0; i < height; i ++) {
			if(!got1) {
				pt1.x = wp.x + i*line1.x;
				pt1.y = wp.y + i*line1.y;
			}
			if(!got2) {
				pt2.x = wp.x - i*line1.x;
				pt2.y = wp.y - i*line1.y;
			}
			if(!got1 && pt1.x >= 0 && pt1.x < width && pt1.y >= 0 && pt1.y < height ) {
				if(cv_map.at<unsigned char>(pt1.y, pt1.x) != 255)
					got1 = true;
			} else {
				got1 = true;
			}

			if(!got2 && pt2.x >= 0 && pt2.x < width && pt2.y >= 0 && pt2.y < height ) {
				if(cv_map.at<unsigned char>(pt2.y, pt2.x) != 255)
					got2 = true;
			} else {
				got2 = true;
			}
			//end sooner if possible
			if(got1 && got2)
				break;
		}

		Point2f res(wp.x, wp.y);
		//check if we really need to correct this point
		float dist1 = sqrt((pt1.x-wp.x)*(pt1.x-wp.x) + (pt1.y-wp.y)*(pt1.y-wp.y));
		float dist2 = sqrt((pt2.x-wp.x)*(pt2.x-wp.x) + (pt2.y-wp.y)*(pt2.y-wp.y));
		if(dist1*resolution < obstacle_bloat_) {
			//correct from pt1
			if(dist2*resolution < obstacle_bloat_*3 || dist2*resolution < obstacle_bloat_) {
				//ok
				res.x = (pt1.x + pt2.x) / 2;
				res.y = (pt1.y + pt2.y) / 2;
			} else {
				//try to move it towards right bloat,5 m
				res.x = pt1.x - ((obstacle_bloat_)/resolution)*line1.x;
				res.y = pt1.y - ((obstacle_bloat_)/resolution)*line1.y;
			}
		} else if(dist2*resolution < obstacle_bloat_) {
			//correct from pt1
			if(dist1*resolution < obstacle_bloat_*3 || dist1*resolution < obstacle_bloat_) {
				//ok
				res.x = (pt1.x + pt2.x) / 2;
				res.y = (pt1.y + pt2.y) / 2;
			} else {
				//try to move it towards right bloat,5 m
				res.x = pt2.x + ((obstacle_bloat_)/resolution)*line1.x;
				res.y = pt2.y + ((obstacle_bloat_)/resolution)*line1.y;
			}
		}

		//when we have it, check the middlepoint if it is valid
		//Point2f res((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2);
#ifdef CV_VISUALIZE
		line(draw, pt1, pt2, Scalar(200), 1, 8, 0);
		circle(draw, Point(wp.x, wp.y), 2, Scalar(150), 3);
		imshow( "win3", draw );
		waitKey(10);
#endif

		if(cv_map.at<unsigned char>(res.y, res.x) == 255) {
#ifdef CV_VISUALIZE
			circle(draw, res, 4, Scalar(150), 3);
			//draw
			imshow( "win3", draw );
			waitKey(10);
#endif
			result.x = -originX * resolution + res.x * resolution;
			result.y = -(res.y - originY)*resolution;

			return true;
		}
		return false;
	}

	// CASE 2. - Point is out of path
	else {
		Point2f res(wp.x, wp.y);
		//lets do this other way...
		bool solution_found = false;
		if(algorithm_type_ == 1) {
			//join robot pose and WP. find first available point of the line that is obstacle_bloat_ away
			//find vector perpendicular to roboline
			Point2f line(wp.x - robo_x, wp.y - robo_y);
			float line_size = sqrt(line.x*line.x + line.y*line.y);
			line.x = line.x / line_size;
			line.y = line.y / line_size;
			//compute distance robot <-> wp
			float robot2wp = sqrt( (wp.x - robo_x)*(wp.x - robo_x) +(wp.y - robo_y)*(wp.y - robo_y) );
			//now move from WP towards robot and check for "OK" point
			bool cycle = true;
			float dist = 0;
			Point2f pt1(0, 0);
			bool got_pt1 = false;
			while(cycle) {
				int nx = wp.x - line.x*dist;
				int ny = wp.y - line.y*dist;
				//circle(draw, Point(nx, ny), 2, Scalar(150), 2);
				//check pixel here
				if(cv_map.at<unsigned char>(ny, nx) == 255) {
					if(!got_pt1) {
						got_pt1 = true;
						pt1.x = nx;
						pt1.y = ny;
						//cout << "1: " << pt1.x << " " << pt1.y << endl;
					} else {
						//check distance from obstacle
						float obst_dist = sqrt( (nx - pt1.x)*(nx - pt1.x) + (ny - pt1.y)*(ny - pt1.y) );
						if(obst_dist > obstacle_bloat_/resolution) {
							//we got the point
							//cout << "2: " << nx << " " << ny << endl;
							res.x = nx;
							res.y = ny;
							cycle = false;
							solution_found = true;
						}
					}
				} else {
					got_pt1 = false;
				}

				//check dist wp <-> pt
				float pt2wp = sqrt( (wp.x - nx)*(wp.x - nx) +(wp.y - ny)*(wp.y - ny) );
				if(pt2wp > robot2wp) {
					cycle = false;
					//perform another approach
				}

				dist += 1.0;
			}
		}
		if(algorithm_type_ == 2 || !solution_found) {
			//find vector perpendicular to roboline			//vector perpendicular to wp_vect
			Point2f seek_vect(wp.y - robo_y, -(wp.x - robo_x));
			float seek_vect_size = sqrt(seek_vect.x*seek_vect.x + seek_vect.y*seek_vect.y);
			seek_vect.x = seek_vect.x / seek_vect_size;
			seek_vect.y = seek_vect.y /seek_vect_size;

			Point2f wp_vect(wp.x - robo_x, wp.y - robo_y);	//vector pointing from robot to WP
			float wp_vect_size = sqrt(wp_vect.x*wp_vect.x + wp_vect.y*wp_vect.y);
			wp_vect.x = wp_vect.x / wp_vect_size;
			wp_vect.y = wp_vect.y / wp_vect_size;

			//decide which way to look for valid point
			//check cross product of robo_heading and wp_vect
			if(robot_heading.x * wp_vect.y - robot_heading.y*wp_vect.x > 0) {
				//invert seek vector
				seek_vect.x = -seek_vect.x;
				seek_vect.y = -seek_vect.y;
			}

			//now search for a solution.
			//now move from WP towards seek vector and check for "OK" point
			bool cycle = true;
			float dist = 0;
			Point2f pt1(0, 0);
			bool got_pt1 = false;
			while(cycle) {
				int nx = wp.x - seek_vect.x*dist;
				int ny = wp.y - seek_vect.y*dist;
				//circle(draw, Point(nx, ny), 2, Scalar(150), 2);
				//check pixel here
				if(cv_map.at<unsigned char>(ny, nx) == 255) {
					if(!got_pt1) {
						got_pt1 = true;
						pt1.x = nx;
						pt1.y = ny;
						//cout << "1: " << pt1.x << " " << pt1.y << endl;
					} else {
						//check distance from obstacle
						float obst_dist = sqrt( (nx - pt1.x)*(nx - pt1.x) + (ny - pt1.y)*(ny - pt1.y) );
						if(obst_dist > obstacle_bloat_/resolution) {
							//we got the point
							//cout << "2: " << nx << " " << ny << endl;
							res.x = nx;
							res.y = ny;
							cycle = false;
							solution_found = true;
						}
					}
				} else {
					got_pt1 = false;
				}

				//check dist wp <-> pt
				float pt2wp = sqrt( (wp.x - nx)*(wp.x - nx) +(wp.y - ny)*(wp.y - ny) );
				if(pt2wp > (obstacle_bloat_*4)/resolution) {
					cycle = false;
					//perform another approach
				}

				dist += 1.0;
			}
		}

#ifdef CV_VISUALIZE
		//ok, we got it and can return it
		circle(draw, Point(res.x, res.y), 4, Scalar(150), 3);
		circle(draw, Point(wp.x, wp.y), 2, Scalar(150), 3);

		//draw
		imshow( "win3", draw );
		waitKey(10);
#endif

		if(cv_map.at<unsigned char>(res.y, res.x) == 255 && solution_found) {
			result.x = -originX * resolution + res.x * resolution;
			result.y = -(res.y - originY)*resolution;
			/*cout << result.x << " " << result.y << endl;
			cout << "r: " << _x << " " << _y << endl;*/
			return true;
		} else {
			return false;
		}
	}

	/*	@brief 	Takes map, robot position and destination as imput and tries to fit better destination point
	 *			to the part of map where road is.
	 */
	vector<pair<Point3f, Point2f> > WaypointCorrector::getSegments(float radius) {
#ifdef CV_VISUALIZE
		//drawing purposes
		Mat draw = cv_map.clone();
		//draw what we got
		circle(draw, Point(robo_x, robo_y), 2, Scalar(150), 3);
#endif

		float granularity = 0.5;	//in degrees
		std::vector<int> circle_histogram;

		//set initial alpha
		float alpha = 0.f;	//in radians

		for(float i = alpha; i < alpha + CV_PI*2; i += (CV_PI*2 / 360) * granularity) {
			//get x, y, in image
			float x = robo_x + cos(i) * radius;
			float y = robo_y + sin(i) * radius;

			//store pixel data to vector
			if(x >= 0 && x < cv_map.cols && y >= 0 && y < cv_map.rows)
				circle_histogram.push_back( (int)(cv_map.at<uchar>(y, x)) );
			else
				circle_histogram.push_back( 0 );

#ifdef CV_VISUALIZE
			circle(draw, Point(x, y), 1, Scalar(150), 2);
#endif
		}

		//now search for segments
		int i = 0;
		while(circle_histogram.at(i) > UNKNOWN_THRESH)
			i ++;

		//find segments
		vector<Point2i> segments;
		bool segment_started = false;
		Point2i segment;
		int index = i;
		for(unsigned int n_iterations = 0; n_iterations < circle_histogram.size(); ++n_iterations, index = (index + 1) % circle_histogram.size()) {
			if(circle_histogram.at(index) > UNKNOWN_THRESH) {
				if(!segment_started) {
					//start segment
					segment.x = index;
					segment_started = true;
				}
			} else {
				if(segment_started) {
					//store complete segment
					segment.y = (index-1) % circle_histogram.size();
					segment_started = false;
					segments.push_back(segment);
				}
			}
		}
		//end segment if it was begun
		if(segment_started) {
			segment.y = index;
			segments.push_back(segment);
		}

		//now draw segments
		vector<pair<Point3f, Point2f> > segment_points;
		for(unsigned int i = 0; i < segments.size(); ++i) {
			Point3f start;
			Point2f end;
			start.x = robo_x + cos((CV_PI*2 / 360) * granularity * segments.at(i).x) * radius;
			start.y = robo_y + sin((CV_PI*2 / 360) * granularity * segments.at(i).x) * radius;
			if(segments.at(i).x < segments.at(i).y)
				start.z = (CV_PI*2 / 360) * granularity * ((segments.at(i).x + segments.at(i).y) / 2);
			else
				start.z = 0;

			end.x = robo_x + cos((CV_PI*2 / 360) * granularity * segments.at(i).y) * radius;
			end.y = robo_y + sin((CV_PI*2 / 360) * granularity * segments.at(i).y) * radius;

			segment_points.push_back( make_pair(start, end) );
#ifdef CV_VISUALIZE
			//draw line
			line(draw, Point(start.x, start.y), end, Scalar(200), 1, 8, 0);
#endif
		}

#ifdef CV_VISUALIZE
		//draw
		imshow( "win1", draw );
		waitKey(10);
#endif

		return segment_points;
	}

	/*	@brief 	Purges out the segments that dont fit the parameters
	 */
	vector<pair<Point3f, Point2f> > WaypointCorrector::processSegments(vector<pair<Point3f, Point2f> > &segments, float min_width, float max_width, float angle_tolerance) {
		//drawing purposes
		vector<pair<Point3f, Point2f> > valid_segments;
#ifdef CV_VISUALIZE
		Mat draw2 = cv_map.clone();
		circle(draw2, Point(robo_x, robo_y), 1, Scalar(150), 3);
#endif

		for(unsigned int i = 0; i < segments.size(); ++i) {
#ifdef CV_VISUALIZE
			//draw line
			line(draw2, Point(segments.at(i).first.x, segments.at(i).first.y), Point(segments.at(i).second.x, segments.at(i).second.y), Scalar(200), 1, 8, 0);
#endif

			//get width of path
			float width1 = pow(segments.at(i).first.x - segments.at(i).second.x, 2) + pow(segments.at(i).first.y - segments.at(i).second.y, 2);
			float width = sqrt(width1);
			//cout << "w: " << width*resolution << " p1: " << segments.at(i).first.x << " " << segments.at(i).first.y << " |p2: " << segments.at(i).second.x << " " << segments.at(i).second.y << endl;
			//cout << _t << "  " << segments.at(i).first.z << " | " << angle_difference(_t, segments.at(i).first.z) << " | " << angle_tolerance << endl;
			if(width*resolution > min_width && width*resolution < max_width &&
					angle_difference(_t, segments.at(i).first.z) < angle_tolerance ) {
#ifdef CV_VISUALIZE
				//draw possibility point
				circle(draw2, Point((segments.at(i).first.x + segments.at(i).second.x)/2,
						(segments.at(i).first.y + segments.at(i).second.y)/2), 2, Scalar(150), 3);
#endif

				//store valid segment
				valid_segments.push_back(segments.at(i));
			}
		}
		//cout << endl;
		//check for crossroads
		if(valid_segments.size() > 1) {
#ifdef CV_VISUALIZE
			//draw it
			circle(draw2, Point(robo_x, robo_y), 8, Scalar(150), 3);
#endif
		}

#ifdef CV_VISUALIZE
		imshow( "win2", draw2 );
		waitKey(10);
#endif

		return valid_segments;
	}

	/*	@brief 	Takes map, robot position and destination as imput and tries to fit better destination point
	 *			to the part of map where road is.
	 */
	/*void WaypointCorrector::getCorrectedPose() {
		//basicaly we morphologicaly find nearest path point to our picked point
		float granularity = 10;	//in degrees
		std::vector<unsigned char> circle_histogram;

		//compute the radius of circle
		int radius = 1;	//in pixels

		//set initial alpha
		float alpha = 0.f;	//in radians

		//output point
		cv::Point2i new_destination;

		//if point is in path, there is no problem
		if(cv_map.at<unsigned char>((int)destination.x, (int)destination.y) > UNKNOWN_THRESH) {
			new_destination.x = (int)destination.x;
			new_destination.y = (int)destination.y;
		} else {

			bool point_found = false;

			for(float i = alpha; i < alpha + CV_PI*2; i += (CV_PI*2 / 360) * granularity) {
				//get x, y, in image
				float x = robot_position.x + cos(i) * radius;
				float y = robot_position.y + sin(i) * radius;

				//store pixel data to vector
				if(x >= 0 && x < cv_map.cols && y >= 0 && y < cv_map.rows)
					circle_histogram.push_back( cv_map.at<unsigned char>(y, x) );
				else
					circle_histogram.push_back( 0 );
			}

			//find correct segmnts
			int first_segment = -1;
			//check if all points are valid
			bool all_valid = true;
			for(int i = 0; i < circle_histogram.size(); i++) {
				if(circle_histogram.at(i) <= UNKNOWN_THRESH) {
					all_valid = false;
					break;
				}
			}
			//if there is well..

			//find valid point
			for(int i = 0; i < circle_histogram.size(); i++) {
				int next = (i + 1) % circle_histogram.size();

				if(circle_histogram.at(i) <= UNKNOWN_THRESH && circle_histogram.at(next) > UNKNOWN_THRESH) {
					first_segment = i;
					break;
				}
			}

			if(first_segment != -1) {
				//now extract the segment
				int segments = 0;
				int begin = first_segment, end = first_segment;
				for(int i = 0, j = first_segment; i < circle_histogram.size(); i++, j = (j + 1) % circle_histogram.size()) {
					if(circle_histogram.at(i) <= UNKNOWN_THRESH) {
						//the segment ends
						end = (j - 1) % circle_histogram.size();
						segments ++;
					}
				}
			}
		}
	}*/

	geometry_msgs::Vector3 WaypointCorrector::getAsEuler(geometry_msgs::Quaternion quat)
	{
		geometry_msgs::Vector3 vec;

		double squ;
		double sqx;
		double sqy;
		double sqz;

		squ = quat.w * quat.w;
		sqx = quat.x * quat.x;
		sqy = quat.y * quat.y;
		sqz = quat.z * quat.z;

		// Roll
		vec.x = atan2(2 * (quat.y*quat.z + quat.w*quat.x), squ - sqx - sqy + sqz);

		// Pitch
		vec.y = asin(-2 * (quat.x*quat.z - quat.w * quat.y));

		// Yaw
		vec.z = atan2(2 * (quat.x*quat.y + quat.w*quat.z), squ + sqx - sqy - sqz);

		return vec;
	}

	float WaypointCorrector::angle_difference(float angle1, float angle2) {
		return min(fabs(angle1 - angle2), (float)fabs((fabs(angle1-angle2)-CV_PI*2)));
	}

}
