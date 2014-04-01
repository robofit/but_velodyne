
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include "rt_road_detection/WaypointCorrector.h"

using namespace cv;
using namespace std;
using namespace rt_road_detection;

WaypointCorrector *wc;

//callback function
#ifdef CV_VISUALIZE
void mouseEvent(int evt, int x, int y, int flags, void* param){
    if(evt==CV_EVENT_LBUTTONDOWN){
        //printf("%d %d\n",x,y);
        Point2i p(x, y);
        Point2f res;
        wc->correctWaypoint(p, res);
    }
}
#endif

int main(int argc, char *argv[])
{
	// ROS initialization
	ros::init(argc, argv, "robotour_waypoint_corrector");

#ifdef CV_VISUALIZE
	namedWindow("win1", 1);
	namedWindow("win2", 1);
	namedWindow("win3", 1);

	setMouseCallback("win1", mouseEvent, 0);
#endif

	wc = new WaypointCorrector();

	//Set the rate in Hz
	ros::Rate rate(10);
	while( ros::ok() )
	{
		//Call all callbacks
		ros::spinOnce();

		rate.sleep();
	}

	return 0;
}
