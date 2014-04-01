#include "but_road_detection/detectors/hsv_hist_detector_ros.h"


using namespace but_road_detection;
using namespace std;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "hsv_hist_detector_node");
  ros::NodeHandle nh("~");


  HSVHistDetectorRos det(nh);

  ROS_INFO("Node started.");
  ros::spin();

}
