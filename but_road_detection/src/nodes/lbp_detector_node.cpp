#include "rt_road_detection/detectors/lbp_detector_ros.h"

using namespace rt_road_detection;
using namespace std;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "lbp_detector_node");
  ros::NodeHandle nh("~");

  LBPDetectorRos det(nh);

  ROS_INFO("Node started.");
  ros::spin();

}
