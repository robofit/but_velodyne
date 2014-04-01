#include "but_road_detection/detectors/sample_hue_detector_ros.h"


using namespace but_road_detection;
using namespace std;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "hue_detector_node");
  ros::NodeHandle nh("~");


  SampleHueDetectorRos det(nh);

  ROS_INFO("Node started.");
  ros::spin();

}
