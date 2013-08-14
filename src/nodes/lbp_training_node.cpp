#include "rt_road_detection/detectors/lbp_training_ros.h"

using namespace rt_road_detection;
using namespace std;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "lbp_training_node");
  ros::NodeHandle nh("~");

  LBPTrainingRos det(nh);

  ROS_INFO("Node started.");
  ros::spin();

}
