/*
 * costmap_node.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include "rt_road_detection/costmap.h"

using namespace rt_road_detection;
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "traversability_costmap_node");
  ros::NodeHandle nh("~");

  TraversabilityCostmap *tc;

  tc = new TraversabilityCostmap(nh);

  ROS_INFO("Node started.");
  ros::spin();

  delete tc;

}
