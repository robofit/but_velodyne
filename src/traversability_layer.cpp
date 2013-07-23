/*
 * traversability_layer.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include<rt_traversability_layer/traversability_layer.h>
#include<costmap_2d/costmap_math.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;
using namespace std;

namespace rt_traversability_layer {

void TraversabilityLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("/det2costmap/occ_map"));

  global_frame_ = layered_costmap_->getGlobalFrameID();

  nh.param("track_unknown_space", track_unknown_space_, true);

  nh.param("rolling_window", rolling_window_, true);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  map_sub_ = g_nh.subscribe(map_topic, 1, &TraversabilityLayer::incomingMap, this);

  map_recieved_ = false;

  /*ros::Rate r(10);
    while (!map_recieved_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }*/

}

void TraversabilityLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map) {

	unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

	ROS_INFO_ONCE("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

	layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
							  new_map->info.origin.position.y, true);


	std::cout << "map resized" << std::endl;

	if (costmap_==NULL) {

		cout << "ajaj..." << endl;
		return;

	}

	unsigned int index = 0;

	//initialize the costmap with static data
	  for (unsigned int i = 0; i < size_y; ++i)
	  {
		for (unsigned int j = 0; j < size_x; ++j)
		{
		  unsigned char value = new_map->data[index];
		  //check if the static value is above the unknown or lethal thresholds
		  if (track_unknown_space_ && value == unknown_cost_value_)
			costmap_[index] = NO_INFORMATION;
		  else if (value >= lethal_threshold_)
			costmap_[index] = LETHAL_OBSTACLE;
		  else
			costmap_[index] = FREE_SPACE;

		  ++index;
		}
	  }
	  map_recieved_ = true;

	  cout << "map received" << endl;

}

void TraversabilityLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y) {

	std::cout << "updating bounds" << std::endl;

	if (rolling_window_) updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);

	if (!enabled_) return;



}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {


	  if (!enabled_)
	    return;

	  cout << "updating costs" << endl;

	  for (int j = min_j; j < max_j; j++)
	  {
	    for (int i = min_i; i < max_i; i++)
	    {
	      int index = getIndex(i, j);
	      master_grid.setCost(i, j, costmap_[index]);
	    }
	  }

}

void TraversabilityLayer::activate() {



}

void TraversabilityLayer::deactivate() {



}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rt_traversability_layer::TraversabilityLayer, costmap_2d::Layer)

