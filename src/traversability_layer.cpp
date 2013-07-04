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

namespace costmap_2d {

void TraversabilityLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);


}

void TraversabilityLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y) {

	// rolling window
	updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);

}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {


}

void TraversabilityLayer::activate() {



}

void TraversabilityLayer::deactivate() {



}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(costmap_2d::TraversabilityLayer, costmap_2d::Layer)

