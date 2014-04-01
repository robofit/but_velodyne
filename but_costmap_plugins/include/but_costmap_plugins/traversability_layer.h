/*
 * traversability_layer.h
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */

#ifndef TRAVERSABILITY_LAYER_H_
#define TRAVERSABILITY_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint_layer.h>

namespace costmap_2d
{
class TraversabilityLayer : public Layer, public Costmap2D
{
public:


  TraversabilityLayer()
  {
    costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();

  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();


protected:

  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

  nav_msgs::OccupancyGridConstPtr map_ptr_;

  std::string global_frame_; ///< @brief The global frame for the costmap

  bool map_received_;
  ros::Subscriber map_sub_;

  bool rolling_window_;

  unsigned char costmap_inc_step_;
  unsigned char costmap_dec_step_;
  unsigned char costmap_unknown_dec_step_;


};
}


#endif /* TRAVERSABILITY_LAYER_H_ */
