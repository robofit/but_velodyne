/*
 * traversability_layer.cpp
 *
 *  Created on: 4.7.2013
 *      Author: imaterna
 */


#include<but_costmap_plugins/traversability_layer.h>
#include<costmap_2d/costmap_math.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;
using namespace std;

namespace costmap_2d {

void TraversabilityLayer::onInitialize()
{


  ros::NodeHandle nh("~/" + name_), g_nh;

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("/det2costmap/occ_map"));

  global_frame_ = layered_costmap_->getGlobalFrameID();
  
  rolling_window_ = layered_costmap_->isRolling();
  
  if (rolling_window_) cout << "rolling window enabled" << endl;
  else cout << "rolling window NOT enabled" << endl;

  map_sub_ = g_nh.subscribe(map_topic, 1, &TraversabilityLayer::incomingMap, this);

  map_ptr_.reset();

  map_received_ = false;
  
  current_ = true;

  int tmp;

  // TODO add some warning msgs
  nh.param<int>("inc_step", tmp, 32);
  if (tmp > LETHAL_OBSTACLE) tmp = LETHAL_OBSTACLE;
  if (tmp < 1) tmp = 1;
  costmap_inc_step_ = (unsigned char)tmp;

  nh.param<int>("dec_step", tmp, 8);
  if (tmp > LETHAL_OBSTACLE) tmp = LETHAL_OBSTACLE;
  if (tmp < 1) tmp = 1;
  costmap_dec_step_ = (unsigned char)tmp;

  nh.param<int>("unknown_dec_step", tmp, 1);
  if (tmp > LETHAL_OBSTACLE) tmp = LETHAL_OBSTACLE;
  if (tmp < 0) tmp = 0;
  costmap_unknown_dec_step_ = (unsigned char)tmp;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
    {
      ROS_INFO_THROTTLE(1.0,"Waiting for map on %s topic...",map_topic.c_str());
      ros::spinOnce();
      r.sleep();
    }

}

void TraversabilityLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map) {

	unsigned int size_x = new_map->info.width, size_y = new_map->info.height;


	ROS_INFO_ONCE("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

	  
  // TODO make buffer of maps ???
  map_ptr_ = new_map;
  
  ROS_ASSERT(map_ptr_->info.width * map_ptr_->info.height == map_ptr_->data.size());

  map_received_ = true;

}

void TraversabilityLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y) {


	if (rolling_window_) updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);


    if (map_received_) {

        for (unsigned int x=0; x < map_ptr_->info.width; x++)
        for (unsigned int y=0; y < map_ptr_->info.height; y++) {

          unsigned int mx,my;

          double px,py;

          px = map_ptr_->info.origin.position.x + (x * map_ptr_->info.resolution);
          py = map_ptr_->info.origin.position.y + (y * map_ptr_->info.resolution);

          if (!worldToMap(px,py,mx,my)) continue;

          unsigned int index = getIndex(mx, my);

          char val = map_ptr_->data[(y*map_ptr_->info.width) + x];

          // handle special values
          /*if (val==-1) costmap_[index] = NO_INFORMATION;
          else if (val==99) costmap_[index] = INSCRIBED_INFLATED_OBSTACLE;
          else if (val==100) costmap_[index] = LETHAL_OBSTACLE;
          else if (val==0) costmap_[index] = FREE_SPACE;
          else {

        	  // fit range 1 to 98 (inclusive) to 1 to 252 (inclusive)
        	  unsigned char new_val = (unsigned char)( (float)(val - 1) / 97.0 * 251.0 ) + 1;

        	  costmap_[index] = new_val;


          }*/

          if (val != -1) {

        	  unsigned char new_val = ((unsigned char)val*LETHAL_OBSTACLE)/100;

        	  if ( new_val > costmap_[index]) {

        		  if ( (new_val - costmap_[index]) > costmap_inc_step_) costmap_[index] += costmap_inc_step_;
        		  else costmap_[index] = new_val;

        		  /*if (costmap_[index] < LETHAL_OBSTACLE - costmap_inc_step_) costmap_[index] += costmap_inc_step_;
        		  else costmap_[index] = LETHAL_OBSTACLE;*/

        	  }

        	  if ( new_val < costmap_[index]) {

        		  if ( (costmap_[index] - new_val) >  costmap_dec_step_) costmap_[index] -= costmap_dec_step_;
        		  else costmap_[index] = FREE_SPACE;

        		  /*if (costmap_[index] > costmap_dec_step_) costmap_[index] -= costmap_dec_step_;
        		  else costmap_[index] = FREE_SPACE;*/

        	  }



          } else {

        	  if (costmap_[index] > 0 && costmap_[index] < 255) costmap_[index]--;

          }

          //std::cout << "val " << (int)val << ", cm: " << (int)costmap_[index] << std::endl;

          *min_x = std::min(px, *min_x);
          *min_y = std::min(py, *min_y);
          *max_x = std::max(px, *max_x);
          *max_y = std::max(py, *max_y);

        }

        current_ = true;

    }


}

void TraversabilityLayer::matchSize()
{

  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
            
}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {

	 
    if (!map_received_) return;


    //int upd = 0;

    const unsigned char* master_array = master_grid.getCharMap();

  for (int j = min_j; j < max_j; j++)
  for (int i = min_i; i < max_i; i++)
    {

      int index = getIndex(i, j);

      if (costmap_[index] == NO_INFORMATION)
        continue;

      unsigned char old_cost = master_array[index];

      if (old_cost == NO_INFORMATION || old_cost < costmap_[index]) {

    	  //std::cout << "old cost " << old_cost << ", new cost: " << costmap_[index] << std::endl;
          master_grid.setCost(i, j, costmap_[index]);

      }

  }



}

void TraversabilityLayer::activate() {



}

void TraversabilityLayer::deactivate() {



}

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(costmap_2d::TraversabilityLayer, costmap_2d::Layer)

