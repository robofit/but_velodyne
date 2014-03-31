/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 14/08/2013
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <but_env_model/env_model_server.h>

namespace but_env_model
{

class EnvModelNodelet: public nodelet::Nodelet
{
public:
  EnvModelNodelet() {}
  ~EnvModelNodelet() {}

private:
  virtual void onInit()
  {
    env_model_server_.reset(new CButServer(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<CButServer> env_model_server_;
};

} // namespace but_velodyne


// Register this plugin with pluginlib. Names must match nodelets.xml.
PLUGINLIB_DECLARE_CLASS(but_env_model,
                        EnvModelNodelet,
                        but_env_model::EnvModelNodelet,
                        nodelet::Nodelet);
