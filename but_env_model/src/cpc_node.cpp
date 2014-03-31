/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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

#include <but_env_model/compressed_pc_publisher.h>

int main(int argc, char **argv)
{
  // Do initial ros stuff
  ros::init(argc, argv, "but_cpc_node");
  ros::NodeHandle n;

  // Create object
  but_env_model::CCompressedPCPublisher cpc(n);

  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}

