/******************************************************************************
 * \file
 * $Id: servernode.cpp 834 2012-05-23 16:36:59Z spanel $
 *
 * Modified by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * This code is derived from the OctoMap server provided by A. Hornung.
 * Please, see the original comments below.
 */

/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009 - 2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/**
 * Copyright (c) 2009-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <srs_env_model/but_server/but_server.h>

#define USAGE "\nUSAGE: octomap_server <map.bt>\n" \
              "  map.bt: octomap 3D map file to read\n"

int main(int argc, char** argv){
  ros::init(argc, argv, "but_env_model");
  std::string mapFilename("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
          ROS_ERROR("%s", USAGE);
          exit(-1);
  }

  std::cerr << "Number of octomap server parameters: " << argc << std::endl;


  if (argc == 2)
  {
	  std::cerr << "Trying to load input octomap file: " << argv[1] << std::endl;
          mapFilename = std::string(argv[1]);
  }
  try{
		// Run server
		srs_env_model::CButServer ms(mapFilename);
		ros::Rate loop_rate(10);

		int count = 0;

		while (ros::ok())
		{
			ros::spinOnce();

			loop_rate.sleep();
			++count;
		}
  }catch(std::runtime_error& e){
          ROS_ERROR("octomap_server exception: %s", e.what());
          return -1;
  }


  return 0;
}
