/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
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

#pragma once
#ifndef BUT_ENV_MDOEL_TOPICS_LIST_H
#define BUT_ENV_MDOEL_TOPICS_LIST_H

#include "services_list.h"

namespace srs_env_model
{
/**
 * but_server
 */
static const std::string WORLD_FRAME_ID = "/map";
static const std::string BASE_FRAME_ID = "/base_footprint";
static const int NUM_PCFRAMES_PROCESSED = 3;

/**
 * cmap_plugin
 */
static const std::string COLLISION_MAP_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/collision_map");
static const std::string COLLISION_MAP_FRAME_ID = "/base_footprint";
static const double COLLISION_MAP_RADIUS_LIMIT = 2.0;

/**
 * collision_object_plugin
 */
static const std::string COLLISION_OBJECT_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/collision_object");
static const std::string COLLISION_OBJECT_FRAME_ID = "/map";

/**
 * imarkers_plugin
 */
static const std::string IM_SERVER_FRAME_ID = "/map";
static const std::string IM_SERVER_TOPIC_NAME = PACKAGE_NAME_PREFIX + std::string("/im_server");

/**
 * imarkers_old_plugin
 */
static const std::string OLD_IM_SERVER_TOPIC_NAME = PACKAGE_NAME_PREFIX + std::string("/old_im_server");

/**
 * marker_array_plugin
 */
static const std::string MARKERARRAY_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/marker_array_object");
static const std::string MARKERARRAY_FRAME_ID = "/map";

/**
 * map2d_plugin
 */
static const std::string MAP2D_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/map2d_object");
static const std::string MAP2D_FRAME_ID = "/map";

/**
 * collision_grid_plugin
 */
static const std::string COLLISIONGRID_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/collision_grid_object");


/**
 * octomap_plugin
 */
static const std::string OCTOMAP_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/binary_octomap");
static const std::string OCTOMAP_FRAME_ID = "/map";
static const std::string CAMERA_INFO_TOPIC_NAME = "camera_info"; // /cam3d/rgb/camera_info
static const std::string MARKERS_TOPIC_NAME = "/visualization_marker";

/**
 * point_cloud_plugin
 */
static const std::string POINTCLOUD_CENTERS_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/pointcloud_centers");
static const std::string SUBSCRIBER_POINT_CLOUD_NAME = "points_in"; // /cam3d/rgb/points
//static const std::string DEFAULT_FRAME_ID = "/head_cam3d_link";
static const std::string SUBSCRIBER_FILTERING_CLOUD_NAME="points_filter";

/**
 * limited_point_cloud_plugin
 */
static const std::string SUBSCRIBER_CAMERA_POSITION_NAME = "rviz_camera_position"; // /rviz_camera_position
static const std::string VISIBLE_POINTCLOUD_CENTERS_PUBLISHER_NAME = PACKAGE_NAME_PREFIX
    + std::string("/visible_pointcloud_centers");

/**
 * compressed_point_cloud_plugin
 */
static const std::string CPC_CAMERA_INFO_PUBLISHER_NAME = CAMERA_INFO_TOPIC_NAME; // /cam3d/rgb/camera_info
static const std::string CPC_SIMPLE_PC_PUBLISHING_TOPIC_NAME = PACKAGE_NAME_PREFIX + std::string("/differential_pointcloud_centers");
static const std::string CPC_COMPLETE_TOPIC_NAME = PACKAGE_NAME_PREFIX + std::string("/octomap_updates");
static const int CPC_NUM_DIFFERENTIAL_FRAMES = 5;

/**
 * Registration
 */
static const std::string REGISTRATION_CONSTRAINED_CLOUD_PUBLISHER_NAME = PACKAGE_NAME_PREFIX + std::string("/registration_constrained_cloud");

/**
 * CPC node
 */
static const std::string CPC_INPUT_TOPIC_NAME = "input"; // /but_env_model/octomap_updates
static const std::string CPC_OUTPUT_TOPIC_NAME = "output"; //PACKAGE_NAME_PREFIX + std::string("/cpc_pointcloud_centers");
static const std::string CPC_CAMERA_FRAME = std::string("/head_cam3d_link");
static const std::string CPC_WORLD_FRAME = std::string("/map");

/**
 * Context Server - topics
 */
static const std::string ContextChanged_TOPIC = PACKAGE_NAME_PREFIX + std::string("/context/changed");

}

#endif // BUT_ENV_MDOEL_TOPICS_LIST_H
