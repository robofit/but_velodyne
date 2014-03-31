/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Jan Gorig (xgorig01@stud.fit.vutbr.cz)
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
#ifndef BUT_ENV_MODEL_SERVICES_LIST_H
#define BUT_ENV_MODEL_SERVICES_LIST_H

#include <string>

namespace but_env_model
{
static const std::string PACKAGE_NAME_PREFIX  = "/but_env_model";

/**
 * env. model - services
 */
static const std::string EnvModelReset_SRV = PACKAGE_NAME_PREFIX + std::string("/env_model_reset");
static const std::string EnvModelPause_SRV = PACKAGE_NAME_PREFIX + std::string("/env_model_pause");
static const std::string EnvModelUseInputColor_SRV = PACKAGE_NAME_PREFIX + std::string("/env_model_use_input_color");

/**
 * objtree - services
 */
static const std::string GetObjectsInBox_SRV = PACKAGE_NAME_PREFIX + std::string("/get_objects_in_box");
static const std::string GetObjectsInHalfspace_SRV = PACKAGE_NAME_PREFIX + std::string("/get_objects_in_halfspace");
static const std::string GetObjectsInSphere_SRV = PACKAGE_NAME_PREFIX + std::string("/get_objects_in_sphere");
static const std::string GetPlane_SRV = PACKAGE_NAME_PREFIX + std::string("/get_plane");
static const std::string GetAlignedBox_SRV = PACKAGE_NAME_PREFIX + std::string("/get_aligned_box");
static const std::string GetBoundingBox_SRV = PACKAGE_NAME_PREFIX + std::string("/get_bounding_box");
static const std::string InsertPlane_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_plane");
static const std::string InsertPlaneByPosition_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_plane_by_position");
static const std::string GetSimilarPlane_SRV = PACKAGE_NAME_PREFIX + std::string("/get_similar_plane");
static const std::string InsertPlanes_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_planes");
static const std::string InsertAlignedBox_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_aligned_box");
static const std::string InsertAlignedBoxByPosition_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_aligned_box_by_position");
static const std::string GetSimilarAlignedBox_SRV = PACKAGE_NAME_PREFIX + std::string("/get_similar_aligned_box");
static const std::string InsertBoundingBox_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_bounding_box");
static const std::string InsertBoundingBoxByPosition_SRV = PACKAGE_NAME_PREFIX + std::string("/insert_bounding_box_by_position");
static const std::string GetSimilarBoundingBox_SRV = PACKAGE_NAME_PREFIX + std::string("/get_similar_bounding_box");
static const std::string RemoveObject_SRV = PACKAGE_NAME_PREFIX + std::string("/remove_object");
static const std::string ShowObject_SRV = PACKAGE_NAME_PREFIX + std::string("/show_object");
static const std::string ShowObjtree_SRV = PACKAGE_NAME_PREFIX + std::string("/show_objtree");

/**
 * cmap_plugin - services
 */
static const std::string GetCollisionMap_SRV = PACKAGE_NAME_PREFIX + std::string("/get_collision_map");
static const std::string IsNewCMap_SRV = PACKAGE_NAME_PREFIX + std::string("/is_new_collision_map");
static const std::string LockCMap_SRV = PACKAGE_NAME_PREFIX + std::string("/lock_collision_map");
static const std::string RemoveCubeCMP_SRV = PACKAGE_NAME_PREFIX + std::string("/remove_cube_from_collision_map");
static const std::string AddCubeCMP_SRV = PACKAGE_NAME_PREFIX + std::string("/add_cube_to_collision_map");

/**
 * octomap_plugin - services
 */
static const std::string RemoveCubeOctomap_SRV = PACKAGE_NAME_PREFIX + std::string("/remove_cube_from_octomap");
static const std::string AddCubeOctomap_SRV = PACKAGE_NAME_PREFIX + std::string("/add_cube_to_octomap");
static const std::string ResetOctomap_SRV = PACKAGE_NAME_PREFIX + std::string("/reset_octomap");
static const std::string GetTreeDepth_SRV = PACKAGE_NAME_PREFIX + std::string("/get_tree_depth");
static const std::string SetCrawlDepth_SRV = PACKAGE_NAME_PREFIX + std::string("/set_crawl_depth");
static const std::string LoadMap_SRV = PACKAGE_NAME_PREFIX + std::string("/load_octomap");
static const std::string SaveMap_SRV = PACKAGE_NAME_PREFIX + std::string("/save_octomap");
static const std::string LoadFullMap_SRV = PACKAGE_NAME_PREFIX + std::string("/load_octomap_full");
static const std::string SaveFullMap_SRV = PACKAGE_NAME_PREFIX + std::string("/save_octomap_full");
static const std::string OctomapAgingPause_SRV = PACKAGE_NAME_PREFIX + std::string("/pause_octomap_aging");

/**
 * Compressed pointcloud plugin
 */
static const std::string SetNumIncompleteFrames_SRV = PACKAGE_NAME_PREFIX + std::string("/set_num_incomplete_frames");
}

#endif // BUT_ENV_MODEL_SERVICES_LIST_H
