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

#include <but_env_model/plugins/octomap_plugin_tools/octomap_filter_ground.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

/**
 * Constructor
 */
but_env_model::COcFilterGround::COcFilterGround(const std::string & octree_frame_id, ERunMode mode /*= FILTER_ALLWAYS*/)
  : COcTreeFilterBase(octree_frame_id, mode), m_numSpecRemoved(0)
  , m_inputPc(0)
  , m_groundPc(new tPointCloud)
  , m_nongroundPc(new tPointCloud)
  , m_groundFilterDistance(0.04)
  , m_groundFilterAngle(0.15)
  , m_groundFilterPlaneDistance(0.07)
{
  assert(m_groundPc != 0 && m_nongroundPc != 0);
}

/**
 * Configure filter before each frame. Set input cloud.
 */
void but_env_model::COcFilterGround::setCloud(const tPointCloud * cloud)
{
  assert(cloud != 0);
  m_inputPc = cloud;
}

/**
 * Initialize. Must be called before first filtering
 */
void but_env_model::COcFilterGround::init(ros::NodeHandle & node_handle)
{
  // distance of points from plane for RANSAC
  node_handle.param("ocmap_ground_filter/distance", m_groundFilterDistance,
                    m_groundFilterDistance);
  // angular derivation of found plane:
  node_handle.param("ocmap_ground_filter/angle", m_groundFilterAngle,
                    m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  node_handle.param("ocmap_ground_filter/plane_distance",
                    m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
}

/**
 * Filtering function implementation
 */
void but_env_model::COcFilterGround::filterInternal(tButServerOcTree & tree)
{
  m_groundPc->header = m_inputPc->header;
  m_groundPc->header = m_inputPc->header;

  if (m_inputPc->size() < 50)
  {
    ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
    *m_nongroundPc = *m_inputPc;
  }
  else
  {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<tPclPoint> seg;
    seg.setOptimizeCoefficients(true);
    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(m_groundFilterAngle);

    tPointCloud cloud_filtered(*m_inputPc);
    // Create the filtering object
    pcl::ExtractIndices<tPclPoint> extract;
    bool groundPlaneFound = false;

    while (cloud_filtered.size() > 10 && !groundPlaneFound)
    {
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        ROS_WARN("No plane found in cloud.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3))
          < m_groundFilterPlaneDistance)
      {
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                  cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1),
                  coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative(false);
        extract.filter(*m_groundPc);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size())
        {
          extract.setNegative(true);
          tPointCloud cloud_out;
          extract.filter(cloud_out);
          *m_nongroundPc += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      }
      else
      {
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                  cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1),
                  coefficients->values.at(2), coefficients->values.at(3));
        tPointCloud cloud_out;
        extract.setNegative(false);
        extract.filter(cloud_out);
        *m_nongroundPc += cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<tPclPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size())
        {
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        }
        else
        {
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound)   // no plane found or remaining points too small
    {
      ROS_WARN("No ground plane found in scan");

      // do a rough filtering on height to prevent spurious obstacles
      pcl::PassThrough<tPclPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance,
                                  m_groundFilterPlaneDistance);
      second_pass.setInputCloud(m_inputPc->makeShared());
      second_pass.filter(*m_groundPc);

      second_pass.setFilterLimitsNegative(true);
      second_pass.filter(*m_nongroundPc);
    }

    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<tPclPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<tPclPoint>("nonground.pcd",pc_nonground, false);

  }
}
