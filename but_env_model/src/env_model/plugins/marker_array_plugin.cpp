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

#include <but_env_model/plugins/marker_array_plugin.h>
#include <but_env_model/topics_list.h>

#include <pcl_ros/transforms.h>


but_env_model::CMarkerArrayPlugin::CMarkerArrayPlugin(const std::string & name)
  : but_env_model::CServerPluginBase(name)
  , m_publishMarkerArray(true)
  , m_markerArrayPublisherName(MARKERARRAY_PUBLISHER_NAME)
  , m_latchedTopics(false)
  , m_markerArrayFrameId(MARKERARRAY_FRAME_ID)
  , m_bHeightMap(true)
  , m_bTransform(false)
  , m_colorFactor(0.8)
{
  assert(m_data != 0);
}



but_env_model::CMarkerArrayPlugin::~CMarkerArrayPlugin()
{
}



bool but_env_model::CMarkerArrayPlugin::shouldPublish()
{
  return(m_publishMarkerArray && m_markerArrayPublisher.getNumSubscribers() > 0);
}



void but_env_model::CMarkerArrayPlugin::init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
{
  private_nh.param("marker_array_publisher", m_markerArrayPublisherName, MARKERARRAY_PUBLISHER_NAME);
  private_nh.param("marker_array_frame_id", m_markerArrayFrameId, MARKERARRAY_FRAME_ID);

  // Get collision map crawling depth
  int depth(m_crawlDepth);
  private_nh.param("marker_array_octree_depth", depth, depth);
  m_crawlDepth = depth > 0 ? depth : 0;

  double r, g, b, a;
  private_nh.param("color/r", r, 0.0);
  private_nh.param("color/g", g, 0.0);
  private_nh.param("color/b", b, 1.0);
  private_nh.param("color/a", a, 1.0);

  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  // Create publisher
  m_markerArrayPublisher = nh.advertise<visualization_msgs::MarkerArray> (m_markerArrayPublisherName, 5, m_latchedTopics);
}



void but_env_model::CMarkerArrayPlugin::publishInternal(const ros::Time & timestamp)
{
  if (shouldPublish())
    m_markerArrayPublisher.publish(*m_data);
}



void but_env_model::CMarkerArrayPlugin::newMapDataCB(SMapWithParameters & par)
{
  // each array stores all cubes of a different size, one for each depth level:
  m_data->markers.resize(par.treeDepth + 1);

  m_ocFrameId = par.frameId;

  // Get octomap parameters
  par.map->getTree().getMetricMin(m_minX, m_minY, m_minZ);
  par.map->getTree().getMetricMax(m_maxX, m_maxY, m_maxZ);

  m_bTransform = m_ocFrameId != m_markerArrayFrameId;

  // Is transform needed?
  if (! m_bTransform)
    return;

  ros::Time timestamp(par.currentTime);
  tf::StampedTransform ocToMarkerArrayTf;


  // Get transform
  try
  {
    // Transformation - to, from, time, waiting time
    m_tfListener.waitForTransform(m_markerArrayFrameId, m_ocFrameId,
                                  timestamp, ros::Duration(5));

    m_tfListener.lookupTransform(m_markerArrayFrameId, m_ocFrameId,
                                 timestamp, ocToMarkerArrayTf);

  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("MarkerArrayPlugin: Transform error - " << ex.what() << ", quitting callback");
    PERROR("Transform error.");
    return;
  }


  Eigen::Matrix4f ocToMarkerArrayTM;

  // Get transformation matrix
  pcl_ros::transformAsMatrix(ocToMarkerArrayTf, ocToMarkerArrayTM);

  // Disassemble translation and rotation
  m_ocToMarkerArrayRot  = ocToMarkerArrayTM.block<3, 3> (0, 0);
  m_ocToMarkerArrayTrans = ocToMarkerArrayTM.block<3, 1> (0, 3);


  tButServerOcTree & tree(par.map->getTree());
  but_env_model::tButServerOcTree::leaf_iterator it, itEnd(tree.end_leafs());

  // Crawl through nodes
  for (it = tree.begin_leafs(m_crawlDepth); it != itEnd; ++it)
  {
    handleNode(it, par);
  } // Iterate through octree

  handlePostNodeTraversal(par);

  m_DataTimeStamp = par.currentTime;

}

void but_env_model::CMarkerArrayPlugin::handleNode(const but_env_model::tButServerOcTree::iterator & it, const SMapWithParameters & mp)
{
  unsigned idx = it.getDepth();
  assert(idx < m_data->markers.size());

  geometry_msgs::Point cubeCenter;



  if (m_bTransform)
  {
    // Transform input point
    Eigen::Vector3f point(it.getX(), it.getY(), it.getZ());
    point = m_ocToMarkerArrayRot * point + m_ocToMarkerArrayTrans;
    cubeCenter.x = it.getX();
    cubeCenter.y = it.getY();
    cubeCenter.z = it.getZ();

  }
  else
  {
    cubeCenter.x = it.getX();
    cubeCenter.y = it.getY();
    cubeCenter.z = it.getZ();
  }

  m_data->markers[idx].points.push_back(cubeCenter);

  if (m_bHeightMap)
  {
    double h = (1.0 - std::min(std::max((cubeCenter.z - m_minZ) / (m_maxZ - m_minZ), 0.0), 1.0)) * m_colorFactor;
    m_data->markers[idx].colors.push_back(heightMapColor(h));
  }
}



void but_env_model::CMarkerArrayPlugin::handlePostNodeTraversal(const SMapWithParameters & mp)
{
  for (unsigned i = 0; i < m_data->markers.size(); ++i)
  {
    double size = mp.map->getTree().getNodeSize(i);

    m_data->markers[i].header.frame_id = m_markerArrayFrameId;
    m_data->markers[i].header.stamp = mp.currentTime;
    m_data->markers[i].ns = "map";
    m_data->markers[i].id = i;
    m_data->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    m_data->markers[i].scale.x = size;
    m_data->markers[i].scale.y = size;
    m_data->markers[i].scale.z = size;
    m_data->markers[i].color = m_color;


    if (m_data->markers[i].points.size() > 0)
      m_data->markers[i].action = visualization_msgs::Marker::ADD;
    else
      m_data->markers[i].action = visualization_msgs::Marker::DELETE;
  }

  invalidate();
}

std_msgs::ColorRGBA but_env_model::CMarkerArrayPlugin::heightMapColor(double h) const
{

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
  case 6:
  case 0:
    color.r = v;
    color.g = n;
    color.b = m;
    break;
  case 1:
    color.r = n;
    color.g = v;
    color.b = m;
    break;
  case 2:
    color.r = m;
    color.g = v;
    color.b = n;
    break;
  case 3:
    color.r = m;
    color.g = n;
    color.b = v;
    break;
  case 4:
    color.r = n;
    color.g = m;
    color.b = v;
    break;
  case 5:
    color.r = v;
    color.g = m;
    color.b = n;
    break;
  default:
    color.r = 1;
    color.g = 0.5;
    color.b = 0.5;
    break;
  }

  return color;
}

/**
 * Pause/resume plugin. All publishers and subscribers are disconnected on pause
 */
void but_env_model::CMarkerArrayPlugin::pause(bool bPause, ros::NodeHandle & nh)
{
  if (bPause)
    m_markerArrayPublisher.shutdown();
  else
    m_markerArrayPublisher = nh.advertise<visualization_msgs::MarkerArray> (m_markerArrayPublisherName, 5, m_latchedTopics);
}
