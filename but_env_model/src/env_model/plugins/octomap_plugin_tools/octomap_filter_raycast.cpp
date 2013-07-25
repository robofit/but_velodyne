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

#include <but_env_model/plugins/octomap_plugin_tools/octomap_filter_raycast.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <visualization_msgs/Marker.h>

#include <but_env_model/topics_list.h>

#define SHOW_VISUALIZATION
//#define NEW_RAYCAST
//#define PUBLISH_GRID

/**
 * Constructor
 */
but_env_model::COcFilterRaycast::COcFilterRaycast(const std::string & octree_frame_id, ERunMode mode /*= FILTER_ALLWAYS*/)
	: COcTreeFilterBase( octree_frame_id, mode )
	, m_bFilterInitialized(false)
	, m_camera_stereo_offset_left(128)
	, m_camera_stereo_offset_right(0)
	, m_bCamModelInitialized(false)
	, m_camera_info_topic(CAMERA_INFO_TOPIC_NAME)
	, m_numLeafsRemoved(0)
	, m_filtered_cloud( new tPointCloud())
	, m_miss_speedup( 2.0f )
	, m_max_sensor_range(7.0)
	, m_tree_resolution(0.1)
{

}

/**
 * Initialize. Must be called before first filtering
 */
void but_env_model::COcFilterRaycast::init(ros::NodeHandle & node_handle)
{

	// stereo cam params for sensor cone:
	node_handle.param<int> ("camera_stereo_offset_left",
			m_camera_stereo_offset_left, 0);
	node_handle.param<int> ("camera_stereo_offset_right",
			m_camera_stereo_offset_right, 0);

	// Used camera info topic
	node_handle.param("camera_info_topic", m_camera_info_topic,
					m_camera_info_topic);

#ifdef SHOW_VISUALIZATION
	std::cerr << "Initializing Raytrace filter visualization marker publisher" << std::endl;
	marker_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 5);
#endif

#ifdef PUBLISH_GRID
	grid_pub_ = node_handle.advertise<sensor_msgs::PointCloud2> ("grid_cloud", 5);
#endif

	// Add camera info subscriber
	m_ciSubscriber = node_handle.subscribe(m_camera_info_topic, 1,
			&COcFilterRaycast::cameraInfoCB, this);
}

/**
 * Set input cloud. Must be called before filter call
 */
void but_env_model::COcFilterRaycast::setCloud(tPointCloudConstPtr cloud)
{
	assert( cloud != 0 );
	m_lockData.lock();
	m_cloudPtr = cloud;
	m_lockData.unlock();
}

//! Write some info about last filter run
void but_env_model::COcFilterRaycast::writeLastRunInfo()
{
	std::cerr << "COcFilterRaycast: Number of leafs tested: " << m_numLeafsTested << std::endl;
	std::cerr << "COcFilterRaycast: Number of leafs removed: " << m_numLeafsRemoved << std::endl;
	std::cerr << "COcFilterRaycast: Number of leafs out of cone: " << m_numLeafsOutOfCone << std::endl;
	std::cerr << "COcFilterRaycast: Number of leafs occluded by cloud: " << m_numLeafsOutOfMap << std::endl;
	std::cerr << "Camera position: " << m_sensor_origin.x() << ", " << m_sensor_origin.y() << ", " << m_sensor_origin.z() << std::endl;
}

/**
 * Camera ifo callback - initialize camera model
 */
void but_env_model::COcFilterRaycast::cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info) {

	boost::mutex::scoped_lock lock( m_lockCamera );

	// Get camera model from camera info
	m_camera_model.fromCameraInfo(*cam_info);
	m_camera_size = m_camera_model.fullResolution();
	m_sensor_header = cam_info->header;

	// HACK
	m_sensor_header.frame_id = "/head_cam3d_link";

	// Set flag
	m_bCamModelInitialized = true;
}

#ifndef NEW_RAYCAST
/**
 * Filtering function implementation
 */
void but_env_model::COcFilterRaycast::filterInternal( tButServerOcTree & tree )
{
	assert( m_cloudPtr != 0 );

	m_numLeafsRemoved = 0;
	m_numLeafsOutOfCone = 0;
	m_numLeafsOutOfMap = 0;
	m_numLeafsTested = 0;

	m_tree_resolution = tree.getResolution();

	//octomap::pose6d sensor_pose(m_sensor_origin.x(), m_sensor_origin.y(), m_sensor_origin.z(), 0, 0, 0);

	// Is camera model initialized?
	if (!m_bCamModelInitialized)
	{
		ROS_ERROR("ERROR: camera model not initialized.");
		return;
	}

	// Get sensor origin
	m_sensor_origin = getSensorOrigin(m_sensor_header);

	tf::StampedTransform trans;
	try
	{
		m_tfListener.lookupTransform(m_sensor_header.frame_id, m_treeFrameId, m_cloudPtr->header.stamp, trans); //m_cloudPtr->header.frame_id
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR_STREAM("ERROR: Cannot find transform: " <<  m_cloudPtr->header.frame_id << " -> " << m_sensor_header.frame_id);
		return;
	}

//	std::cerr << m_sensor_header.frame_id << " <-- " << m_cloudPtr->header.frame_id << std::cerr;

	tf::Transform to_sensor = trans;

	// compute bbx from sensor cone
	octomap::point3d min;
	octomap::point3d max;
	computeBBX(m_sensor_header, min, max);

	double resolution(tree.getResolution());
	float probMissLog(tree.getProbMissLog() * m_miss_speedup);
//	std::cerr << tree.getProbMissLog() << ", " << probMissLog << std::endl;

	boost::mutex::scoped_lock lock(m_lockCamera);

	unsigned query_time = time(NULL);
	unsigned max_update_time = 1;
	for (tButServerOcTree::leaf_bbx_iterator it =
			tree.begin_leafs_bbx(min, max), end = tree.end_leafs_bbx(); it
			!= end; ++it) {

		++m_numLeafsTested;

		if (tree.isNodeOccupied(*it) && ((query_time - it->getTimestamp())
				> max_update_time)) {
			tf::Point pos(it.getX(), it.getY(), it.getZ());
			tf::Point posRel = to_sensor(pos);
			cv::Point2d uv = m_camera_model.project3dToPixel(cv::Point3d(
					posRel.x(), posRel.y(), posRel.z()));

//			it->setColor(255, 255, 255);

			double distance(pos.distance(tf::Point(m_sensor_origin.x(), m_sensor_origin.y(), m_sensor_origin.z())));

			if( isOccludedRaw(uv, distance))
			{
//				it->setColor(220, 128, 128);
				++m_numLeafsOutOfMap;
				continue;
			}

			// ignore point if not in sensor cone
			if (!inSensorCone(uv))
			{
//				it->setColor(180, 128, 128);

				++m_numLeafsOutOfCone;
				continue;
			}


			// ignore point if it is occluded in the map
			if (isOccludedMap(m_sensor_origin, it.getCoordinate(), resolution, tree))
			{
//				it->setColor(230, 128, 128);
				continue;
			}

//			std::cerr << "5" << std::endl;

			// otherwise: degrade node
//			it->setColor(255, 0, 0);
			it->setValue(probMissLog);
//			tree.integrateMissNoTime(&(*it));
//			tree.updateNodeLogOdds(&(*it), probMissLog);

			++m_numLeafsRemoved;
		}
	}
}

#else

/**
 * Filtering function implementation
 */
void but_env_model::COcFilterRaycast::filterInternal( tButServerOcTree & tree )
{
	assert( m_cloudPtr != 0 );

	// Is camera model initialized?
	if (!m_bCamModelInitialized)
	{
		ROS_ERROR("ERROR: camera model not initialized.");
		return;
	}

	// compute bbx from sensor cone
	octomap::point3d min;
	octomap::point3d max;
	computeBBX(m_sensor_header, min, max);

	m_lockData.lock();

	m_numLeafsRemoved = 0;
	m_numLeafsOutOfCone = 0;

	// Get sensor origin
	m_sensor_origin = getSensorOrigin(m_sensor_header);

	float resolution(tree.getResolution());

	m_vgfilter.setLeafSize(resolution, resolution, resolution);
	m_vgfilter.setInputCloud(m_cloudPtr);

//	std::cerr << "Voxelgrid filter. Resolution: " << resolution << " Input cloud size: " << m_cloudPtr->size() << std::endl;
//	std::cerr << "Output cloud: " << m_filtered_cloud << ", size: " << m_filtered_cloud->size() << std::endl;

	m_vgfilter.filter(*m_filtered_cloud);

//	std::cerr << "SO: " << m_sensor_origin.x() << ", " << m_sensor_origin.y() << ", " << m_sensor_origin.z() << std::endl;


//	std::cerr << "Raytracing. Grid size: " << m_filtered_cloud->size() << std::endl;

#ifdef PUBLISH_GRID
	tPointCloud::Ptr ends_cloud(new tPointCloud );
#endif

	float probMissLog(tree.getProbMissLog() * m_miss_speedup);

	// For all points in cloud
	tPointCloud::VectorType::const_iterator it, itEnd( m_filtered_cloud->points.end());

	for( it = m_filtered_cloud->points.begin(); it != itEnd; ++it)
	{
		// Cast ray through octomap
		octomap::point3d direction(octomap::point3d(it->x, it->y, it->z) - m_sensor_origin);
//		std::cerr << "D: " << direction.x() << ", " << direction.y() << ", " << direction.z() << std::endl;

		octomap::point3d obstacle;
		double range = direction.norm() - resolution;

		octomap::point3d ray_end( m_sensor_origin + direction.normalized() * range);

//*
#ifdef PUBLISH_GRID
		tPclPoint point;
		point.x = ray_end.x(); point.y = ray_end.y(); point.z = ray_end.z();
		point.r = point.g = point.b = 255;

		ends_cloud->points.push_back(point);
#endif
//*/

		octomap::KeyRay keyray;

		if(tree.computeRayKeys(m_sensor_origin, ray_end, keyray))
		{
			// For all found cells
			octomap::KeyRay::const_iterator itc, itcEnd( keyray.end());
			for ( itc = keyray.begin(); itc != itcEnd; ++itc)
			{
				tButServerOcTree::NodeType * node(0);
				node = tree.search(*itc);

				if( node != 0 && tree.isNodeOccupied(node) && (!node->hasChildren()))
				{
					octomap::point3d point;
					tree.genCoords(*itc, tree.getTreeDepth(), point);
/*
#ifdef PUBLISH_GRID
		tPclPoint pclpoint;
		pclpoint.x = point.x(); pclpoint.y = point.y(); pclpoint.z = point.z();
		pclpoint.r = pclpoint.g = pclpoint.b = 255;

		ends_cloud->points.push_back(pclpoint);
#endif
*/
					tree.updateNodeLogOdds(node, probMissLog);
					node->setColor(255, 0, 0 );
/*
					float node_range( (point-m_sensor_origin).norm() );
					if( node_range > range)
						node->setColor(255, 0, 0 );
					else
						node->setColor(0, 255.0*float(node_range/(0.1 + range)), 0 );

					min = -(node_range-range) < min ? range : min;
					max = -(node_range-range) > max ? range : max;

*/
					++m_numLeafsRemoved;
				}
			}

		}

//*/

	}

#ifdef PUBLISH_GRID
	sensor_msgs::PointCloud2 cloud;
	// pcl::toROSMsg< tPclPoint >(*m_filtered_cloud, cloud);
	pcl::toROSMsg< tPclPoint >(*ends_cloud, cloud);

	// Set message parameters and publish
	cloud.header.frame_id = m_cloudPtr->header.frame_id;
	cloud.header.stamp = m_cloudPtr->header.stamp;

	grid_pub_.publish(cloud);
#endif

	m_lockData.unlock();
}

#endif
/**
 * Compute sensor origin from the header
 */
octomap::point3d but_env_model::COcFilterRaycast::getSensorOrigin(const std_msgs::Header& sensor_header)
{
	geometry_msgs::PointStamped stamped_in;
	geometry_msgs::PointStamped stamped_out;
	stamped_in.header = sensor_header;

	std::string fixed_frame_(m_treeFrameId);

	// HACK: laser origin
	if (sensor_header.frame_id == "base_footprint")
	{
		stamped_in.header.frame_id = "laser_tilt_link";
	}

	geometry_msgs::Point p;
	p.x = p.y = p.z = 0;
	try {
		m_tfListener.transformPoint(fixed_frame_, stamped_in, stamped_out);
	} catch (tf::TransformException& ex) {
		ros::Time t;
		std::string err_string;
		ROS_INFO_STREAM("Transforming sensor origin using latest common time because there's a tf problem");
		if (m_tfListener.getLatestCommonTime(fixed_frame_, stamped_in.header.frame_id,
				stamped_in.header.stamp, &err_string) == tf::NO_ERROR)
		{
			try {
				m_tfListener.transformPoint(fixed_frame_, stamped_in, stamped_out);
			} catch (...) {
				ROS_WARN_STREAM("Still can't transform sensor origin between " << fixed_frame_ << " and " << stamped_in.header.frame_id);
			}
		}
		else
		{
			ROS_WARN_STREAM("No common time between " << fixed_frame_ << " and " << stamped_in.header.frame_id);
		}
	}

	octomap::point3d retval(stamped_out.point.x, stamped_out.point.y,
			stamped_out.point.z);

	return retval;
}
/**
 * Is point in sensor cone?
 */
bool but_env_model::COcFilterRaycast::inSensorCone(const cv::Point2d& uv) const {
	// Check if projected 2D coordinate in pixel range.
	// This check is a little more restrictive than it should be by using
	// 1 pixel less to account for rounding / discretization errors.
	// Otherwise points on the corner are accounted to be in the sensor cone.
	return (	(uv.x > m_camera_stereo_offset_left + 1)
			&& 	(uv.x < m_camera_size.width + m_camera_stereo_offset_right - 2)
			&& 	(uv.y > 1)
			&& 	(uv.y < m_camera_size.height - 2));
}

/**
 * Return true, if occupied cell is between origin and p
 */
bool but_env_model::COcFilterRaycast::isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p, double resolution, tButServerOcTree & tree) const
{
	octomap::point3d direction(p - sensor_origin);
	octomap::point3d obstacle;
	double range = direction.norm() - resolution;

	if (tree.castRay(sensor_origin, direction, obstacle, true, range))
	{
		// fprintf(stderr, "<%.2f , %.2f , %.2f> -> <%.2f , %.2f , %.2f> // obs at: <%.2f , %.2f , %.2f>, range: %.2f\n",
		//         sensor_origin.x(), sensor_origin.y(), sensor_origin.z(),
		//         p.x(), p.y(), p.z(),
		//         obstacle.x(), obstacle.y(), obstacle.z(), (obstacle-p).norm());
		return true;
	}
	return false;
}

#define MIN(x, y) ((x) < (y) ? (x) : (y))
/**
 * Return true if point is occluded by pointcloud
 */
bool but_env_model::COcFilterRaycast::isOccludedRaw(const cv::Point2d& uv, double range)
{
//	if( !m_cloudPtr->isOrganized() )
//		return false;

	if ((uv.x < 0) || (uv.y < 0) || (uv.x > m_camera_size.width) || (uv.y > m_camera_size.height))
	{
//		std::cerr << uv.x << ", " << uv.y << " - " << m_camera_size.width << ", " << m_camera_size.height << std::endl;
		return false;
	}

	double sensor_range = MIN((*m_cloudPtr)(uv.x, uv.y).z, m_max_sensor_range);


//	std::cerr << sensor_range << ", " << range << std::endl;

	if( sensor_range != sensor_range )
		return false;

//	if( sensor_range < 0.0 )
//		return false;

	// Discretization step
	sensor_range -= m_tree_resolution;

	return (sensor_range < range);
}

/**
 * Compute boundig box
 */
void but_env_model::COcFilterRaycast::computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max)
{
	std::string sensor_frame = sensor_header.frame_id;

	//  transform sensor FOV
	geometry_msgs::PointStamped stamped_in;
	geometry_msgs::PointStamped stamped_out;
	stamped_in.header = sensor_header;
	stamped_in.header.frame_id = sensor_frame;

	// get max 3d points from camera at 0.5m and 5m.
	geometry_msgs::Point p[8];

	// define min/max 2d points
	cv::Point2d uv[4];
	uv[0].x = m_camera_stereo_offset_left;
	uv[0].y = 0;
	uv[1].x = m_camera_size.width + m_camera_stereo_offset_right;
	uv[1].y = 0;
	uv[2].x = m_camera_size.width + m_camera_stereo_offset_right;
	uv[2].y = m_camera_size.height;
	uv[3].x = m_camera_stereo_offset_left;
	uv[3].y = m_camera_size.height;

	// transform to 3d space
	cv::Point3d xyz[4];
	for (int i = 0; i < 4; i++) {
		xyz[i] = m_camera_model.projectPixelTo3dRay(uv[i]);
		cv::Point3d xyz_05 = xyz[i] * 0.5;
		xyz[i] *= 5.; // 5meters
		p[i].x = xyz[i].x;
		p[i].y = xyz[i].y;
		p[i].z = xyz[i].z;
		p[i + 4].x = xyz_05.x;
		p[i + 4].y = xyz_05.y;
		p[i + 4].z = xyz_05.z;
	}

	// transform to world coodinates and find axis-aligned bbx
	bbx_min.x() = bbx_min.y() = bbx_min.z() = 1e6;
	bbx_max.x() = bbx_max.y() = bbx_max.z() = -1e6;
	for (int i = 0; i < 8; i++) {
		stamped_in.point = p[i];
		m_tfListener.transformPoint(m_treeFrameId, stamped_in,
				stamped_out);
		p[i].x = stamped_out.point.x;
		p[i].y = stamped_out.point.y;
		p[i].z = stamped_out.point.z;
		if (p[i].x < bbx_min.x())
			bbx_min.x() = p[i].x;
		if (p[i].y < bbx_min.y())
			bbx_min.y() = p[i].y;
		if (p[i].z < bbx_min.z())
			bbx_min.z() = p[i].z;
		if (p[i].x > bbx_max.x())
			bbx_max.x() = p[i].x;
		if (p[i].y > bbx_max.y())
			bbx_max.y() = p[i].y;
		if (p[i].z > bbx_max.z())
			bbx_max.z() = p[i].z;
	}

#ifdef SHOW_VISUALIZATION
//	std::cerr << "Publishing visualization marker publisher" << std::endl;
	std::string fixed_frame_(m_treeFrameId);
	// // visualize axis-aligned querying bbx
	visualization_msgs::Marker bbx;
	bbx.header.frame_id = fixed_frame_;
	bbx.header.stamp = ros::Time::now();
	bbx.ns = "collider";
	bbx.id = 1;
	bbx.action = visualization_msgs::Marker::ADD;
	bbx.type = visualization_msgs::Marker::CUBE;
	bbx.pose.orientation.w = 1.0;
	bbx.pose.position.x = (bbx_min.x() + bbx_max.x()) / 2.;
	bbx.pose.position.y = (bbx_min.y() + bbx_max.y()) / 2.;
	bbx.pose.position.z = (bbx_min.z() + bbx_max.z()) / 2.;
	bbx.scale.x = bbx_max.x()-bbx_min.x();
	bbx.scale.y = bbx_max.y()-bbx_min.y();
	bbx.scale.z = bbx_max.z()-bbx_min.z();
	bbx.color.g = 1;
	bbx.color.a = 0.3;
	marker_pub_.publish(bbx);


	// visualize sensor cone
	visualization_msgs::Marker bbx_points;
	bbx_points.header.frame_id = fixed_frame_;
	bbx_points.header.stamp = ros::Time::now();
	bbx_points.ns = "collider";
	bbx_points.id = 2;
	bbx_points.action = visualization_msgs::Marker::ADD;
	bbx_points.type = visualization_msgs::Marker::LINE_STRIP;
	bbx_points.pose.orientation.w = 1.0;
	bbx_points.scale.x = 0.02;
	bbx_points.scale.y = 0.02;
	bbx_points.color.g = 1;
	bbx_points.color.a = 0.3;
	bbx_points.points.push_back(p[0]);
	bbx_points.points.push_back(p[1]);
	bbx_points.points.push_back(p[2]);
	bbx_points.points.push_back(p[3]);
	bbx_points.points.push_back(p[0]);
	bbx_points.points.push_back(p[4]);
	bbx_points.points.push_back(p[5]);
	bbx_points.points.push_back(p[6]);
	bbx_points.points.push_back(p[7]);
	bbx_points.points.push_back(p[4]);
	bbx_points.points.push_back(p[7]);
	bbx_points.points.push_back(p[3]);
	bbx_points.points.push_back(p[2]);
	bbx_points.points.push_back(p[6]);
	bbx_points.points.push_back(p[5]);
	bbx_points.points.push_back(p[1]);
	marker_pub_.publish(bbx_points);
#endif

}
