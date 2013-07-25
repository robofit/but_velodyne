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

#pragma once
#ifndef OCTOMAP_FILTER_GROUND_H_
#define OCTOMAP_FILTER_GROUND_H_

#include "octomap_filter_base.h"

namespace but_env_model
{
/**
 * Filter single specles from the octree
 */
class COcFilterGround : public COcTreeFilterBase
{
public:
	//! Constructor
	COcFilterGround(const std::string & octree_frame_id, ERunMode mode = FILTER_ALLWAYS);

	//! Write some info about last filter run
	virtual void writeLastRunInfo()
	{
		std::cerr << "COcFilterGround: Ground cloud size: " << m_groundPc->points.size()<< std::endl;
		std::cerr << "COcFilterGround: Non ground cloud size: " << m_nongroundPc->points.size()<< std::endl;
	}

	//! Initialize. Must be called before first filtering
	virtual void init(ros::NodeHandle & node_handle);

	//! Configure filter before each frame. Set input cloud.
	void setCloud(const tPointCloud * cloud);

	//! Get ground cloud
	tPointCloud * getGroundPc() { return m_groundPc; }

	//! Get non ground cloud
	tPointCloud * getNongroundPc() { return m_nongroundPc; }

protected:
	//! Filtering function implementation
	virtual void filterInternal(tButServerOcTree & tree);

protected:
	//! Number of specles removed
	long m_numSpecRemoved;

	//! Input point cloud
	const tPointCloud * m_inputPc;

	//! Ground cloud
	tPointCloud * m_groundPc;

	//! Nonground cloud
	tPointCloud * m_nongroundPc;

	double m_groundFilterDistance;
	double m_groundFilterAngle;
	double m_groundFilterPlaneDistance;
};

} // namespace but_env_model


#endif /* OCTOMAP_FILTER_GROUND_H_ */
