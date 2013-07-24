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
#ifndef OCTOMAP_FILTER_BASE_H_
#define OCTOMAP_FILTER_BASE_H_

#include <but_env_model/server_tools.h>
#include <boost/timer.hpp>

namespace but_env_model
{

class COcTreeFilterBase
{
public:
	//! Running mode
	enum ERunMode
	{
		FILTER_ALLWAYS,
		FILTER_TEST_FRAME,
		FILTER_TEST_TIME
	};

public:
	//! Constructor - set running mode
	COcTreeFilterBase( const std::string & octree_frame_id, ERunMode mode = FILTER_ALLWAYS );

	//! Set number of frames skipped between runs
	void setFrameSkip( unsigned skip );

	//! Set timer lap
	bool setTimerLap( double lap );

	//! Filter tree
	void filter( tButServerOcTree & tree, bool bPruneAfterFinish = true );

	//! Set filter running mode
	void setRunMode( ERunMode mode ) { m_mode = mode; }

	//! Set tree frame id
	void setTreeFrameId( const std::string & tree_frame_id ) { m_treeFrameId = tree_frame_id; }

	//! Write some info about last filter run
	virtual void writeLastRunInfo(){}

protected:
	//! Test if this frame should be used
	bool useFrame();

	//! Filtering function implementation
	virtual void filterInternal( tButServerOcTree & tree ){}

protected:
	//! Used running mode
	ERunMode m_mode;

	//! How many frames should be skipped between runs
	unsigned m_framesSkipped;

	//! Frames counter
	long m_framesCount;

	//! Timer
	boost::timer m_timer;

	//! Timer lap
	double m_lap;

	//! Tree frame id
	std::string m_treeFrameId;

}; // class COcTreeFilter

} // namespace but_env_model

#endif /* OCTOMAP_FILTER_BASE_H_ */
