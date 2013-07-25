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

#include <but_env_model/plugins/octomap_plugin_tools/octomap_filter_base.h>

but_env_model::COcTreeFilterBase::COcTreeFilterBase( const std::string & octree_frame_id, ERunMode mode /*= FILTER_ALLWAYS*/ )
: m_mode(mode)
, m_framesSkipped(0)
, m_framesCount(0)
, m_lap( m_timer.elapsed_min() )
, m_treeFrameId( octree_frame_id )
{
	m_timer.restart();
}

/**
 * Set number of frames skipped between runs
 */
void but_env_model::COcTreeFilterBase::setFrameSkip( unsigned skip )
{
	m_framesSkipped = skip;
}

/**
 * Set timer lap
 * \return true if lap can be measured and is set
 */
bool but_env_model::COcTreeFilterBase::setTimerLap( double lap )
{
	if( lap > m_timer.elapsed_min() && lap < m_timer.elapsed_max() )
	{
		m_lap = lap;
		m_timer.restart();
		return true;
	}

	return false;
}

/**
 * Filter tree
 */
void but_env_model::COcTreeFilterBase::filter( tButServerOcTree & tree, bool bPruneAfterFinish /*= true*/ )
{
	++m_framesCount;

	if( useFrame() )
	{
		filterInternal( tree );
		if(bPruneAfterFinish)
			tree.prune();
	}
}

/**
 * Test if this frame should be used
 */
bool but_env_model::COcTreeFilterBase::useFrame()
{
	switch( m_mode )
	{
	case FILTER_ALLWAYS:
//		std::cerr << "Allways" << std::endl;
		return true;

	case FILTER_TEST_FRAME:
//		std::cerr << "Frame" << std::endl;
		if( m_framesCount > m_framesSkipped )
		{
			m_framesCount = 0;
			return true;
		}

		return false;

	case FILTER_TEST_TIME:
//		std::cerr << "Time. Elapsed: " << m_timer.elapsed() << std::endl;
		if( m_timer.elapsed() > m_lap)
		{
//			std::cerr << "Time elapsed: " << m_timer.elapsed() << ", lap: " << m_lap << std::endl;
			m_timer.restart();
			return true;
		}

		return false;

	default:
		return false;
	}

	return false;
}
