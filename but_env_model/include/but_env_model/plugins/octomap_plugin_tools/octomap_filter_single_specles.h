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
#ifndef OCTOMAP_FILTER_SINGLE_SPECLES_H_
#define OCTOMAP_FILTER_SINGLE_SPECLES_H_

#include "octomap_filter_base.h"

namespace but_env_model
{
/**
 * Filter single specles from the octree
 */
class COcFilterSingleSpecles : public COcTreeFilterBase
{
public:
	//! Constructor
	COcFilterSingleSpecles(const std::string & octree_frame_id, ERunMode mode = FILTER_ALLWAYS) : COcTreeFilterBase(octree_frame_id,mode), m_numSpecRemoved(0) {}

	//! Write some info about last filter run
	virtual void writeLastRunInfo()
	{
		std::cerr << "COcFilterSingleSpecles: Number of specles removed: " << m_numSpecRemoved << std::endl;
	}

protected:
	//! Filtering function implementation
	virtual void filterInternal(tButServerOcTree & tree)
	{
		m_numSpecRemoved = 0;

		tButServerOcTree::leaf_iterator it, itEnd(tree.end_leafs());
		for (it = tree.begin_leafs(); it != itEnd; ++it)
		{
			// Test if node is occupied
			if (tree.isNodeOccupied(*it))
			{
				octomap::OcTreeKey nKey = it.getKey();
				octomap::OcTreeKey key;
				bool neighborFound = false;

				// Find neighbours
				for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2]
						+ 1; ++key[2]) {
					for (key[1] = nKey[1] - 1; !neighborFound && key[1]
							<= nKey[1] + 1; ++key[1]) {
						for (key[0] = nKey[0] - 1; !neighborFound && key[0]
								<= nKey[0] + 1; ++key[0]) {
							if (key != nKey) {
								tButServerOcTree::NodeType* node = tree.search(
										key);
								if (node && tree.isNodeOccupied(node)) {
									// we have a neighbor => break!
									neighborFound = true;
								}
							}
						}
					}
				}

				// done with search, see if found and degrade otherwise:
				if (!neighborFound) {
					// Remove it...
					tree.integrateMissNoTime(&*it);
					++m_numSpecRemoved;
				}

			}
		}
	}

protected:
	//! Number of specles removed
	long m_numSpecRemoved;
};

} // namespace but_env_model

#endif /* OCTOMAP_FILTER_SINGLE_SPECLES_H_ */
