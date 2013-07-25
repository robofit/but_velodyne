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
 * Date: 25/1/2012
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

#include <but_env_model/octonode.h>

using namespace octomap;

/**
 * Constructor
 */
but_env_model::EModelTreeNode::EModelTreeNode() :
	octomap::OcTreeNodeStamped(), m_r(0), m_g(0), m_b(0), m_a(0) {

}

/**
 * Destructor
 */
but_env_model::EModelTreeNode::~EModelTreeNode() {

}

/**
 * Create child
 */
bool but_env_model::EModelTreeNode::createChild(unsigned int i) {
//	if (itsChildren == NULL) {
    if (children == NULL) {
		allocChildren();
	}
//	itsChildren[i] = new EModelTreeNode();
    children[i] = new EModelTreeNode();
	return true;
}

void but_env_model::EModelTreeNode::updateColorChildren() {
	setAverageChildColor();
}

bool but_env_model::EModelTreeNode::pruneNode() {
	// checks for equal occupancy only, color ignored
	if (!this->collapsible())
		return false;
	// set occupancy value
	setLogOdds(getChild(0)->getLogOdds());
	// set color to average color
	if (isColorSet()) {
		setAverageChildColor();
	}
	// delete children
	for (unsigned int i = 0; i < 8; i++) {
//		delete itsChildren[i];
        delete children[i];
	}
//    delete[] itsChildren;
//    itsChildren = NULL;
	delete[] children;
	children = NULL;
	return true;
}

void but_env_model::EModelTreeNode::expandNode() {
	assert(!hasChildren());

	// expand node, set children color same as node color
	for (unsigned int k = 0; k < 8; k++) {
		createChild(k);
//		itsChildren[k]->setValue(value);
        children[k]->setValue(value);
		getChild(k)->setColor(r(), g(), b(), a());
	}
}

void but_env_model::EModelTreeNode::setAverageChildColor() {
	int mr(0), mg(0), mb(0), ma(0);
	int c(0);
	for (int i = 0; i < 8; i++) {
		if (childExists(i) && getChild(i)->isColorSet()) {
			mr += getChild(i)->r();
			mg += getChild(i)->g();
			mb += getChild(i)->b();
			ma += getChild(i)->a();
			++c;
		}
	}
	if (c) {
		mr /= c;
		mg /= c;
		mb /= c;
		ma /= c;

		m_r = mr;
		m_g = mg;
		m_b = mb;
		m_a = ma;
		setColor((unsigned char) mr, (unsigned char) mg, (unsigned char) mb,
				(unsigned char) ma);
	} else { // no child had a color other than white
		setColor((unsigned char) 255, (unsigned char) 255, (unsigned char) 255,
				(unsigned char) 255);
	}
}

/**
 * Read node from binary stream (incl. float value),
 * recursively continue with all children.
 *
 * @param s
 * @return
 */
std::istream& but_env_model::EModelTreeNode::readValue(std::istream &s)
{
	// Read common data
	octomap::OcTreeNodeStamped::readValue( s );

	char children_char;

	// read data:
	s.read((char*) &m_r, sizeof(m_r));
	s.read((char*) &m_g, sizeof(m_g));
	s.read((char*) &m_b, sizeof(m_b));
	s.read((char*) &m_a, sizeof(m_a));

	s.read((char*)&children_char, sizeof(char));
	std::bitset<8> children ((unsigned long long) children_char);

	//std::cerr << "Read color: " << int(m_r) << ", " << int(m_g) << ", " << int(m_b) << ", " << int(m_a) << std::endl;

	// std::cout << "read: " << value << " "
	//           << children.to_string<char,std::char_traits<char>,std::allocator<char> >()
	//           << std::endl;

	for (unsigned int i=0; i<8; i++) {
	  if (children[i] == 1){
		createChild(i);
		getChild(i)->readValue(s);
	  }
	}
	return s;
}

/**
 * Write node to binary stream (incl float value),
 * recursively continue with all children.
 * This preserves the complete state of the node.
 *
 * @param s
 * @return
 */
std::ostream& but_env_model::EModelTreeNode::writeValue(std::ostream &s) const
{
	// 1 bit for each children; 0: empty, 1: allocated
	std::bitset<8> children;

	for (unsigned int i=0; i<8; i++) {
	  if (childExists(i))
		children[i] = 1;
	  else
		children[i] = 0;
	}

	octomap::OcTreeNodeStamped::writeValue(s);

	char children_char = (char) children.to_ulong();

	// Write node data
	s.write((const char*) &m_r, sizeof(m_r));
	s.write((const char*) &m_g, sizeof(m_g));
	s.write((const char*) &m_b, sizeof(m_b));
	s.write((const char*) &m_a, sizeof(m_a));

//	std::cerr << "Writing node" << std::endl;

	s.write((char*)&children_char, sizeof(char));

	// std::cout << "wrote: " << value << " "
	//           << children.to_string<char,std::char_traits<char>,std::allocator<char> >()
	//           << std::endl;

	// write children's children
	for (unsigned int i=0; i<8; i++) {
	  if (children[i] == 1) {
		this->getChild(i)->writeValue(s);
	  }
	}
	return s;
}


/**
 * Creates a new (empty) OcTree of a given resolution
 * @param _resolution
 */
but_env_model::EMOcTree::EMOcTree(double _resolution) :
	octomap::OccupancyOcTreeBase<but_env_model::EModelTreeNode> (_resolution) {
//	itsRoot = new EModelTreeNode();
    root = new EModelTreeNode();
	tree_size++;
}

/**
 * Reads an OcTree from a binary file
 * @param _filename
 *
 */
but_env_model::EMOcTree::EMOcTree(std::string _filename) :
	OccupancyOcTreeBase<but_env_model::EModelTreeNode> (0.1) { // resolution will be set according to tree file
//	itsRoot = new EModelTreeNode();
    root = new EModelTreeNode();
	tree_size++;

	readBinary(_filename);
}

but_env_model::EModelTreeNode* but_env_model::EMOcTree::setNodeColor(const octomap::OcTreeKey& key,
		const unsigned char& r, const unsigned char& g, const unsigned char& b,
		const unsigned char& a) {
	EModelTreeNode* n = search(key);
	if (n != 0) {
		n->setColor(r, g, b, a);
	}
	return n;
}

but_env_model::EModelTreeNode* but_env_model::EMOcTree::averageNodeColor(
		const octomap::OcTreeKey& key, const unsigned char& r, const unsigned char& g,
		const unsigned char& b, const unsigned char& a) {
	EModelTreeNode* n = search(key);
	if (n != 0) {
		if (n->isColorSet()) {
			// get previous color
			unsigned char prev_color_r = n->r();
			unsigned char prev_color_g = n->g();
			unsigned char prev_color_b = n->b();
			unsigned char prev_color_a = n->a();

			// average it with new color and set
			n->setColor((prev_color_r + r) / 2, (prev_color_g + g) / 2,
					(prev_color_b + b) / 2, (prev_color_a + a) / 2);
		} else {
			// nothing to average with
			n->setColor(r, g, b, a);
		}
	}
	return n;
}

but_env_model::EModelTreeNode* but_env_model::EMOcTree::integrateNodeColor(
		const octomap::OcTreeKey& key, const unsigned char& r, const unsigned char& g,
		const unsigned char& b, const unsigned char& a) {
	EModelTreeNode* n = search(key);
	if (n != 0) {
		if (n->isColorSet()) {
			// get previous color
			unsigned char prev_color_r = n->r();
			unsigned char prev_color_g = n->g();
			unsigned char prev_color_b = n->b();
			unsigned char prev_color_a = n->a();

			// average it with new color taking account of node probability
			double node_prob = n->getOccupancy();
			unsigned char new_r = (unsigned char) ((double) prev_color_r
					* node_prob + (double) r * (0.99 - node_prob));
			unsigned char new_g = (unsigned char) ((double) prev_color_g
					* node_prob + (double) g * (0.99 - node_prob));
			unsigned char new_b = (unsigned char) ((double) prev_color_b
					* node_prob + (double) b * (0.99 - node_prob));
			unsigned char new_a = (unsigned char) ((double) prev_color_a
					* node_prob + (double) a * (0.99 - node_prob));
			n->setColor(new_r, new_g, new_b, new_a);
		} else {
			n->setColor(r, g, b, a);
		}
	}
	return n;
}

void but_env_model::EMOcTree::updateInnerOccupancy() {
//	this->updateInnerOccupancyRecurs(this->itsRoot, 0);
    this->updateInnerOccupancyRecurs(this->root, 0);
}

void but_env_model::EMOcTree::updateInnerOccupancyRecurs(EModelTreeNode* node,
		unsigned int depth) {
	// only recurse and update for inner nodes:
	if (node->hasChildren()) {
		// return early for last level:
		if (depth < this->tree_depth) {
			for (unsigned int i = 0; i < 8; i++) {
				if (node->childExists(i)) {
					updateInnerOccupancyRecurs(node->getChild(i), depth + 1);
				}
			}
		}
		node->updateOccupancyChildren();
		node->updateColorChildren();
	}
}

unsigned int but_env_model::EMOcTree::getLastUpdateTime() {
	// this value is updated whenever inner nodes are
	// updated using updateOccupancyChildren()
//	return itsRoot->getTimestamp();
    return root->getTimestamp();
}

void but_env_model::EMOcTree::degradeOutdatedNodes(unsigned int time_thres) {
	unsigned int query_time = (unsigned int) time(NULL);

	for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it
			!= end; ++it) {
		if (this->isNodeOccupied(*it) && ((query_time - it->getTimestamp())
				> time_thres)) {
			integrateMissNoTime(&*it);
		}
	}
}

void but_env_model::EMOcTree::updateNodeLogOdds(EModelTreeNode* node,
		const float& update) const {
	OccupancyOcTreeBase<EModelTreeNode>::updateNodeLogOdds(node, update);
	node->updateTimestamp();
}

void but_env_model::EMOcTree::integrateMissNoTime(EModelTreeNode* node) const {
//	OccupancyOcTreeBase<EModelTreeNode>::updateNodeLogOdds(node, probMissLog);
    OccupancyOcTreeBase<EModelTreeNode>::updateNodeLogOdds(node, prob_miss_log);
}

void but_env_model::EMOcTree::insertColoredScan(const typePointCloud& coloredScan,
		const octomap::point3d& sensor_origin, double maxrange, bool pruning,
		bool lazy_eval) {

	// convert colored scan to octomap pcl
	octomap::Pointcloud scan;
	octomap::pointcloudPCLToOctomap(coloredScan, scan);

	// insert octomap pcl to count new probabilities
	octomap::KeySet free_cells, occupied_cells;
	computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);

	// insert data into tree  -----------------------
	for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
		updateNode(*it, false, lazy_eval);
	}
	for (octomap::KeySet::iterator it = occupied_cells.begin(); it
			!= occupied_cells.end(); ++it) {
		updateNode(*it, true, lazy_eval);
	}

	// TODO: does pruning make sense if we used "lazy_eval"?
	if (pruning)
		this->prune();

	// update node colors
	BOOST_FOREACH (const pcl::PointXYZRGB& pt, coloredScan.points)
	{
		// averageNodeColor or integrateNodeColor can be used to count final color
		averageNodeColor(pt.x, pt.y, pt.z, (unsigned char)pt.r, (unsigned char)pt.g, (unsigned char)pt.b, (unsigned char)255);
	}

}

/// to ensure static initialization (only once)
but_env_model::EMOcTree::StaticMemberInitializer but_env_model::EMOcTree::ocEMOcTreeMemberInit;

