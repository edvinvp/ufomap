/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, Edvin von Platen, KTH Royal Institute of Technology, Copyright (c)
 * 2021.
 * @see https://github.com/edvinvp/ufomap
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, E. von Platen KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_ITERATOR_SEMANTIC_OCCUPANCY_MAP_H
#define UFO_MAP_ITERATOR_SEMANTIC_OCCUPANCY_MAP_H

#include <ufo/map/iterator/occupancy_map.h>
#include <ufo/map/occupancy_map_node.h>

namespace ufo::map
{
template <typename TREE, typename DATA_TYPE, typename INNER_NODE, typename LEAF_NODE,
          bool ONLY_LEAF>
class SemanticOccupancyMapIterator
    : public OccupancyMapIterator<TREE, DATA_TYPE, INNER_NODE, LEAF_NODE, ONLY_LEAF>
{
 private:
	using Base = OccupancyMapIterator<TREE, DATA_TYPE, INNER_NODE, LEAF_NODE, ONLY_LEAF>;
	using IteratorNode = typename Base::IteratorNode;
	using INNER_NODE_TYPE = OccupancyMapInnerNode<DATA_TYPE>;

	float instance_log_prob_threshold_;
	DepthType instance_depth_;   // minimum depth instances are stored at
	DepthType query_depth_ = 5;  // want nodes from this depth
	std::vector<InstanceType>
	    instances_;  // Nodes extracted should contain atleast one of these instances

 public:
	SemanticOccupancyMapIterator() {}

	SemanticOccupancyMapIterator(TREE const* tree, INNER_NODE const& root,
	                             ufo::geometry::BoundingVolume const& bounding_volume,
	                             std::vector<uint32_t> const& instances,
	                             DepthType instance_depth, DepthType query_depth,
	                             float instance_prob_log_threshold,
	                             bool occupied_space = true, bool free_space = true,
	                             bool unknown_space = false, bool contains = false,
	                             DepthType min_depth = 0)
	    : Base(tree, bounding_volume, occupied_space, free_space, unknown_space, contains,
	           min_depth),
	      instances_(instances),
	      instance_depth_(instance_depth),
	      query_depth_(query_depth),
	      instance_log_prob_threshold_(instance_prob_log_threshold)
	{
		Base::Base::init(root);  // called from Base(...)
	}

	SemanticOccupancyMapIterator(SemanticOccupancyMapIterator const& other)
	    : Base(other),
	      instances_(other.instances_),
	      instance_depth_(other.instance_depth_),
	      query_depth_(other.query_depth_),
	      instance_log_prob_threshold_(other.instance_log_prob_threshold_)
	{
	}

	SemanticOccupancyMapIterator& operator=(SemanticOccupancyMapIterator const& rhs)
	{
		Base::operator=(rhs);
		instances_ = rhs.instances_;
		instance_depth_ = rhs.instance_depth_;
		query_depth_ = rhs.query_depth_;
		instance_log_prob_threshold_ = rhs.instance_log_prob_threshold_;
		return *this;
	}

	bool operator==(SemanticOccupancyMapIterator const& rhs) const
	{
		bool i_depth = instance_depth_ == rhs.instance_depth_;
		bool q_depth = query_depth_ == rhs.query_depth_;
		return Base::operator==(rhs) && i_depth && q_depth;
	}

	bool operator!=(SemanticOccupancyMapIterator const& rhs) const
	{
		bool i_depth = instance_depth_ == rhs.instance_depth_;
		bool q_depth = query_depth_ == rhs.query_depth_;
		return Base::operator!=(rhs) && !(i_depth && q_depth);
	}

	// Postfix increment
	SemanticOccupancyMapIterator operator++(int)
	{
		SemanticOccupancyMapIterator result = *this;
		++(*this);
		return result;
	}

	// Prefix increment
	SemanticOccupancyMapIterator& operator++()
	{
		Base::Base::increment();
		return *this;
	}

 protected:
	virtual bool validNode(IteratorNode& node, unsigned int depth) const override
	{
		if (!Base::validNode(node, depth) || query_depth_ > depth) {
			return false;
		}
		INNER_NODE const& inner_node = static_cast<INNER_NODE const&>(*(node.node));

		if (depth >= instance_depth_) {
			if (inner_node.value.instances.empty()) {
				return false;
			}

			for (InstanceType const instance : instances_) {
				// Octree leaf base -> semantic leaf -> inner node
				auto contains = inner_node.value.instances.find(instance);

				if (contains != inner_node.value.instances.end()) {
					if (contains->second > instance_log_prob_threshold_) {
						return true;
					}
				}
			}
			return false;
		} else {
			// query_depth_ < depth && depth < instance_depth_
			// i.e. the ancestor of this node at instance_depth_ contains the wanted instance,
			// so this is a valid node.
			return true;
		}
	}

	virtual bool validReturnNode() const override
	{
		if (!Base::validReturnNode() || Base::Base::current_depth_ != query_depth_) {
			return false;
		}

		return true;
	}
};
}  // Namespace ufo::map
#endif