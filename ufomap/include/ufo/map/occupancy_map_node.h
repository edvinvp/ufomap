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
 * Copyright (c) 2020, D. Duberg, E. von Platen, KTH Royal Institute of Technology
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

#ifndef UFO_MAP_OCCUPANCY_MAP_NODE_H
#define UFO_MAP_OCCUPANCY_MAP_NODE_H

// UFO
#include <ufo/map/code.h>
#include <ufo/map/color.h>
#include <ufo/map/octree_node.h>

// STD
#include <cstddef>  // std::byte
#include <map>

namespace ufo::map
{
using Intensity = uint8_t;

template <typename T>
struct OccupancyNode {
	T occupancy = 0;

	OccupancyNode() {}

	OccupancyNode(T occupancy) : occupancy(occupancy) {}

	bool operator==(OccupancyNode const& rhs) const { return rhs.occupancy == occupancy; }
	bool operator!=(OccupancyNode const& rhs) const { return rhs.occupancy != occupancy; }

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		s.write(reinterpret_cast<char const*>(&occupancy), sizeof(occupancy));
		return s;
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		s.read(reinterpret_cast<char*>(&occupancy), sizeof(occupancy));
		return s;
	}
};

struct ColorNode {
	Color color;

	ColorNode() {}

	ColorNode(Color const& color) : color(color) {}

	bool operator==(ColorNode const& rhs) const { return rhs.color == color; }
	bool operator!=(ColorNode const& rhs) const { return rhs.color != color; }

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		s.write(reinterpret_cast<char const*>(&color), sizeof(color));
		return s;
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		s.read(reinterpret_cast<char*>(&color), sizeof(color));
		return s;
	}
};

template <typename T>
struct ColorOccupancyNode : OccupancyNode<T>, ColorNode {
	ColorOccupancyNode() {}

	ColorOccupancyNode(T occupancy, Color const& color = Color())
	    : OccupancyNode<T>(occupancy), ColorNode(color)
	{
	}

	bool operator==(ColorOccupancyNode const& rhs) const
	{
		return OccupancyNode<T>::operator==(rhs) && ColorNode::operator==(rhs);
	}

	bool operator!=(ColorOccupancyNode const& rhs) const
	{
		return OccupancyNode<T>::operator!=(rhs) || ColorNode::operator!=(rhs);
	}

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return ColorNode::writeData(OccupancyNode<T>::writeData(s));
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return ColorNode::readData(OccupancyNode<T>::readData(s));
	}
};

struct SemanticColorNode {
	Color color;

	std::map<InstanceType, float> instances;

	SemanticColorNode() {}

	~SemanticColorNode() {}

	SemanticColorNode(Color const& color) : color(color) {}

	// TODO: Check instances
	// The constant false / true is to stop pruning of these nodes
	bool operator==(SemanticColorNode const& rhs) const
	{
		return rhs.color == color && instances.empty() && rhs.instances.empty();
	}
	bool operator!=(SemanticColorNode const& rhs) const
	{
		return !(rhs.color == color && instances.empty() && rhs.instances.empty());
	}

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		s.write(reinterpret_cast<char const*>(&color), sizeof(color));
		return s;
	}

	std::ostream& writeInstances(std::ostream& s) const
	{
		// Number of instances
		uint32_t num_instances = instances.size();
		s.write(reinterpret_cast<char*>(&num_instances), sizeof(num_instances));

		// The pair of instances (instance, prob)
		for (auto& [instance, prob] : instances) {
			s.write(reinterpret_cast<char const*>(&instance), sizeof(instance));
			s.write(reinterpret_cast<char const*>(&prob), sizeof(prob));
		}
		return s;
	}
	std::istream& readInstances(std::istream& s)
	{
		uint32_t num_instances;
		s.read(reinterpret_cast<char*>(&num_instances), sizeof(num_instances));
		instances.clear();

		for (uint32_t i = 0; i < num_instances; ++i) {
			float prob;
			InstanceType instance;
			s.read(reinterpret_cast<char*>(&instance), sizeof(instance));
			s.read(reinterpret_cast<char*>(&prob), sizeof(prob));
			auto t = instances.emplace(instance, prob);
		}
		return s;
	}
	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		s.read(reinterpret_cast<char*>(&color), sizeof(color));
		return s;
	}
};

template <typename T>
struct SemanticColorOccupancyNode : OccupancyNode<T>, SemanticColorNode {
	SemanticColorOccupancyNode() {}

	SemanticColorOccupancyNode(T occupancy, Color const& color = Color())
	    : OccupancyNode<T>(occupancy), SemanticColorNode(color)
	{
	}

	bool operator==(SemanticColorOccupancyNode const& rhs) const
	{
		return OccupancyNode<T>::operator==(rhs) && SemanticColorNode::operator==(rhs);
	}

	bool operator!=(SemanticColorOccupancyNode const& rhs) const
	{
		return OccupancyNode<T>::operator!=(rhs) || SemanticColorNode::operator!=(rhs);
	}

	/**
	 * @brief Write the data from this node to stream s
	 *
	 * @param s The stream to write the data to
	 * @return std::ostream&
	 */
	std::ostream& writeData(std::ostream& s) const
	{
		return SemanticColorNode::writeData(OccupancyNode<T>::writeData(s));
	}

	std::ostream& wrteInstances(std::ostream& s) const
	{
		return SemanticColorNode::writeInstances(s);
	}

	/**
	 * @brief Read the data for this node from stream s
	 *
	 * @param s The stream to read the data from
	 * @return std::istream&
	 */
	std::istream& readData(std::istream& s)
	{
		return SemanticColorNode::readData(OccupancyNode<T>::readData(s));
	}

	std::istream& readInstances(std::istream& s)
	{
		return SemanticColorNode::readInstances(s);
	}
};

template <typename T>
using OccupancyMapLeafNode = OctreeLeafNode<T>;

template <typename T>
struct OccupancyMapInnerNodeBase : OccupancyMapLeafNode<T> {
	// Indicates whether this node or any of its children contains free space
	bool contains_free;
	// Indicates whether this node or any of its children contains unknown space
	bool contains_unknown;
};

template <typename T>
using OccupancyMapInnerNode = OctreeInnerNodeBase<OccupancyMapInnerNodeBase<T>>;

template <typename T>
struct Node {
	OccupancyMapLeafNode<T> const* node;
	Code code;
};

}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_NODE_H