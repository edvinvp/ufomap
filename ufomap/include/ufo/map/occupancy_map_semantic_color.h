/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
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

#ifndef UFO_MAP_OCCUPANCY_MAP_SEMANTIC_COLOR_H
#define UFO_MAP_OCCUPANCY_MAP_SEMANTIC_COLOR_H

#include <ufo/map/iterator/semantic_occupancy_map.h>
#include <ufo/map/occupancy_map_base.h>

#include <map>
#include <set>

namespace ufo::map
{
class OccupancyMapSemanticColor
    : public OccupancyMapBase<SemanticColorOccupancyNode<float>>

{
 private:
	using DATA_TYPE = SemanticColorOccupancyNode<float>;
	using Base = OccupancyMapBase<DATA_TYPE>;

	LogitType prob_instance_inc_log_;
	LogitType prob_instance_dec_log_;
	DepthType instance_depth_;

 protected:
	using OccupancyMapInstanceIterator =
	    SemanticOccupancyMapIterator<Base, DATA_TYPE, INNER_NODE, LEAF_NODE, false>;

 public:
	//
	// Constructors
	//
	OccupancyMapSemanticColor(double resolution, DepthType depth_levels = 16,
	                          DepthType instance_depth = 0, bool automatic_pruning = true,
	                          double prob_instance_inc = 0.7,
	                          double prob_instance_dec = 0.3, double occupied_thres = 0.5,
	                          double free_thres = 0.5, double prob_hit = 0.7,
	                          double prob_miss = 0.4, double clamping_thres_min = 0.1192,
	                          double clamping_thres_max = 0.971);

	OccupancyMapSemanticColor(std::string const& filename, bool automatic_pruning = true,
	                          double prob_instance_inc = 0.7,
	                          double prob_instance_dec = 0.3, double occupied_thres = 0.5,
	                          double free_thres = 0.5, double prob_hit = 0.7,
	                          double prob_miss = 0.4, double clamping_thres_min = 0.1192,
	                          double clamping_thres_max = 0.971);

	OccupancyMapSemanticColor(OccupancyMapSemanticColor const& other);

	//
	// Destructor
	//

	virtual ~OccupancyMapSemanticColor() {}

	//
	// Tree Type
	//

	virtual std::string getTreeType() const noexcept override
	{
		return "occupancy_map_semantic_color";
	}

	// Calculate memory consumption of the map
	// Return <usage, inner node count, leaf node count>
	std::tuple<unsigned int, unsigned int, unsigned int> getMemoryUsage()
	{
		unsigned int inner_node_count = 0u;
		unsigned int leaf_node_count = 0u;
		unsigned int mem_size = 0u;
		unsigned int instance_prob_bytes = sizeof(InstanceType) + sizeof(float);
		for (auto it = beginTree(), it_end = endTree(); it != it_end; ++it) {
			if (it.getDepth() == 0) {
				leaf_node_count++;
				mem_size += sizeof(LEAF_NODE) +
				            it->instances.size() *
				                (sizeof(std::_Rb_tree_node_base) + instance_prob_bytes);
			} else {
				inner_node_count++;
				mem_size += sizeof(INNER_NODE) +
				            it->instances.size() *
				                (sizeof(std::_Rb_tree_node_base) + instance_prob_bytes);
			}
		}
		return std::make_tuple(mem_size, inner_node_count, leaf_node_count);
	}

	//
	// Get instances of node at depth and pos
	//
	const std::map<InstanceType, float>& getInstances(Point3 pos, DepthType depth)
	{
		depth = std::max<DepthType>(depth, instance_depth_);
		const Code instanceCode = Base::toCode(Base::toKey(pos, depth));
		const auto& node_pair = Base::getNode(instanceCode);
		return node_pair.first->value.instances;
	}

	const std::map<InstanceType, float>& getInstancesInMap() const
	{
		auto& root = getRoot();
		return root.value.instances;
	}

	//
	// Does node at depth and pos have instance x
	//
	std::pair<bool, DepthType> hasInstance(Point3 pos, DepthType depth,
	                                       InstanceType instance, LogitType prob_threshold)
	{
		depth = std::max<DepthType>(depth, instance_depth_);
		const Code instanceCode = Base::toCode(Base::toKey(pos, depth));
		const auto& node_pair = Base::getNode(instanceCode);
		auto search = node_pair.first->value.instances.find(instance);
		if (search != node_pair.first->value.instances.end()) {
			if (node_pair.first->value.instances[instance] >= prob_threshold) {
				return std::make_pair(true, node_pair.second);
			}
		}
		return std::make_pair(false, node_pair.second);
	}
	//
	// Iterator overload if no instance depth argument.
	//
	OccupancyMapInstanceIterator beginInstanceTree(std::vector<uint32_t> const& instances,
	                                               bool occupied_space = true,
	                                               bool free_space = true,
	                                               bool unknown_space = false,
	                                               bool contains = false,
	                                               DepthType min_depth = 0) const noexcept
	{
		return beginInstanceTree(instances, instance_depth_, instance_depth_, 2.5,
		                         occupied_space, free_space, unknown_space, contains,
		                         min_depth);
	}

	//
	// "Normal" iterators
	//

	OccupancyMapBasereeIterator beginTree(bool occupied_space = true,
	                                      bool free_space = true,
	                                      bool unknown_space = false, bool contains = false,
	                                      DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapBasereeIterator(this, Base::Base::getRoot(),
		                                   ufo::geometry::BoundingVolume(), occupied_space,
		                                   free_space, unknown_space, contains, min_depth);
	}

	OccupancyMapBasereeIterator endTree() const noexcept
	{
		return OccupancyMapBasereeIterator();
	}

	//
	// Iterator
	//
	OccupancyMapInstanceIterator beginInstanceTree(
	    std::vector<uint32_t> const& instances, DepthType instance_depth,
	    DepthType query_depth, double instance_prob_threshold, bool occupied_space = true,
	    bool free_space = true, bool unknown_space = false, bool contains = false,
	    DepthType min_depth = 0) const noexcept
	{
		return OccupancyMapInstanceIterator(
		    this, Base::getRoot(), ufo::geometry::BoundingVolume(), instances, instance_depth,
		    query_depth, toLogit(instance_prob_threshold), occupied_space, free_space,
		    unknown_space, contains, min_depth);
	}

	OccupancyMapInstanceIterator endInstanceTree() const noexcept
	{
		return OccupancyMapInstanceIterator();
	}

	//
	// Integration
	//

	template <typename T>
	void insertPointCloud(Point3 const& sensor_origin, T const& cloud,
	                      double max_range = -1, DepthType depth = 0,
	                      bool simple_ray_casting = false, unsigned int early_stopping = 0,
	                      bool async = false)
	{
		if constexpr (std::is_same_v<T, PointCloud>) {
			Base::insertPointCloud(sensor_origin, cloud, max_range, depth, simple_ray_casting,
			                       early_stopping, async);

			if constexpr (std::is_same_v<T, PointCloudColor>) {
				integrateColors(sensor_origin, cloud, max_range);
			}
		} else if constexpr (std::is_same_v<T, PointCloudColor>) {
			std::vector<std::tuple<Code, float, Color>> occupied_hits;
			occupied_hits.reserve(cloud.size());
			PointCloud discretized;
			discretized.reserve(cloud.size());
			Point3 min_change = Base::getMax();
			Point3 max_change = Base::getMin();
			for (Point3Color& end_color : cloud) {
				Point3 end = end_color;
				Point3 origin = sensor_origin;
				Point3 direction = (end - origin);
				double distance = direction.norm();

				// Move origin and end inside BBX
				if (!Base::moveLineInside(origin, end)) {
					// Line outside of BBX
					continue;
				}

				if (0 > max_range || distance <= max_range) {
					// Occupied space
					Code end_code = Base::toCode(end);
					if (indices_.insert(end_code).second) {
						occupied_hits.push_back(
						    std::make_tuple(end_code, prob_hit_log_, end_color.getColor()));
					}
				} else {
					direction /= distance;
					end = origin + (direction * max_range);
				}

				discretized.push_back(end);

				for (int i : {0, 1, 2}) {
					min_change[i] = std::min(min_change[i], std::min(end[i], origin[i]));
					max_change[i] = std::max(max_change[i], std::max(end[i], origin[i]));
				}
			}

			LogitType prob_miss_log = prob_miss_log_ / double((2.0 * depth) + 1);

			indices_.clear();

			if (integrate_.valid()) {
				integrate_.wait();
			}

			if (async) {
				integrate_ = std::async(
				    std::launch::async, &OccupancyMapSemanticColor::insertPointCloudHelper, this,
				    sensor_origin, std::move(discretized), std::move(occupied_hits),
				    prob_miss_log, depth, simple_ray_casting, early_stopping, min_change,
				    max_change);
			} else {
				insertPointCloudHelper(sensor_origin, std::move(discretized),
				                       std::move(occupied_hits), prob_miss_log, depth,
				                       simple_ray_casting, early_stopping, min_change,
				                       max_change);
			}
		}
	}

	template <typename T>
	void insertPointCloud(Point3 const& sensor_origin, T cloud,
	                      math::Pose6 const& frame_origin, double max_range = -1,
	                      DepthType depth = 0, bool simple_ray_casting = false,
	                      unsigned int early_stopping = 0, bool async = false)
	{
		cloud.transform(frame_origin, async);
		insertPointCloud(sensor_origin, cloud, max_range, depth, simple_ray_casting,
		                 early_stopping, async);
	}

	template <typename T>
	void insertPointCloudDiscrete(Point3 const& sensor_origin, T const& cloud,
	                              double max_range = -1, DepthType depth = 0,
	                              bool simple_ray_casting = false,
	                              unsigned int early_stopping = 0, bool async = false)
	{
		if constexpr (std::is_same_v<T, PointCloud>) {
			Base::insertPointCloudDiscrete(sensor_origin, cloud, max_range, depth,
			                               simple_ray_casting, early_stopping);
		} else if constexpr (std::is_same_v<T, PointCloudColor>) {
			double squared_max_range = max_range * max_range;

			std::vector<std::tuple<Code, float, Color>> occupied_hits;
			occupied_hits.reserve(cloud.size());
			PointCloud discretized;
			discretized.reserve(cloud.size());
			Point3 min_change = Base::getMax();
			Point3 max_change = Base::getMin();
			for (Point3Color const& end_color : cloud) {
				Point3 end = end_color;
				double dist_sqrt = (end - sensor_origin).squaredNorm();
				if (0 > max_range || dist_sqrt < squared_max_range) {
					if (Base::isInside(end)) {
						Code end_code = Base::toCode(end);
						if (!indices_.insert(end_code).second) {
							continue;
						}
						occupied_hits.push_back(
						    std::make_tuple(end_code, prob_hit_log_, end_color.getColor()));
					}
				} else {
					Point3 direction = Base::toCoord(Base::toKey(end, depth)) - sensor_origin;
					dist_sqrt = direction.squaredNorm();
					if (0 <= max_range && dist_sqrt > squared_max_range) {
						direction /= std::sqrt(dist_sqrt);
						end = sensor_origin + (direction * max_range);
					}
				}
				Point3 current = sensor_origin;
				// Move origin and end inside map
				if (!Base::moveLineInside(current, end)) {
					// Line outside of map
					continue;
				}

				Key end_key = Base::toKey(end, depth);

				if (!indices_.insert(Base::toCode(end_key)).second) {
					continue;
				}

				Point3 end_coord = Base::toCoord(end_key);

				discretized.push_back(end_coord);

				// Min/max change detection
				Point3 current_center = Base::toCoord(Base::toKey(current, depth));
				Point3 end_center = end_coord;

				double temp = Base::getNodeHalfSize(depth);
				for (int i : {0, 1, 2}) {
					min_change[i] = std::min(
					    min_change[i], std::min(end_center[i] - temp, current_center[i] - temp));
					max_change[i] = std::max(
					    max_change[i], std::max(end_center[i] + temp, current_center[i] + temp));
				}
			}

			LogitType prob_miss_log = prob_miss_log_ / double((2.0 * depth) + 1);

			indices_.clear();

			if (integrate_.valid()) {
				integrate_.wait();
			}

			if (async) {
				integrate_ = std::async(
				    std::launch::async, &OccupancyMapSemanticColor::insertPointCloudHelper, this,
				    sensor_origin, std::move(discretized), std::move(occupied_hits),
				    prob_miss_log, depth, simple_ray_casting, early_stopping, min_change,
				    max_change);
			} else {
				insertPointCloudHelper(sensor_origin, std::move(discretized),
				                       std::move(occupied_hits), prob_miss_log, depth,
				                       simple_ray_casting, early_stopping, min_change,
				                       max_change);
			}
		} else if constexpr (std::is_same_v<T, PointCloudSemanticColor>) {
			double squared_max_range = max_range * max_range;

			std::vector<std::tuple<Code, float, Color>> occupied_hits;
			occupied_hits.reserve(cloud.size());
			PointCloud discretized;
			discretized.reserve(cloud.size());
			Point3 min_change = Base::getMax();
			Point3 max_change = Base::getMin();
			for (Point3ColorInstance const& end_color_instance : cloud) {
				Point3 end = end_color_instance;
				double dist_sqrt = (end - sensor_origin).squaredNorm();
				if (0 > max_range || dist_sqrt < squared_max_range) {
					if (Base::isInside(end)) {
						Code end_code = Base::toCode(end);

						// Store multiple instances
						const Code instanceCode = Base::toCode(Base::toKey(end, instance_depth_));
						instanceMap_.try_emplace(instanceCode, std::set<InstanceType>());
						instanceMap_[instanceCode].insert(end_color_instance.getInstance());

						if (!indices_.insert(end_code).second) {
							continue;
						}
						occupied_hits.push_back(
						    std::make_tuple(end_code, prob_hit_log_, end_color_instance.getColor()));
					}
				} else {
					Point3 direction = Base::toCoord(Base::toKey(end, depth)) - sensor_origin;
					dist_sqrt = direction.squaredNorm();
					if (0 <= max_range && dist_sqrt > squared_max_range) {
						direction /= std::sqrt(dist_sqrt);
						end = sensor_origin + (direction * max_range);
					}
				}
				Point3 current = sensor_origin;
				// Move origin and end inside map
				if (!Base::moveLineInside(current, end)) {
					// Line outside of map
					continue;
				}

				Key end_key = Base::toKey(end, depth);

				if (!indices_.insert(Base::toCode(end_key)).second) {
					continue;
				}

				Point3 end_coord = Base::toCoord(end_key);

				discretized.push_back(end_coord);

				// Min/max change detection
				Point3 current_center = Base::toCoord(Base::toKey(current, depth));
				Point3 end_center = end_coord;

				double temp = Base::getNodeHalfSize(depth);
				for (int i : {0, 1, 2}) {
					min_change[i] = std::min(
					    min_change[i], std::min(end_center[i] - temp, current_center[i] - temp));
					max_change[i] = std::max(
					    max_change[i], std::max(end_center[i] + temp, current_center[i] + temp));
				}
			}

			LogitType prob_miss_log = prob_miss_log_ / double((2.0 * depth) + 1);
			indices_.clear();

			if (integrate_.valid()) {
				integrate_.wait();
			}

			if (async) {
				integrate_ = std::async(
				    std::launch::async, &OccupancyMapSemanticColor::insertPointCloudHelper, this,
				    sensor_origin, std::move(discretized), std::move(occupied_hits),
				    prob_miss_log, depth, simple_ray_casting, early_stopping, min_change,
				    max_change);
			} else {
				insertPointCloudHelper(sensor_origin, std::move(discretized),
				                       std::move(occupied_hits), prob_miss_log, depth,
				                       simple_ray_casting, early_stopping, min_change,
				                       max_change);
			}
		}

		// Clear for next point cloud
		instanceMap_.clear();
	}

	void updateValue(Code const& code, LogitType const& update, Color color)
	{
		auto path = Base::createNode(code);
		DepthType depth = code.getDepth();

		if (Base::isLeaf(path[depth], depth)) {
			updateNodeColor(*path[depth], color, toProb(update));

			if (updateOccupancy(path[depth]->value.occupancy, update)) {
				if (change_detection_enabled_) {
					changes_.insert(code);
				}
			}
		} else {
			// TODO: Error
		}

		Base::updateParents(path, depth);
	}

	void insertInstanceMap()
	{
		// Create parent bookkeeping set
		for (unsigned int d = std::max(1u, instance_depth_ + 1);
		     d <= Base::Base::getTreeDepthLevels(); ++d) {
			parentInstancesToUpdate_.try_emplace(d, std::set<LEAF_NODE*>());
			parentInstancesToUpdate_[d].clear();
		}

		for (auto const& [code, instanceSet] : instanceMap_) {
			// Foreach code node, update instance probs
			auto path = Base::createNode(code);
			DepthType depth = code.getDepth();
			if (depth == instance_depth_) {
				// Parent bookkeeping
				for (unsigned int d = std::max(1u, depth + 1);
				     d <= Base::Base::getTreeDepthLevels(); ++d) {
					INNER_NODE& node = static_cast<INNER_NODE&>(*path[d]);
					parentInstancesToUpdate_[d].emplace(path[d]);
				}
				updateInstances(static_cast<INNER_NODE&>(*(path[depth])), instanceSet);

			} else {
				// TODO: ERROR
			}
		}
	}

	void updateInstances(INNER_NODE& node, std::set<InstanceType> const& instances)
	{
		for (InstanceType const& instance : instances) {
			// Uniform prior
			auto emp_pair = node.value.instances.emplace(instance, toLogit(0.5f));

			node.value.instances[instance] =
			    std::clamp<LogitType>(node.value.instances[instance] + prob_instance_inc_log_,
			                          clamping_thres_min_log_, clamping_thres_max_log_);
		}

		// Build present instance set
		std::set<InstanceType> present_instances;

		for (auto i = node.value.instances.begin(); i != node.value.instances.end(); ++i) {
			present_instances.emplace(i->first);
		}

		std::set<InstanceType> instances_to_dec;
		std::set_difference(present_instances.begin(), present_instances.end(),
		                    instances.begin(), instances.end(),
		                    std::inserter(instances_to_dec, instances_to_dec.begin()));

		// Decrease the not seen
		for (InstanceType const& instance : instances_to_dec) {
			node.value.instances[instance] =
			    std::clamp<LogitType>(node.value.instances[instance] + prob_instance_dec_log_,
			                          clamping_thres_min_log_, clamping_thres_max_log_);
		}
	}

	void updateInstanceParents()
	{
		// Go through the levels of the tree, starting and instance_depth + 1 and
		// iteratively update parents.
		DepthType startDepth = instance_depth_ + 1;
		// Handle case when instance_depth_ are leafs.
		if (startDepth == 1) {
			for (LEAF_NODE* node_ptr : parentInstancesToUpdate_[startDepth]) {
				INNER_NODE& node = static_cast<INNER_NODE&>(*node_ptr);
				node.value.instances.clear();
				// Take max of children instances
				for (int i = 0; i < 8; ++i) {
					LEAF_NODE const& child = Base::Base::getLeafChild(node, i);
					for (auto const [instance, prob] : child.value.instances) {
						auto insert = node.value.instances.emplace(instance, prob);
						if (!insert.second) {
							// Instance present in parent, take max
							node.value.instances[instance] =
							    std::max<LogitType>(node.value.instances[instance], prob);
						}
					}
				}
			}
			startDepth += 1;
		}
		for (unsigned int d = std::max(1u, startDepth); d <= Base::Base::getTreeDepthLevels();
		     ++d) {
			int count = 0;
			for (LEAF_NODE* node_ptr : parentInstancesToUpdate_[d]) {
				INNER_NODE& node = static_cast<INNER_NODE&>(*node_ptr);
				node.value.instances.clear();
				// Take max of children instances
				for (int i = 0; i < 8; ++i) {
					count++;
					INNER_NODE const& child = Base::Base::getInnerChild(node, i);
					for (auto const [instance, prob] : child.value.instances) {
						auto insert = node.value.instances.emplace(instance, prob);
						if (!insert.second) {
							// Instance present in parent, take max
							node.value.instances[instance] =
							    std::max<LogitType>(node.value.instances[instance], prob);
						}
					}
				}
			}
		}
	}

	template <typename T>
	void InsertPointCloudDiscrete(Point3 const& sensor_origin, PointCloudColor cloud,
	                              math::Pose6 const& frame_origin, double max_range = -1,
	                              DepthType depth = 0, bool simple_ray_casting = false,
	                              unsigned int early_stopping = 0, bool async = false)
	{
		cloud.transform(frame_origin, async);
		insertPointCloudDiscrete(sensor_origin, cloud, max_range, depth, simple_ray_casting,
		                         early_stopping, async);
	}

	//
	// Set color
	//

	void setColor(Code const& code, Color color);

	//
	// Get color
	//

	Color getColor(Code const& code) const;

 protected:
	// Multiple instances per node
	CodeMap<std::set<InstanceType>> instanceMap_;
	std::map<unsigned int, std::set<LEAF_NODE*>> parentInstancesToUpdate_;

	//
	// Integrate colors
	//

	void integrateColors(Point3 const& sensor_origin, PointCloudColor const& cloud,
	                     double max_range = -1);

	//
	// Integrator helper
	//

	void insertPointCloudHelper(Point3 sensor_origin, PointCloud&& discretized,
	                            std::vector<std::tuple<Code, float, Color>>&& occupied_hits,
	                            LogitType prob_miss_log, DepthType depth,
	                            bool simple_ray_casting, unsigned int early_stopping,
	                            Point3 min_change, Point3 max_change)
	{
		std::future<void> f = std::async(std::launch::async, [this, &occupied_hits]() {
			std::for_each(begin(occupied_hits), end(occupied_hits), [this](auto&& hit) {
				updateValue(std::get<0>(hit), std::get<1>(hit), std::get<2>(hit));
			});
		});

		CodeMap<LogitType> free_hits;

		freeSpace(sensor_origin, discretized, free_hits, prob_miss_log, depth,
		          simple_ray_casting, early_stopping);

		f.wait();
		// auto start = std::chrono::system_clock::now();
		insertInstanceMap();
		// std::chrono::duration<double> insert_instance_elapsed =
		//    std::chrono::system_clock::now() - start;
		// double elapsed_instance_insert =
		//    std::chrono::duration_cast<std::chrono::nanoseconds>(insert_instance_elapsed)
		//        .count() /
		//    1000000.0;

		// start = std::chrono::system_clock::now();
		updateInstanceParents();
		/*
		std::chrono::duration<double> parent_instance_elapsed =
		    std::chrono::system_clock::now() - start;
		double elapsed_instance_parent =
		    std::chrono::duration_cast<std::chrono::nanoseconds>(parent_instance_elapsed)
		        .count() /
		    1000000.0;
		instance_insertion_stat.update(elapsed_instance_insert);
		parent_propagation_stat.update(elapsed_instance_parent);
		double tot_time = elapsed_instance_parent + elapsed_instance_insert;
		instance_fusion_stat.update(tot_time);
		max_ins_insert = std::max(max_ins_insert, elapsed_instance_insert);
		min_ins_insert = std::min(min_ins_insert, elapsed_instance_insert);
		max_par_prop = std::max(max_par_prop, elapsed_instance_parent);
		min_par_prop = std::min(min_par_prop, elapsed_instance_parent);
		max_fusion = std::max(max_fusion, tot_time);
		min_fusion = std::min(min_fusion, tot_time);
		*/
		for (auto const& [code, value] : free_hits) {
			Base::updateValue(code, value);
		}

		if (min_max_change_detection_enabled_) {
			for (int i : {0, 1, 2}) {
				min_change_[i] = std::min(min_change_[i], min_change[i]);
				max_change_[i] = std::max(max_change_[i], max_change[i]);
			}
		}
	}

	//
	// Input/output (read/write)
	//
	virtual bool readNodes(std::istream& s,
	                       ufo::geometry::BoundingVolume const& bounding_volume) override
	{
		// Check if inside bounding_volume
		Point3 const center(0, 0, 0);
		double half_size = Base::getNodeHalfSize(Base::getTreeDepthLevels());
		if (!bounding_volume.empty() &&
		    !bounding_volume.intersects(ufo::geometry::AABB(center, half_size))) {
			return true;  // No node intersects
		}

		uint32_t instance_depth;
		s.read(reinterpret_cast<char*>(&instance_depth), sizeof(instance_depth));
		instance_depth_ = instance_depth;

		uint8_t children;
		s.read(reinterpret_cast<char*>(&children), sizeof(children));

		if (0 == children) {
			Base::deleteChildren(Base::getRoot(), Base::getTreeDepthLevels());
			Base::getRoot().readData(s);
			Base::getRoot().readInstances(s);
			updateNode(Base::getRoot(), Base::getTreeDepthLevels());

			return true;
		}
		auto res = readNodesRecursSemantic(s, bounding_volume, Base::getRoot(), center,
		                                   Base::getTreeDepthLevels());
		Base::getRoot().readInstances(s);
		return res;
	}

	bool readNodesRecursSemantic(std::istream& s,
	                             ufo::geometry::BoundingVolume const& bounding_volume,
	                             INNER_NODE& node, Point3 const& center,
	                             unsigned int current_depth)
	{
		DepthType const child_depth = current_depth - 1;
		double const child_half_size = Base::getNodeHalfSize(child_depth);

		// 1 bit for each child; 0: leaf child, 1: child has children
		uint8_t children;
		s.read(reinterpret_cast<char*>(&children), sizeof(children));

		std::array<Point3, 8> child_centers;
		std::bitset<8> child_intersects;
		for (size_t i = 0; i < 8; ++i) {
			child_centers[i] = Base::getChildCenter(center, child_half_size, i);
			child_intersects[i] =
			    bounding_volume.empty() || bounding_volume.intersects(ufo::geometry::AABB(
			                                   child_centers[i], child_half_size));
		}

		Base::createChildren(node, current_depth);

		for (size_t i = 0; i < 8; ++i) {
			if (child_intersects[i]) {
				INNER_NODE& child = Base::getInnerChild(node, i);

				if ((children >> i) & 1U) {
					if (1 == child_depth) {
						double const grandchild_half_size = Base::getNodeHalfSize(0);
						Base::createChildren(child, child_depth);
						for (size_t j = 0; j < 8; ++j) {
							if (bounding_volume.empty() ||
							    bounding_volume.intersects(ufo::geometry::AABB(
							        Base::getChildCenter(child_centers[i], grandchild_half_size, j),
							        grandchild_half_size))) {
								Base::getLeafChild(child, j).readData(s);
								Base::getLeafChild(child, j).readInstances(s);
							}
						}
						updateNode(child, child_depth);

					} else {
						readNodesRecursSemantic(s, bounding_volume, child, child_centers[i],
						                        child_depth);
					}
				} else {
					Base::deleteChildren(child, child_depth);
					child.readData(s);
					updateNode(child, child_depth);
				}
				child.readInstances(s);
			}
		}

		updateNode(node, current_depth);  // To set indicators and propagate instances
		return true;
	}

	virtual bool writeNodes(std::ostream& s,
	                        ufo::geometry::BoundingVolume const& bounding_volume,
	                        DepthType min_depth) const override
	{
		// Check if inside bounding_volume
		Point3 const center(0, 0, 0);
		double half_size = Base::getNodeHalfSize(Base::getTreeDepthLevels());
		if (!bounding_volume.empty() &&
		    !bounding_volume.intersects(ufo::geometry::AABB(center, half_size))) {
			return true;  // No node intersects
		}
		// Write at what level instances are stored
		uint32_t instance_depth = instance_depth_;
		s.write(reinterpret_cast<char*>(&instance_depth), sizeof(instance_depth));

		uint8_t children = 0;
		if (Base::hasChildren(Base::getRoot()) && Base::getTreeDepthLevels() > min_depth) {
			children = UINT8_MAX;
		}
		s.write(reinterpret_cast<char*>(&children), sizeof(children));

		if (0 == children) {
			Base::getRoot().writeData(s);
			Base::getRoot().writeInstances(s);
			return true;
		}
		auto res = writeNodesRecursSemantic(s, bounding_volume, Base::getRoot(), center,
		                                    Base::getTreeDepthLevels(), min_depth);
		Base::getRoot().writeInstances(s);
		return res;
	}

	/*
	 * Writes leaf data recursively
	 */
	bool writeNodesRecursSemantic(std::ostream& s,
	                              ufo::geometry::BoundingVolume const& bounding_volume,
	                              INNER_NODE const& node, Point3 const& center,
	                              DepthType current_depth, DepthType min_depth = 0) const
	{
		DepthType const child_depth = current_depth - 1;
		double const child_half_size = Base::getNodeHalfSize(child_depth);

		// 1 bit for each child; 0: leaf child, 1: child has children
		uint8_t children = 0;
		std::array<Point3, 8> child_centers;
		std::bitset<8> child_intersects;
		for (size_t i = 0; i < 8; ++i) {
			if (child_depth > min_depth && Base::hasChildren(Base::getInnerChild(node, i))) {
				children |= 1U << i;
			}

			child_centers[i] = Base::getChildCenter(center, child_half_size, i);
			child_intersects[i] =
			    bounding_volume.empty() || bounding_volume.intersects(ufo::geometry::AABB(
			                                   child_centers[i], child_half_size));
		}

		s.write(reinterpret_cast<char*>(&children), sizeof(children));

		for (size_t i = 0; i < 8; ++i) {
			if (child_intersects[i]) {
				INNER_NODE const& child = Base::getInnerChild(node, i);

				if ((children >> i) & 1U) {
					if (1 == child_depth) {
						double const grandchild_half_size = Base::getNodeHalfSize(0);
						for (size_t j = 0; j < 8; ++j) {
							if (bounding_volume.empty() ||
							    bounding_volume.intersects(ufo::geometry::AABB(
							        Base::getChildCenter(child_centers[i], grandchild_half_size, j),
							        grandchild_half_size))) {
								Base::getLeafChild(child, j).writeData(s);
								Base::getLeafChild(child, j).writeInstances(s);
							}
						}
					} else {
						writeNodesRecursSemantic(s, bounding_volume, child, child_centers[i],
						                         child_depth, min_depth);
					}
				} else {
					// Child is leaf
					child.writeData(s);
				}
				child.writeInstances(s);
			}
		}

		return true;
	}

	//
	// Update node
	//

	virtual bool updateNode(INNER_NODE& node, DepthType depth) override;

	//
	// Update node color
	//

	void updateNodeColor(Code code, Color update);

	void updateNodeColor(LEAF_NODE& node, Color update, double prob);

	//
	//	updateNodeInstances
	//
	void updateNodeInstances(INNER_NODE& node, DepthType depth);

	//
	// Average child color
	//

	Color getAverageChildColor(INNER_NODE const& node, DepthType depth) const;

	//
	// Average color
	//

	Color getAverageColor(std::vector<Color> const& colors) const;
};
}  // namespace ufo::map

#endif  // UFO_MAP_OCCUPANCY_MAP_COLOR_H