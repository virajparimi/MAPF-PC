#pragma once

#include "common.h"
#include "CBSNode.h"
#include "PathTableWC.h"
#include <cstdint>
#include <stdexcept>


class ConstraintTable
{
public:
	enum class CATBackend
	{
		Legacy,
		PathTableWC
	};

	int length_min = 0;
	int length_max = MAX_TIMESTEP;
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.
	size_t num_col;
	size_t map_size;
	int cat_size = 0;

  vector<int> leq_goal_time;
  vector<int> g_goal_time;

	int getHoldingTime(); // the earliest timestep that the agent can hold its goal location
	int getLastCollisionTimestep(size_t loc) const;

	// void clear(){ct.clear(); cat_small.clear(); cat_large.clear(); landmarks.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}

	bool constrained(size_t loc, int t) const;
	bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;
	int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;
	int getFutureNumOfCollisions(size_t loc, int timestep) const;
	int getCATVertexConflictCount(size_t loc, int timestep) const;
	int getCATEdgeConflictCount(size_t curr_id, size_t next_id, int next_timestep) const;
	bool hasCATVertexConflict(size_t loc, int timestep) const;
	bool hasCATEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const;
	ConstraintTable() : cat_backend_(global_cat_backend_) {}
	ConstraintTable(size_t num_col, size_t map_size, int goal_location = -1)
	    : goal_location(goal_location), num_col(num_col), map_size(map_size),
	      cat_backend_(global_cat_backend_) {}
	ConstraintTable(const ConstraintTable& other) { copy(other); }

	static bool setGlobalCATBackendByName(const string& backend_name);
	static CATBackend getGlobalCATBackend() { return global_cat_backend_; }
	static void setGlobalCATBackendApplyOnSmallMaps(bool enabled) {
		global_cat_backend_apply_on_small_maps_ = enabled;
	}
	static bool getGlobalCATBackendApplyOnSmallMaps() {
		return global_cat_backend_apply_on_small_maps_;
	}
	static const char* catBackendName(CATBackend backend);
	CATBackend getCATBackend() const { return cat_backend_; }

	struct CATQueryStats
	{
		uint64_t build_cat_calls = 0;
		uint64_t build_cat_total_ns = 0;
		uint64_t vertex_calls = 0;
		uint64_t vertex_total_ns = 0;
		uint64_t edge_calls = 0;
		uint64_t edge_total_ns = 0;
		uint64_t future_calls = 0;
		uint64_t future_total_ns = 0;
		uint64_t last_collision_calls = 0;
		uint64_t last_collision_total_ns = 0;
	};
	static void resetCATQueryStats();
	static CATQueryStats getCATQueryStats();

	void copy(const ConstraintTable& other);
	void copyCAT(const ConstraintTable& other);
	void build(const CBSNode& node, int agent, int num_of_stops); // build the constraint table for the given agent at the given node
	void buildCAT(int agent, const vector<Path*>& paths, size_t cat_size); // build the conflict avoidance table

  void addPath(const Path& path, bool wait_at_goal);

	void insert2CT(size_t loc, int t_min, int t_max); // insert a vertex constraint to the constraint table
	void insert2CT(size_t from, size_t to, int t_min, int t_max); // insert an edge constraint to the constraint table

	size_t getNumOfLandmarks() const { return landmarks.size(); }
	unordered_map<size_t, size_t> getLandmarks() const { return landmarks; }
	list<pair<int, int>> decodeBarrier(int B1, int B2, int t);
protected:
	// Constraint Table (CT)
	unordered_map<size_t, list<pair<int, int>>> ct; // location -> time range, or edge -> time range

	unordered_map<size_t, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep

	void insertLandmark(size_t loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep

	inline size_t getEdgeIndex(size_t from, size_t to) const
	{
		if (from >= map_size || to >= map_size)
			throw std::out_of_range("ConstraintTable::getEdgeIndex: endpoint out of bounds");
		assert(from < map_size && to < map_size);
		return (1 + from) * map_size + to;
	}

private:
	size_t map_size_threshold = 10000;
	vector<list<size_t>> cat_large; // conflict avoidance table for large maps
	vector<vector<uint16_t>> cat_small; // per-(time,vertex) occupancy count
	vector<unordered_map<size_t, uint16_t>> cat_small_edges; // per-time reverse-edge occupancy count
	PathTableWC cat_path_table_wc_;
	CATBackend cat_backend_ = CATBackend::Legacy;
	bool cat_using_path_table_wc_ = false;
	static CATBackend global_cat_backend_;
	static bool global_cat_backend_apply_on_small_maps_;
	enum class CATQueryMetric
	{
		BuildCAT,
		Vertex,
		Edge,
		Future,
		LastCollision
	};
	static CATQueryStats cat_query_stats_;
	static void recordCATQueryMetric(CATQueryMetric metric, uint64_t elapsed_ns);

};
