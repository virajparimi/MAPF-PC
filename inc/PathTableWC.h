#pragma once

#include "common.h"

// Sparse, count-valued path table for CAT-style conflict queries on large maps.
// This backend keeps per-time sparse occupancy and reverse-edge counts.
class PathTableWC {
public:
  PathTableWC() = default;

  void clear();
  void build(int ignored_agent, const vector<Path*>& paths, size_t cat_size,
             size_t map_size);

  int getVertexConflictCount(size_t loc, int timestep) const;
  int getEdgeConflictCount(size_t curr_id, size_t next_id,
                           int next_timestep) const;
  int getFutureNumOfCollisions(size_t loc, int timestep) const;
  int getLastCollisionTimestep(size_t loc) const;

  int getCatSize() const { return cat_size_; }
  bool empty() const { return cat_size_ <= 0; }

private:
  size_t map_size_ = 0;
  int cat_size_ = 0;
  vector<unordered_map<size_t, uint16_t>> vertex_counts_by_time_;
  vector<unordered_map<size_t, uint16_t>> edge_counts_by_time_;

  static void saturatedIncrement(unordered_map<size_t, uint16_t>& bucket,
                                 size_t key);
  size_t getEdgeIndex(size_t from, size_t to) const;
};
