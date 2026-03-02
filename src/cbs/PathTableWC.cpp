#include "PathTableWC.h"

#include <stdexcept>

void PathTableWC::clear() {
  map_size_ = 0;
  cat_size_ = 0;
  vertex_counts_by_time_.clear();
  edge_counts_by_time_.clear();
}

void PathTableWC::saturatedIncrement(unordered_map<size_t, uint16_t>& bucket,
                                     size_t key) {
  auto it = bucket.find(key);
  if (it == bucket.end()) {
    bucket.emplace(key, static_cast<uint16_t>(1));
    return;
  }
  if (it->second < UINT16_MAX) {
    ++(it->second);
  }
}

size_t PathTableWC::getEdgeIndex(size_t from, size_t to) const {
  if (from >= map_size_ || to >= map_size_) {
    throw std::out_of_range("PathTableWC::getEdgeIndex: endpoint out of range");
  }
  return (1 + from) * map_size_ + to;
}

void PathTableWC::build(int ignored_agent, const vector<Path*>& paths,
                        size_t cat_size, size_t map_size) {
  clear();
  map_size_ = map_size;
  if (map_size_ == 0 || cat_size == 0) {
    return;
  }

  cat_size_ = static_cast<int>(cat_size);
  vertex_counts_by_time_.assign(cat_size_, unordered_map<size_t, uint16_t>());
  edge_counts_by_time_.assign(cat_size_, unordered_map<size_t, uint16_t>());

  for (size_t agent = 0; agent < paths.size(); ++agent) {
    if ((int)agent == ignored_agent || paths[agent] == nullptr ||
        paths[agent]->empty()) {
      continue;
    }

    int prev = paths[agent]->front().location;
    const size_t hard_limit = std::min(paths[agent]->size(), cat_size);
    for (size_t timestep = 0; timestep < hard_limit; ++timestep) {
      const int curr = paths[agent]->at((int)timestep).location;
      if (curr >= 0 && curr < (int)map_size_) {
        saturatedIncrement(vertex_counts_by_time_[timestep],
                           static_cast<size_t>(curr));
      }
      if (timestep > 0 && prev >= 0 && prev < (int)map_size_ && curr >= 0 &&
          curr < (int)map_size_) {
        const size_t rev_edge =
            getEdgeIndex(static_cast<size_t>(curr), static_cast<size_t>(prev));
        saturatedIncrement(edge_counts_by_time_[timestep], rev_edge);
      }
      prev = curr;
    }

    const int goal = paths[agent]->back().location;
    if (goal < 0 || goal >= (int)map_size_) {
      continue;
    }
    for (size_t timestep = paths[agent]->size(); timestep < cat_size;
         ++timestep) {
      saturatedIncrement(vertex_counts_by_time_[timestep],
                         static_cast<size_t>(goal));
    }
  }
}

int PathTableWC::getVertexConflictCount(size_t loc, int timestep) const {
  if (loc >= map_size_ || timestep < 0 || cat_size_ <= 0) {
    return 0;
  }

  const int bucket = std::min(timestep, cat_size_ - 1);
  const auto& row = vertex_counts_by_time_[bucket];
  const auto it = row.find(loc);
  return (it == row.end()) ? 0 : static_cast<int>(it->second);
}

int PathTableWC::getEdgeConflictCount(size_t curr_id, size_t next_id,
                                      int next_timestep) const {
  if (curr_id == next_id || curr_id >= map_size_ || next_id >= map_size_ ||
      next_timestep <= 0 || next_timestep >= cat_size_ || cat_size_ <= 0) {
    return 0;
  }

  const size_t rev_edge = getEdgeIndex(curr_id, next_id);
  const auto& row = edge_counts_by_time_[next_timestep];
  const auto it = row.find(rev_edge);
  return (it == row.end()) ? 0 : static_cast<int>(it->second);
}

int PathTableWC::getFutureNumOfCollisions(size_t loc, int timestep) const {
  if (loc >= map_size_ || cat_size_ <= 0) {
    return 0;
  }

  if (timestep >= cat_size_ - 1) {
    return getVertexConflictCount(loc, timestep + 1);
  }

  int collisions = 0;
  const int start = std::max(0, timestep + 1);
  for (int t = start; t < cat_size_; ++t) {
    const auto& row = vertex_counts_by_time_[t];
    const auto it = row.find(loc);
    if (it != row.end()) {
      collisions += static_cast<int>(it->second);
    }
  }
  return collisions;
}

int PathTableWC::getLastCollisionTimestep(size_t loc) const {
  if (loc >= map_size_ || cat_size_ <= 0) {
    return -1;
  }

  for (int t = cat_size_ - 1; t >= 0; --t) {
    const auto& row = vertex_counts_by_time_[t];
    if (row.find(loc) != row.end()) {
      return t;
    }
  }
  return -1;
}
