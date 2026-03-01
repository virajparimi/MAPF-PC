#include "ConstraintTable.h"

void ConstraintTable::addPath(const Path & path, bool wait_at_goal){
  int offset = path.begin_time;
  for (int i = 0; i + 1 < path.size(); i++){
    int t = i + offset;
    insert2CT(path[i].location, t, t + 1);
    insert2CT(path[i + 1].location, path[i].location, t + 1, t + 2);
  }
  if (wait_at_goal){
    int i = path.size() - 1;
    int t = i + offset;
    insert2CT(path[i].location, t, MAX_TIMESTEP);
  } else {
    int i = path.size() - 1;
    int t = i + offset;
    insert2CT(path[i].location, t, t + 1);
  }
}


void ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max)
{
	insert2CT(getEdgeIndex(from, to), t_min, t_max);
}

void ConstraintTable::insert2CT(size_t loc, int t_min, int t_max)
{
	assert(loc >= 0);
	ct[loc].emplace_back(t_min, t_max);
	if (t_max < MAX_TIMESTEP && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
	else if (t_max == MAX_TIMESTEP && t_min > latest_timestep)
	{
		latest_timestep = t_min;
	}
}

void ConstraintTable::insertLandmark(size_t loc, int t)
{
	auto it = landmarks.find(t);
	if (it == landmarks.end())
	{
		landmarks[t] = loc;
		if (t > latest_timestep)
			latest_timestep = t;
	}
	else
		assert(it->second == loc);
}

// return the location-time pairs on the barrier in an increasing order of their timesteps
list<pair<int, int>> ConstraintTable::decodeBarrier(int x, int y, int t)
{
	list<pair<int, int>> rst;
	int x1 = x / num_col, y1 = x % num_col;
	int x2 = y / num_col, y2 = y % num_col;
	if (x1 == x2)
	{
		if (y1 < y2)
			for (int i = min(y2 - y1, t); i >= 0; i--)
			{
				rst.emplace_back(x1 * num_col + y2 - i, t - i);
			}
		else
			for (int i = min(y1 - y2, t); i >= 0; i--)
			{
				rst.emplace_back(x1 * num_col + y2 + i, t - i);
			}
	}
	else // y1== y2
	{
		if (x1 < x2)
			for (int i = min(x2 - x1, t); i >= 0; i--)
			{
				rst.emplace_back((x2 - i) * num_col + y1, t - i);
			}
		else
			for (int i = min(x1 - x2, t); i >= 0; i--)
			{
				rst.emplace_back((x2 + i) * num_col + y1, t - i);
			}
	}
	return rst;
}

bool ConstraintTable::constrained(size_t loc, int t) const
{
	assert(loc >= 0);
	if (loc < map_size)
	{
		const auto& it = landmarks.find(t);
		if (it != landmarks.end() && it->second != loc)
			return true;  // violate the positive vertex constraint
	}

	const auto& it = ct.find(loc);
	if (it == ct.end())
	{
		return false;
	}
	for (const auto& constraint: it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}
	return false;
}

bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc, int next_t) const
{
	return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
	length_min = other.length_min;
	length_max = other.length_max;
  leq_goal_time = other.leq_goal_time;
  g_goal_time = other.g_goal_time;
	goal_location = other.goal_location;
	latest_timestep = other.latest_timestep;
	num_col = other.num_col;
	map_size = other.map_size;
	ct = other.ct;
	landmarks = other.landmarks;
	// we do not copy cat
}

void ConstraintTable::copyCAT(const ConstraintTable& other)
{
	cat_size = other.cat_size;
	cat_small = other.cat_small;
	cat_small_edges = other.cat_small_edges;
	cat_large = other.cat_large;
}


// build the constraint table for the given agent at the given node
void ConstraintTable::build(const CBSNode& node, int agent, int num_of_stops)
{
  leq_goal_time.resize(num_of_stops, INT_MAX);
  g_goal_time.resize(num_of_stops, -1);

	auto curr = &node;
	while (curr->parent != nullptr)
	{
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = curr->constraints.front();
		switch (type)
		{
		case constraint_type::LEQLENGTH:
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to reach its goal at or before timestep t.
				length_max = min(length_max, t);
			else // other agents cannot stay at x at or after timestep t
				insert2CT(x, t, MAX_TIMESTEP);
			break;
		case constraint_type::GLENGTH:
			assert(curr->constraints.size() == 1);
			if (a == agent) // path of agent_id should be of length at least t + 1
				length_min = max(length_min, t + 1);
			break;
		case constraint_type::POSITIVE_VERTEX:
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t
			{
				insertLandmark(x, t);
			}
			else // other agents cannot stay at x at timestep t
			{
				insert2CT(x, t, t + 1);
			}
			break;
		case constraint_type::POSITIVE_EDGE:
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
			{
				insertLandmark(x, t - 1);
				insertLandmark(y, t);
			}
			else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
			{
				insert2CT(x, t - 1, t);
				insert2CT(y, t, t + 1);
				insert2CT(y, x, t, t + 1);
			}
			break;
		case constraint_type::VERTEX:
			if (a == agent)
			{
				for (const auto& constraint : curr->constraints) // we might have multiple vertex constraints generated by mutex propagation
				{
					tie(a, x, y, t, type) = constraint;
					insert2CT(x, t, t + 1);
				}
			}
			break;
		case constraint_type::EDGE:
			assert(curr->constraints.size() == 1);
			if (a == agent)
				insert2CT(x, y, t, t + 1);
			break;
    case constraint_type::LEQSTOP:
      for (const auto& constraint : curr->constraints) // we might have multiple vertex constraints generated by mutex propagation
        {
          tie(a, x, y, t, type) = constraint;
          if (a == agent){
            if (type == constraint_type::GSTOP){
              g_goal_time[x] = max(g_goal_time[x], t);
              length_min = max(length_min, t + 1);
            }
            if (type == constraint_type::LEQSTOP){
              leq_goal_time[x] = min(leq_goal_time[x], t);
            }
          }
        }
      break;
    case constraint_type::GSTOP:
      for (const auto& constraint : curr->constraints) // we might have multiple vertex constraints generated by mutex propagation
				{
					tie(a, x, y, t, type) = constraint;
          if (a == agent){
            if (type == constraint_type::GSTOP){
              g_goal_time[x] = max(g_goal_time[x], t);
              length_min = max(length_min, t + 1);
            }
            if (type == constraint_type::LEQSTOP){
              leq_goal_time[x] = min(leq_goal_time[x], t);
            }
          }
        }
      break;
		case constraint_type::BARRIER:
			if (a == agent)
			{
				for (auto constraint : curr->constraints)
				{
					tie(a, x, y, t, type) = constraint;
					assert(a == agent);
					auto states = decodeBarrier(x, y, t); // state = (location, timestep)
					for (const auto& state : states)
					{
						insert2CT(state.first, state.second, state.second + 1);
					}
				}
			}
			break;
		case constraint_type::RANGE:
			assert(curr->constraints.size() == 1);
			if (a == agent)
			{
				insert2CT(x, y, t + 1); // the agent cannot stay at x from timestep y to timestep t.
			}
			break;
		}
		curr = curr->parent;
	}
	if (latest_timestep < length_min)
		latest_timestep = length_min;
	if (length_max < MAX_TIMESTEP && latest_timestep < length_max)
		latest_timestep = length_max;
}


// build the conflict avoidance table
void ConstraintTable::buildCAT(int agent, const vector<Path*>& paths, size_t _cat_size)
{
	if (length_min >= MAX_TIMESTEP || length_min > length_max) // the agent cannot reach its goal location
		return; // don't have to build CAT
	cat_size = std::max(_cat_size, (size_t) latest_timestep);
	if (map_size < map_size_threshold)
	{
		cat_small.assign(cat_size, vector<uint16_t>(map_size, 0));
		cat_small_edges.assign(cat_size, unordered_map<size_t, uint16_t>());
		for (size_t ag = 0; ag < paths.size(); ag++)
		{
			if (ag == agent || paths[ag] == nullptr || paths[ag]->size() == 0)
				continue;
			int prev = paths[ag]->front().location;
			for (size_t timestep = 0; timestep < paths[ag]->size(); timestep++)
			{
				const int loc = paths[ag]->at(timestep).location;
				if (loc < 0 || loc >= map_size)
				{
					prev = loc;
					continue;
				}
				if (cat_small[timestep][loc] < UINT16_MAX)
					cat_small[timestep][loc]++;
				if (timestep > 0 && prev >= 0 && prev < (int)map_size)
				{
					const size_t rev_edge = getEdgeIndex((size_t)loc, (size_t)prev);
					auto& edge_count = cat_small_edges[timestep][rev_edge];
					if (edge_count < UINT16_MAX)
						edge_count++;
				}
				prev = loc;
			}
			int goal = paths[ag]->back().location;
			if (goal < 0 || goal >= map_size)
				continue;
			for (size_t timestep = paths[ag]->size(); timestep < cat_size; timestep++)
			{
				if (cat_small[timestep][goal] < UINT16_MAX)
					cat_small[timestep][goal]++;
			}
		}
	}
	else
	{
		cat_small.clear();
		cat_small_edges.clear();
		cat_large.resize(cat_size);
		for (size_t ag = 0; ag < paths.size(); ag++)
		{
			if (ag == agent || paths[ag] == nullptr || paths[ag]->size() == 0)
				continue;
			int prev = paths[ag]->front().location;
			if (prev < 0 || prev >= map_size)
				continue;
			int curr;
			for (size_t timestep = 1; timestep < paths[ag]->size(); timestep++)
			{
				curr = paths[ag]->at(timestep).location;
				if (curr < 0 || curr >= map_size)
				{
					prev = curr;
					continue;
				}
				cat_large[timestep].push_back(curr);
				if (prev >= 0 && prev < map_size)
					cat_large[timestep].push_back(getEdgeIndex(curr, prev));
				prev = curr;
			}
			int goal = paths[ag]->back().location;
			if (goal < 0 || goal >= map_size)
				continue;
			for (size_t timestep = paths[ag]->size(); timestep < cat_size; timestep++)
				cat_large[timestep].push_back(goal);
		}
	}
}

int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const
{
	if (hasCATVertexConflict(next_id, next_timestep))
		return 1;
	if (hasCATEdgeConflict(curr_id, next_id, next_timestep))
		return 1;
	return 0;
}

int ConstraintTable::getCATVertexConflictCount(size_t loc, int timestep) const
{
	if (loc >= map_size || timestep < 0)
		return 0;
	if (map_size < map_size_threshold)
	{
		if (cat_small.empty())
			return 0;
		if (timestep >= (int)cat_small.size())
			return (int)cat_small.back()[loc];
		return (int)cat_small[timestep][loc];
	}
	if (cat_large.empty())
		return 0;
	const int bucket = min(timestep, (int)cat_large.size() - 1);
	int count = 0;
	for (const auto& occupied : cat_large[bucket])
	{
		if (occupied == loc)
			count++;
	}
	return count;
}

int ConstraintTable::getCATEdgeConflictCount(size_t curr_id, size_t next_id, int next_timestep) const
{
	if (curr_id == next_id || curr_id >= map_size || next_id >= map_size || next_timestep <= 0)
		return 0;
	const size_t rev_edge = getEdgeIndex(curr_id, next_id);
	if (map_size < map_size_threshold)
	{
		if (cat_small_edges.empty() || next_timestep >= (int)cat_small_edges.size())
			return 0;
		const auto& row = cat_small_edges[next_timestep];
		const auto it = row.find(rev_edge);
		return it == row.end() ? 0 : (int)it->second;
	}
	if (cat_large.empty() || next_timestep >= (int)cat_large.size())
		return 0;
	int count = 0;
	for (const auto& occupied : cat_large[next_timestep])
	{
		if (occupied == rev_edge)
			count++;
	}
	return count;
}

int ConstraintTable::getFutureNumOfCollisions(size_t loc, int timestep) const
{
	if (loc >= map_size || cat_size <= 0)
		return 0;

	// If we are already beyond CAT horizon, report whether the implicit CAT tail
	// still marks this location as occupied.
	if (timestep >= cat_size - 1)
		return getCATVertexConflictCount(loc, timestep + 1);

	int rst = 0;
	const int start = max(0, timestep + 1);
	if (map_size < map_size_threshold)
	{
		if (cat_small.empty())
			return 0;
		const int end = (int)cat_small.size();
			for (int t = start; t < end; t++)
				rst += (int)cat_small[t][loc];
			return rst;
		}

	if (cat_large.empty())
		return 0;
	const int end = (int)cat_large.size();
	for (int t = start; t < end; t++)
	{
		for (const auto& occupied : cat_large[t])
		{
			if (occupied == loc)
				rst++;
		}
	}
	return rst;
}

int ConstraintTable::getLastCollisionTimestep(size_t loc) const
{
	if (loc >= map_size || cat_size <= 0)
		return -1;

	if (map_size < map_size_threshold)
	{
		if (cat_small.empty())
			return -1;
		for (int t = (int)cat_small.size() - 1; t >= 0; t--)
		{
			if (cat_small[t][loc])
				return t;
		}
		return -1;
	}

	if (cat_large.empty())
		return -1;
	for (int t = (int)cat_large.size() - 1; t >= 0; t--)
	{
		for (const auto& occupied : cat_large[t])
		{
			if (occupied == loc)
				return t;
		}
	}
	return -1;
}

bool ConstraintTable::hasCATVertexConflict(size_t loc, int timestep) const
{
	return getCATVertexConflictCount(loc, timestep) > 0;
}

bool ConstraintTable::hasCATEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const
{
	return getCATEdgeConflictCount(curr_id, next_id, next_timestep) > 0;
}


// return the earliest timestep that the agent can hold its goal location
int ConstraintTable::getHoldingTime()
{
	int rst = length_min;
	auto it = ct.find(goal_location);
	if (it != ct.end())
	{
		for (auto time_range : it->second)
			rst = max(rst, time_range.second);
	}
	for (auto landmark : landmarks)
	{
		if (landmark.second != goal_location)
			rst = max(rst, (int) landmark.first + 1);
	}
  if (g_goal_time.size() != 0){
    rst = max(rst, g_goal_time.back() + 1);
  }
	return rst;
}
