#include "SIPP.h"
#include "SpaceTimeAStar.h"
#include <cstdlib>

namespace {
bool envFlagEnabled(const char* name) {
  if (name == nullptr) {
    return false;
  }
  const char* value = std::getenv(name);
  return value != nullptr && std::atoi(value) != 0;
}

bool hardPathSatisfiesConstraints(const Path& path,
                                  const ConstraintTable& constraint_table) {
  if (path.empty()) {
    return false;
  }
  const int offset = path.begin_time;
  for (int i = 0; i < (int)path.size(); i++) {
    const int t = offset + i;
    if (constraint_table.constrained(path[i].location, t)) {
      return false;
    }
  }
  for (int i = 0; i + 1 < (int)path.size(); i++) {
    const int next_t = offset + i + 1;
    if (constraint_table.constrained(path[i].location, path[i + 1].location,
                                     next_t)) {
      return false;
    }
  }
  return true;
}

bool canWaitAtLocationUntil(const ConstraintTable& constraint_table, int location,
                            int from_exclusive_t, int to_inclusive_t) {
  for (int t = from_exclusive_t + 1; t <= to_inclusive_t; t++) {
    if (constraint_table.constrained((size_t)location, t)) {
      return false;
    }
  }
  return true;
}
}  // namespace

void MultiLabelSIPP::updatePath(const LLNode* goal, Path& path,
                                bool collect_stage_timestamps) {
  if (goal == nullptr) {
    path.path.clear();
    path.timestamps.clear();
    return;
  }

  path.path.resize(goal->g_val + 1);
  if (collect_stage_timestamps) {
    path.timestamps.assign(goal_location.size(), 0);
    if (!path.timestamps.empty()) {
      path.timestamps.back() = goal->g_val;
    }
  } else {
    path.timestamps.clear();
  }

  const LLNode* curr = goal;
  while (curr != nullptr) {
    path.path[curr->g_val].location = curr->location;
    path.path[curr->g_val].mdd_width = 0;
    path.path[curr->g_val].is_goal = false;

    // SIPP can jump timesteps when waiting through blocked intervals.
    // Fill intermediate timesteps so CAT consumers never see default -1 entries.
    if (curr->parent != nullptr && curr->g_val > curr->parent->g_val + 1) {
      for (int t = curr->g_val - 1; t > curr->parent->g_val; --t) {
        path.path[t].location = curr->parent->location;
        path.path[t].mdd_width = 0;
        path.path[t].is_goal = false;
      }
    }

    if (collect_stage_timestamps && curr->parent != nullptr &&
        curr->stage != curr->parent->stage &&
        curr->parent->stage < path.timestamps.size()) {
      path.timestamps[curr->parent->stage] = curr->g_val;
      path.path[curr->g_val].is_goal = true;
    }
    curr = curr->parent;
  }
}

inline void MultiLabelSIPP::pushNodeToOpenAndFocal(MultiLabelSIPPNode* node) {
  num_generated++;
  node->open_handle = open_list_.push(node);
  node->in_openlist = true;
  if (node->getFVal() <= w_ * min_f_val_) {
    node->focal_handle = focal_list_.push(node);
  }
  allNodes_table_[node].push_back(node);
}

inline void MultiLabelSIPP::pushNodeToFocalOnly(MultiLabelSIPPNode* node) {
  num_generated++;
  node->in_openlist = true;
  node->focal_handle = focal_list_.push(node);
  allNodes_table_[node].push_back(node);
}

inline void MultiLabelSIPP::eraseNodeFromLists(MultiLabelSIPPNode* node) {
  if (!node->in_openlist) {
    return;
  }
  if (open_list_.empty()) {
    focal_list_.erase(node->focal_handle);
  } else if (focal_list_.empty()) {
    open_list_.erase(node->open_handle);
  } else {
    open_list_.erase(node->open_handle);
    if (node->getFVal() <= w_ * min_f_val_) {
      focal_list_.erase(node->focal_handle);
    }
  }
  node->in_openlist = false;
}

void MultiLabelSIPP::updateFocalList() {
  if (open_list_.empty()) {
    return;
  }
  auto* open_head = open_list_.top();
  if (open_head->getFVal() > min_f_val_) {
    int new_min_f_val = (int)open_head->getFVal();
    for (auto* n : open_list_) {
      if (n->getFVal() > w_ * min_f_val_ &&
          n->getFVal() <= w_ * new_min_f_val) {
        n->focal_handle = focal_list_.push(n);
      }
    }
    min_f_val_ = new_min_f_val;
  }
}

void MultiLabelSIPP::releaseNodes() {
  while (!open_list_.empty()) {
    open_list_.pop();
  }
  while (!focal_list_.empty()) {
    focal_list_.pop();
  }
  for (auto& bucket : allNodes_table_) {
    for (auto* n : bucket.second) {
      delete n;
    }
  }
  allNodes_table_.clear();
  for (auto* n : stale_nodes_) {
    delete n;
  }
  stale_nodes_.clear();
}

bool MultiLabelSIPP::dominanceCheck(MultiLabelSIPPNode* new_node) {
  if (envFlagEnabled("MAPFPC_SIPP_DISABLE_DOMINANCE")) {
    return true;
  }
  auto bucket_it = allNodes_table_.find(new_node);
  if (bucket_it == allNodes_table_.end()) {
    return true;
  }

  auto& bucket = bucket_it->second;
  if (envFlagEnabled("MAPFPC_SIPP_USE_LNS2_DOMINANCE")) {
    for (auto it = bucket.begin(); it != bucket.end(); ++it) {
      auto* old_node = *it;
      if (old_node->timestep <= new_node->timestep &&
          old_node->num_of_conflicts <= new_node->num_of_conflicts) {
        return false;
      } else if (old_node->timestep >= new_node->timestep &&
                 old_node->num_of_conflicts >= new_node->num_of_conflicts) {
        if (old_node->in_openlist) {
          eraseNodeFromLists(old_node);
        }
        stale_nodes_.push_back(old_node);
        bucket.erase(it);
        num_generated--;
        if (bucket.empty()) {
          allNodes_table_.erase(bucket_it);
        }
        return true;
      } else if (old_node->timestep < new_node->high_expansion &&
                 new_node->timestep < old_node->high_expansion) {
        if (old_node->timestep <= new_node->timestep) {
          if (old_node->num_of_conflicts > new_node->num_of_conflicts) {
            old_node->high_expansion =
                std::min(old_node->high_expansion, new_node->timestep);
          }
        } else {
          if (old_node->num_of_conflicts <= new_node->num_of_conflicts) {
            new_node->high_expansion =
                std::min(new_node->high_expansion, old_node->timestep);
          }
        }
      }
    }
    if (bucket.empty()) {
      allNodes_table_.erase(bucket_it);
    }
    return true;
  }

  for (auto it = bucket.begin(); it != bucket.end();) {
    auto* old_node = *it;
    // Dominance must preserve reachable future wait/move options. A node with
    // a tighter expansion window cannot dominate one with a wider window.
    if (old_node->timestep <= new_node->timestep &&
        old_node->num_of_conflicts <= new_node->num_of_conflicts &&
        old_node->high_expansion >= new_node->high_expansion) {
      return false;
    }

    if (old_node->timestep >= new_node->timestep &&
        old_node->num_of_conflicts >= new_node->num_of_conflicts &&
        old_node->high_expansion <= new_node->high_expansion) {
      if (old_node->in_openlist) {
        eraseNodeFromLists(old_node);
      }
      stale_nodes_.push_back(old_node);
      it = bucket.erase(it);
      num_generated--;
      continue;
    }

    if (old_node->timestep < new_node->high_expansion &&
        new_node->timestep < old_node->high_expansion) {
      if (old_node->timestep <= new_node->timestep) {
        if (old_node->num_of_conflicts > new_node->num_of_conflicts) {
          old_node->high_expansion = new_node->timestep;
        }
      } else {
        if (old_node->num_of_conflicts <= new_node->num_of_conflicts) {
          new_node->high_expansion = old_node->timestep;
        }
      }
    }
    ++it;
  }
  if (bucket.empty()) {
    allNodes_table_.erase(bucket_it);
  }
  return true;
}

Path MultiLabelSIPP::findPathSegment(ConstraintTable& constraint_table,
                                     int start_time, int stage,
                                     int lowerbound) {
  num_expanded = 0;
  num_generated = 0;
  w_ = (low_level_suboptimality >= 1.0) ? low_level_suboptimality : 1.0;
  Path path;
  path.begin_time = start_time;

  if (stage < 0 || stage >= (int)goal_location.size()) {
    return path;
  }

  const int start =
      (stage == 0) ? start_location : goal_location[(size_t)stage - 1];
  const int goal = goal_location[stage];
  int holding_time = constraint_table.length_min;
  if (stage == (int)goal_location.size() - 1) {
    holding_time = constraint_table.getHoldingTime();
  }
  if (start_time > constraint_table.length_max ||
      holding_time > constraint_table.length_max) {
    return path;
  }

  ReservationTable reservation_table(constraint_table.num_col,
                                     constraint_table.map_size, goal);
  reservation_table.setSoftConflictMode(
      ReservationTable::SoftConflictMode::Binary);
  reservation_table.copy(constraint_table);
  reservation_table.copyCAT(constraint_table);

  const int horizon = min(constraint_table.length_max, MAX_TIMESTEP - 1) + 1;
  Interval interval;
  const bool has_start_interval =
      reservation_table.find_safe_interval(interval, start, start_time);
  const bool virtual_start = !has_start_interval;
  if (virtual_start) {
    interval = Interval(start_time, horizon, 0);
  }
  if (virtual_start && start == goal && start_time >= holding_time) {
    path.path.resize(1);
    path.path[0].location = start;
    path.path[0].is_goal = true;
    return path;
  }

  auto* root = new MultiLabelSIPPNode(
      start, 0, max(get_heuristic(stage, start), holding_time - start_time),
      nullptr, start_time, stage, (int)std::get<2>(interval),
      (int)std::get<1>(interval), (int)std::get<1>(interval),
      (int)std::get<2>(interval) > 0);
  root->secondary_keys.push_back(0);
  root->dist_to_next = my_heuristic[stage][start];

  min_f_val_ = (int)root->getFVal();
  vector<int> f_ub(goal_location.size(), INT_MAX);
  if ((int)constraint_table.leq_goal_time.size() == (int)goal_location.size()) {
    f_ub.back() = constraint_table.leq_goal_time[goal_location.size() - 1];
    for (int i = (int)goal_location.size() - 2; i >= 0; --i) {
      if (constraint_table.leq_goal_time[i] != INT_MAX) {
        f_ub[i] =
            min(f_ub[i + 1], constraint_table.leq_goal_time[i] + heuristic_landmark[i]);
      } else {
        f_ub[i] = f_ub[i + 1];
      }
    }
  }
  if ((int)root->stage < (int)f_ub.size() &&
      root->g_val + root->h_val > f_ub[root->stage]) {
    delete root;
    releaseNodes();
    return path;
  }
  const int segment_lb =
      max(holding_time - start_time, max(min_f_val_, lowerbound));
  min_f_val_ = max(min_f_val_, segment_lb);
  pushNodeToOpenAndFocal(root);

  while (!open_list_.empty()) {
    updateFocalList();
    if (focal_list_.empty()) {
      break;
    }
    auto* curr = focal_list_.top();
    focal_list_.pop();
    open_list_.erase(curr->open_handle);
    curr->in_openlist = false;
    num_expanded++;

    if (curr->location == goal && curr->timestep >= holding_time) {
      updatePath(curr, path, false);
      if (!path.empty()) {
        path.back().is_goal = true;
      }
      if (hardPathSatisfiesConstraints(path, constraint_table)) {
        break;
      }
      path.path.clear();
      path.timestamps.clear();
      continue;
    }
    if (curr->timestep >= constraint_table.length_max) {
      continue;
    }

    for (int next_location : instance.getNeighbors(curr->location)) {
      const auto safe_intervals = reservation_table.get_safe_intervals(
          curr->location, next_location, curr->timestep + 1,
          curr->high_expansion + 1);
      for (const auto& next_interval : safe_intervals) {
        const int next_timestep =
            max(curr->timestep + 1, (int)std::get<0>(next_interval));
        if (next_timestep > curr->timestep + 1 &&
            !canWaitAtLocationUntil(constraint_table, curr->location,
                                    curr->timestep, next_timestep - 1)) {
          continue;
        }
        const int next_g_val = next_timestep - start_time;
        if (next_timestep + get_heuristic(stage, next_location) >
            constraint_table.length_max) {
          continue;
        }
        const int next_h_val =
            max(get_heuristic(stage, next_location), holding_time - next_timestep);
        if ((int)stage < (int)f_ub.size() && next_g_val + next_h_val > f_ub[stage]) {
          continue;
        }
        const int next_conflicts =
            curr->num_of_conflicts + (int)std::get<2>(next_interval);
        auto* next = new MultiLabelSIPPNode(
            next_location, next_g_val, next_h_val, curr, next_timestep, stage,
            next_conflicts, (int)std::get<1>(next_interval),
            (int)std::get<1>(next_interval),
            (int)std::get<2>(next_interval) > 0);
        next->timestamps = curr->timestamps;
        next->secondary_keys.push_back(-next_g_val);
        next->dist_to_next = my_heuristic[stage][next_location];
        if (next->stage == goal_location.size() - 1 &&
            next_location == goal_location.back() &&
            curr->location == goal_location.back()) {
          next->wait_at_goal = true;
        }
        if (dominanceCheck(next)) {
          pushNodeToOpenAndFocal(next);
        } else {
          delete next;
        }
      }
    }

    Interval wait_interval;
    if (curr->high_expansion == curr->high_generation &&
        reservation_table.find_safe_interval(wait_interval, curr->location,
                                             curr->high_expansion) &&
        (int)std::get<0>(wait_interval) <= constraint_table.length_max) {
      const int next_timestep = (int)std::get<0>(wait_interval);
      if (next_timestep > curr->timestep + 1 &&
          !canWaitAtLocationUntil(constraint_table, curr->location,
                                  curr->timestep, next_timestep - 1)) {
        continue;
      }
      const int next_g_val = next_timestep - start_time;
      const int next_h_val =
          max(get_heuristic(stage, curr->location), holding_time - next_timestep);
      if ((int)stage < (int)f_ub.size() && next_g_val + next_h_val > f_ub[stage]) {
        continue;
      }
      const int next_conflicts =
          curr->num_of_conflicts + (int)std::get<2>(wait_interval);
      auto* next = new MultiLabelSIPPNode(
          curr->location, next_g_val, next_h_val, curr, next_timestep, stage,
          next_conflicts, (int)std::get<1>(wait_interval),
          (int)std::get<1>(wait_interval),
          (int)std::get<2>(wait_interval) > 0);
      next->timestamps = curr->timestamps;
      next->secondary_keys.push_back(-next_g_val);
      next->dist_to_next = my_heuristic[stage][curr->location];
      if (curr->location == goal) {
        next->wait_at_goal = true;
      }
      if (dominanceCheck(next)) {
        pushNodeToOpenAndFocal(next);
      } else {
        delete next;
      }
    }
  }

  releaseNodes();
  return path;
}

Path MultiLabelSIPP::findPath(const CBSNode& node,
                              const ConstraintTable& initial_constraints,
                              const vector<Path*>& paths, int agent,
                              int lower_bound) {
  num_expanded = 0;
  num_generated = 0;
  w_ = (low_level_suboptimality >= 1.0) ? low_level_suboptimality : 1.0;
  Path path;
  path.begin_time = 0;
  const bool disable_stage_gates =
      envFlagEnabled("MAPFPC_SIPP_DISABLE_STAGE_GATES");
  const bool disable_fub = envFlagEnabled("MAPFPC_SIPP_DISABLE_FUB");
  // Default behavior: disable goal-CAT lower-bound term unless explicitly
  // overridden with MAPFPC_SIPP_DISABLE_GOAL_CAT_LB=0.
  const bool disable_goal_cat_lb = []() {
    if (const char* env = std::getenv("MAPFPC_SIPP_DISABLE_GOAL_CAT_LB")) {
      return std::atoi(env) != 0;
    }
    return true;
  }();
  const bool disable_hard_path_check =
      envFlagEnabled("MAPFPC_SIPP_DISABLE_HARD_PATH_CHECK");
  const bool disable_wait_feasibility_check =
      envFlagEnabled("MAPFPC_SIPP_DISABLE_WAIT_FEASIBILITY_CHECK");
  const bool disable_future_goal_soft_conflicts = envFlagEnabled(
      "MAPFPC_SIPP_DISABLE_FUTURE_GOAL_SOFT_CONFLICTS");

  const int num_stages = (int)goal_location.size();
  if (num_stages <= 0) {
    return path;
  }

  bool debug_windows = false;
  if (const char* env = std::getenv("MAPFPC_DEBUG_AGENT_WINDOWS")) {
    const int debug_agent = atoi(env);
    debug_windows = (debug_agent == agent);
  }

  auto dump_vec = [&](const char* tag, const vector<int>& v) {
    if (!debug_windows) {
      return;
    }
    cout << "[SIPP_DEBUG] agent " << agent << " " << tag << ":";
    for (size_t i = 0; i < v.size(); i++) {
      if (v[i] == INT_MAX) {
        cout << " " << i << "=INF";
      } else {
        cout << " " << i << "=" << v[i];
      }
    }
    cout << endl;
  };

  if (debug_windows) {
    cout << "[SIPP_DEBUG] agent " << agent << " lower_bound=" << lower_bound
         << " num_stages=" << num_stages << endl;
    dump_vec("initial g_goal_time", initial_constraints.g_goal_time);
    dump_vec("initial leq_goal_time", initial_constraints.leq_goal_time);
    cout << "[SIPP_DEBUG] agent " << agent << " node stop constraints:";
    bool printed = false;
    for (const auto& con : node.constraints) {
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      if (type == constraint_type::LEQSTOP || type == constraint_type::GSTOP) {
        printed = true;
        cout << " <" << a << "," << x << "," << y << "," << t << ","
             << (type == constraint_type::LEQSTOP ? "LS" : "GS") << ">";
      }
    }
    if (!printed) {
      cout << " none";
    }
    cout << endl;
  }

  auto t = clock();
  ConstraintTable constraint_table(initial_constraints);
  constraint_table.build(node, agent, num_stages);
  runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
  dump_vec("built g_goal_time", constraint_table.g_goal_time);
  dump_vec("built leq_goal_time", constraint_table.leq_goal_time);

  // Enforce final-stage LEQSTOP as a hard horizon bound.
  if ((int)constraint_table.leq_goal_time.size() == num_stages &&
      constraint_table.leq_goal_time[(size_t)num_stages - 1] != INT_MAX) {
    constraint_table.length_max =
        min(constraint_table.length_max,
            constraint_table.leq_goal_time[(size_t)num_stages - 1]);
  }

  if (constraint_table.length_min >= MAX_TIMESTEP ||
      constraint_table.length_min > constraint_table.length_max ||
      constraint_table.constrained(start_location, 0)) {
    if (debug_windows) {
      cout << "[SIPP_DEBUG] agent " << agent
           << " early-fail after build: length_min=" << constraint_table.length_min
           << " length_max=" << constraint_table.length_max
           << " constrained_start=" << constraint_table.constrained(start_location, 0)
           << endl;
    }
    return path;
  }

  t = clock();
  constraint_table.buildCAT(agent, paths, node.makespan + 1);
  runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

  auto can_advance_stage = [&](unsigned int stage_idx, int arrival_time) -> bool {
    if (disable_stage_gates) {
      return true;
    }
    if ((int)constraint_table.g_goal_time.size() > (int)stage_idx &&
        arrival_time <= constraint_table.g_goal_time[stage_idx]) {
      return false;  // GSTOP: must be strictly later than gate
    }
    if ((int)constraint_table.leq_goal_time.size() > (int)stage_idx &&
        constraint_table.leq_goal_time[stage_idx] != INT_MAX &&
        arrival_time > constraint_table.leq_goal_time[stage_idx]) {
      return false;  // LEQSTOP: must not miss this stage deadline
    }
    return true;
  };

  vector<int> f_ub(goal_location.size(), INT_MAX);
  if ((int)constraint_table.leq_goal_time.size() == (int)goal_location.size()) {
    f_ub.back() = constraint_table.leq_goal_time[goal_location.size() - 1];
    for (int i = (int)goal_location.size() - 2; i >= 0; --i) {
      if (constraint_table.leq_goal_time[i] != INT_MAX) {
        f_ub[i] =
            min(f_ub[i + 1], constraint_table.leq_goal_time[i] + heuristic_landmark[i]);
      } else {
        f_ub[i] = f_ub[i + 1];
      }
    }
  }
  dump_vec("f_ub", f_ub);

  ReservationTable reservation_table(constraint_table.num_col,
                                     constraint_table.map_size,
                                     goal_location.back());
  reservation_table.setSoftConflictMode(
      ReservationTable::SoftConflictMode::Binary);
  reservation_table.copy(constraint_table);
  reservation_table.copyCAT(constraint_table);

  const int horizon = min(constraint_table.length_max, MAX_TIMESTEP - 1) + 1;
  Interval interval;
  const bool has_start_interval =
      reservation_table.find_safe_interval(interval, start_location, 0);
  if (!has_start_interval) {
    interval = Interval(0, horizon, 0);
  }

  auto* root = new MultiLabelSIPPNode(
      start_location, 0, get_heuristic(0, start_location), nullptr, 0, 0,
      (int)std::get<2>(interval), (int)std::get<1>(interval),
      (int)std::get<1>(interval), (int)std::get<2>(interval) > 0);
  root->secondary_keys.push_back(0);
  root->dist_to_next = my_heuristic[0][start_location];
  if ((int)constraint_table.g_goal_time.size() >= 1 &&
      root->location == goal_location[0] &&
      can_advance_stage(0, root->g_val) &&
      root->stage < goal_location.size() - 1) {
    root->stage += 1;
    root->h_val = get_heuristic(root->stage, root->location);
    root->dist_to_next = my_heuristic[root->stage][root->location];
    if (use_timestamps) {
      root->timestamps.push_back(root->g_val);
    }
  }

  min_f_val_ = (int)root->getFVal();
  const int holding_time = constraint_table.getHoldingTime();
  const int last_target_collision_time =
      constraint_table.getLastCollisionTimestep(goal_location.back());
  min_f_val_ = max(holding_time, max(min_f_val_, lower_bound));
  if (!disable_goal_cat_lb && last_target_collision_time >= 0) {
    min_f_val_ = max(min_f_val_, last_target_collision_time + 1);
  }
  if (!disable_fub && (int)root->stage < (int)f_ub.size() &&
      root->g_val + root->h_val > f_ub[root->stage]) {
    delete root;
    releaseNodes();
    return path;
  }
  pushNodeToOpenAndFocal(root);

  while (!open_list_.empty()) {
    updateFocalList();
    if (focal_list_.empty()) {
      break;
    }
    auto* curr = focal_list_.top();
    focal_list_.pop();
    open_list_.erase(curr->open_handle);
    curr->in_openlist = false;
    num_expanded++;

    if (curr->terminal_goal) {
      updatePath(curr, path, true);
      if (disable_hard_path_check ||
          hardPathSatisfiesConstraints(path, constraint_table)) {
        break;
      }
      path.path.clear();
      path.timestamps.clear();
      continue;
    }
    if (curr->location == goal_location.back() &&
        curr->stage == goal_location.size() - 1 &&
        curr->timestep >= holding_time) {
      // Optional MLA*-style behavior: accept terminal goal as soon as hard
      // holding-time condition is met, without internalizing future CAT soft
      // conflicts at the goal cell.
      if (disable_future_goal_soft_conflicts) {
        updatePath(curr, path, true);
        if (disable_hard_path_check ||
            hardPathSatisfiesConstraints(path, constraint_table)) {
          break;
        }
        path.path.clear();
        path.timestamps.clear();
        continue;
      }

      const int future_collisions =
          constraint_table.getFutureNumOfCollisions(curr->location,
                                                    curr->timestep);
      if (future_collisions == 0) {
        updatePath(curr, path, true);
        if (disable_hard_path_check ||
            hardPathSatisfiesConstraints(path, constraint_table)) {
          break;
        }
        path.path.clear();
        path.timestamps.clear();
        continue;
      }
      // MAPF-LNS2-style soft-goal handling: keep a terminal node that
      // internalizes remaining future soft conflicts at the current goal time.
      if (!curr->wait_at_goal) {
        auto* goal = new MultiLabelSIPPNode(
            curr->location, curr->g_val, 0, curr->parent, curr->timestep,
            curr->stage, curr->num_of_conflicts + future_collisions,
            curr->high_generation, curr->high_expansion, curr->collision_v);
        goal->timestamps = curr->timestamps;
        goal->secondary_keys = curr->secondary_keys;
        goal->dist_to_next = curr->dist_to_next;
        goal->wait_at_goal = curr->wait_at_goal;
        goal->terminal_goal = true;
        if (dominanceCheck(goal)) {
          pushNodeToOpenAndFocal(goal);
        } else {
          delete goal;
        }
      }
    }
    if (curr->timestep >= constraint_table.length_max) {
      continue;
    }

    for (int next_location : instance.getNeighbors(curr->location)) {
      const auto safe_intervals = reservation_table.get_safe_intervals(
          curr->location, next_location, curr->timestep + 1,
          curr->high_expansion + 1);
      for (const auto& next_interval : safe_intervals) {
        const int interval_low = (int)std::get<0>(next_interval);
        const int interval_high = (int)std::get<1>(next_interval);
        const int interval_conflicts = (int)std::get<2>(next_interval);
        const int earliest_timestep = max(curr->timestep + 1, interval_low);
        if (earliest_timestep >= interval_high) {
          continue;
        }

        auto enqueue_successor_at = [&](int candidate_timestep) {
          if (candidate_timestep < earliest_timestep ||
              candidate_timestep >= interval_high) {
            return;
          }
          if (candidate_timestep > curr->timestep + 1 &&
              !disable_wait_feasibility_check &&
              !canWaitAtLocationUntil(constraint_table, curr->location,
                                      curr->timestep, candidate_timestep - 1)) {
            return;
          }
          if (candidate_timestep + get_heuristic(curr->stage, next_location) >
              constraint_table.length_max) {
            return;
          }

          const int next_g_val = candidate_timestep;
          unsigned int next_stage = curr->stage;
          auto next_timestamps = curr->timestamps;
          if (next_stage < goal_location.size() - 1 &&
              next_location == goal_location[next_stage]) {
            if (can_advance_stage(next_stage, next_g_val)) {
              next_stage += 1;
              if (use_timestamps) {
                next_timestamps.push_back(next_g_val);
              }
            }
          }

          const int next_h_val = get_heuristic(next_stage, next_location);
          if (next_g_val + next_h_val > constraint_table.length_max) {
            return;
          }
          if (!disable_fub && (int)next_stage < (int)f_ub.size() &&
              next_g_val + next_h_val > f_ub[next_stage]) {
            return;
          }
          const int next_conflicts =
              curr->num_of_conflicts + interval_conflicts;

          auto* next = new MultiLabelSIPPNode(
              next_location, next_g_val, next_h_val, curr, candidate_timestep,
              next_stage, next_conflicts, interval_high, interval_high,
              interval_conflicts > 0);
          next->timestamps = std::move(next_timestamps);
          next->secondary_keys.push_back(-next_g_val);
          next->dist_to_next = my_heuristic[next_stage][next_location];
          if (next->stage == goal_location.size() - 1 &&
              next_location == goal_location.back() &&
              curr->location == goal_location.back()) {
            next->wait_at_goal = true;
          }
          if (dominanceCheck(next)) {
            pushNodeToOpenAndFocal(next);
          } else {
            delete next;
          }
        };

        // Standard SIPPS expansion: earliest reachable time in interval.
        enqueue_successor_at(earliest_timestep);

        // Precedence window support: also consider delayed arrival at the
        // current-stage goal right after GSTOP, when that delay is still within
        // the same safe interval.
        if (!disable_stage_gates && curr->stage < goal_location.size() - 1 &&
            next_location == goal_location[curr->stage] &&
            curr->stage < constraint_table.g_goal_time.size()) {
          const int gate_open_time =
              constraint_table.g_goal_time[curr->stage] + 1;
          if (gate_open_time > earliest_timestep &&
              gate_open_time < interval_high) {
            enqueue_successor_at(gate_open_time);
          }
        }
      }
    }

    // If the current stage is blocked only by GSTOP at this location, allow a
    // direct in-interval wait to gate_open_time.
    bool generated_gate_wait = false;
    if (!disable_stage_gates && curr->stage < goal_location.size() - 1 &&
        curr->location == goal_location[curr->stage] &&
        curr->stage < constraint_table.g_goal_time.size()) {
      const int gate_open_time = constraint_table.g_goal_time[curr->stage] + 1;
      if (gate_open_time > curr->timestep &&
          gate_open_time <= curr->high_expansion &&
          gate_open_time <= constraint_table.length_max) {
        if (!disable_wait_feasibility_check &&
            !canWaitAtLocationUntil(constraint_table, curr->location,
                                    curr->timestep, gate_open_time - 1)) {
          continue;
        }
        const int next_timestep = gate_open_time;
        const int next_g_val = next_timestep;

        unsigned int next_stage = curr->stage;
        auto next_timestamps = curr->timestamps;
        if (can_advance_stage(next_stage, next_g_val)) {
          next_stage += 1;
          if (use_timestamps) {
            next_timestamps.push_back(next_g_val);
          }
        }

        const int next_h_val = get_heuristic(next_stage, curr->location);
        if (next_g_val + next_h_val <= constraint_table.length_max) {
          if (!disable_fub && (int)next_stage < (int)f_ub.size() &&
              next_g_val + next_h_val > f_ub[next_stage]) {
            continue;
          }
          auto* next = new MultiLabelSIPPNode(
              curr->location, next_g_val, next_h_val, curr, next_timestep,
              next_stage, curr->num_of_conflicts, curr->high_generation,
              curr->high_expansion, curr->collision_v);
          next->timestamps = std::move(next_timestamps);
          next->secondary_keys.push_back(-next_g_val);
          next->dist_to_next = my_heuristic[next_stage][curr->location];
          if (curr->location == goal_location.back()) {
            next->wait_at_goal = true;
          }
          if (dominanceCheck(next)) {
            pushNodeToOpenAndFocal(next);
            generated_gate_wait = true;
          } else {
            delete next;
          }
        }
      }
    }

    Interval wait_interval;
    if (!generated_gate_wait && curr->high_expansion == curr->high_generation &&
        reservation_table.find_safe_interval(wait_interval, curr->location,
                                             curr->high_expansion) &&
        (int)std::get<0>(wait_interval) <= constraint_table.length_max) {
      const int next_timestep = (int)std::get<0>(wait_interval);
      if (next_timestep > curr->timestep + 1 &&
          !disable_wait_feasibility_check &&
          !canWaitAtLocationUntil(constraint_table, curr->location,
                                  curr->timestep, next_timestep - 1)) {
        continue;
      }
      const int next_g_val = next_timestep;

      unsigned int next_stage = curr->stage;
      auto next_timestamps = curr->timestamps;
      if (next_stage < goal_location.size() - 1 &&
          curr->location == goal_location[next_stage]) {
        if (can_advance_stage(next_stage, next_g_val)) {
          next_stage += 1;
          if (use_timestamps) {
            next_timestamps.push_back(next_g_val);
          }
        }
      }

      const int next_h_val = get_heuristic(next_stage, curr->location);
      if (next_g_val + next_h_val <= constraint_table.length_max) {
        if (!disable_fub && (int)next_stage < (int)f_ub.size() &&
            next_g_val + next_h_val > f_ub[next_stage]) {
          continue;
        }
        const int next_conflicts =
            curr->num_of_conflicts + (int)std::get<2>(wait_interval);
        auto* next = new MultiLabelSIPPNode(
            curr->location, next_g_val, next_h_val, curr, next_timestep,
            next_stage, next_conflicts, (int)std::get<1>(wait_interval),
            (int)std::get<1>(wait_interval),
            (int)std::get<2>(wait_interval) > 0);
        next->timestamps = std::move(next_timestamps);
        next->secondary_keys.push_back(-next_g_val);
        next->dist_to_next = my_heuristic[next_stage][curr->location];
        if (curr->location == goal_location.back()) {
          next->wait_at_goal = true;
        }
        if (dominanceCheck(next)) {
          pushNodeToOpenAndFocal(next);
        } else {
          delete next;
        }
      }
    }
  }

  if (debug_windows && path.path.empty()) {
    cout << "[SIPP_DEBUG] agent " << agent
         << " search failed: expanded=" << num_expanded
         << " generated=" << num_generated << endl;
  }

  releaseNodes();
  return path;
}

int MultiLabelSIPP::getTravelTime(int start, int end,
                                  const ConstraintTable& constraint_table,
                                  int upper_bound) {
  num_expanded = 0;
  num_generated = 0;
  w_ = 1.0;
  min_f_val_ = -1;  // disable focal insertion; run OPEN-only

  int length = MAX_TIMESTEP;
  auto* root = new MultiLabelSIPPNode(
      start, 0, compute_heuristic(start, end), nullptr, 0, /*stage=*/0,
      /*num_of_conflicts=*/0, /*high_generation=*/1, /*high_expansion=*/1,
      /*collision_v=*/false);
  pushNodeToOpenAndFocal(root);

  const int static_timestep = constraint_table.latest_timestep;
  while (!open_list_.empty()) {
    auto* curr = open_list_.top();
    open_list_.pop();
    curr->in_openlist = false;
    num_expanded++;

    if (curr->location == end) {
      length = curr->g_val;
      break;
    }

    list<int> next_locations = instance.getNeighbors(curr->location);
    next_locations.emplace_back(curr->location);  // wait action
    for (int next_location : next_locations) {
      int next_timestep = curr->timestep + 1;
      int next_g_val = curr->g_val + 1;
      if (static_timestep <= curr->timestep) {
        if (curr->location == next_location) {
          continue;
        }
        next_timestep--;
      }

      if (constraint_table.constrained(next_location, next_timestep) ||
          constraint_table.constrained(curr->location, next_location,
                                       next_timestep)) {
        continue;
      }

      const int next_h_val = compute_heuristic(next_location, end);
      if (next_g_val + next_h_val >= upper_bound) {
        continue;
      }

      auto* next = new MultiLabelSIPPNode(
          next_location, next_g_val, next_h_val, nullptr, next_timestep,
          /*stage=*/0, /*num_of_conflicts=*/0,
          /*high_generation=*/next_timestep + 1,
          /*high_expansion=*/next_timestep + 1,
          /*collision_v=*/false);
      if (dominanceCheck(next)) {
        pushNodeToOpenAndFocal(next);
      } else {
        delete next;
      }
    }
  }

  releaseNodes();
  // Keep travel-time probes side-effect free for caller stats.
  num_expanded = 0;
  num_generated = 0;
  return length;
}
