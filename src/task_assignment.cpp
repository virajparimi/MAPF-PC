/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <unordered_set>
#include "PBS.h"
#include "TaskAssignment.h"
#include "stp/TemporalGraph.hpp"

/* Declare some static utility functions */
static void usage();

static std::vector<int> parseIntList(const std::string& line) {
  std::string normalized = line;
  for (char& ch : normalized) {
    if ((ch >= '0' && ch <= '9') || ch == '-') {
      continue;
    }
    ch = ' ';
  }
  std::istringstream iss(normalized);
  std::vector<int> values;
  int value = 0;
  while (iss >> value) {
    values.push_back(value);
  }
  return values;
}

static bool loadMutableAgentsMaskFromFile(const std::string& file_path,
                                          int num_agents,
                                          std::vector<bool>& mutable_mask) {
  mutable_mask.assign(num_agents, true);
  if (file_path.empty()) {
    return true;
  }

  std::ifstream in(file_path);
  if (!in.is_open()) {
    std::cerr << "Failed to open mutable agent file: " << file_path << std::endl;
    return false;
  }

  std::string line;
  std::vector<int> mutable_agents;
  while (std::getline(in, line)) {
    const auto values = parseIntList(line);
    mutable_agents.insert(mutable_agents.end(), values.begin(), values.end());
  }

  if (mutable_agents.empty()) {
    std::cerr << "Mutable agent file is empty: " << file_path << std::endl;
    return false;
  }

  mutable_mask.assign(num_agents, false);
  for (int agent : mutable_agents) {
    if (agent < 0 || agent >= num_agents) {
      std::cerr << "Mutable agent id out of range in " << file_path
                << ": " << agent << std::endl;
      return false;
    }
    mutable_mask[agent] = true;
  }
  return true;
}

static bool loadMutableGlobalTasksFromFile(
    const std::string& file_path,
    std::unordered_set<int>& mutable_global_tasks) {
  mutable_global_tasks.clear();
  if (file_path.empty()) {
    return true;
  }

  std::ifstream in(file_path);
  if (!in.is_open()) {
    std::cerr << "Failed to open mutable task file: " << file_path << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    const auto values = parseIntList(line);
    for (int value : values) {
      mutable_global_tasks.insert(value);
    }
  }

  if (mutable_global_tasks.empty()) {
    std::cerr << "Mutable task file is empty: " << file_path << std::endl;
    return false;
  }
  return true;
}

static bool loadInitialJoinedPathsFromFile(const std::string& file_path,
                                           int num_agents,
                                           std::vector<Path>& joined_paths) {
  joined_paths.clear();
  if (file_path.empty()) {
    return true;
  }

  std::ifstream in(file_path);
  if (!in.is_open()) {
    std::cerr << "Failed to open initial paths file: " << file_path
              << std::endl;
    return false;
  }

  auto trim = [](const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return std::string();
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
  };

  auto next_data_line = [&](std::string& out) -> bool {
    while (std::getline(in, out)) {
      out = trim(out);
      if (out.empty() || out[0] == '#') {
        continue;
      }
      return true;
    }
    return false;
  };

  std::string line;
  bool has_pending_header = false;
  std::string pending_header;
  if (!next_data_line(line)) {
    std::cerr << "Initial paths file is empty: " << file_path << std::endl;
    return false;
  }
  if (line.rfind("AGENT_PATHS", 0) == 0) {
    // consume header and continue
  } else if (line.rfind("Agent", 0) == 0) {
    has_pending_header = true;
    pending_header = line;
  } else {
    std::cerr << "Unexpected header in initial paths file: '" << line << "'"
              << std::endl;
    return false;
  }

  joined_paths.assign(num_agents, Path());
  for (int agent = 0; agent < num_agents; agent++) {
    std::string agent_header;
    if (has_pending_header) {
      agent_header = pending_header;
      has_pending_header = false;
    } else if (!next_data_line(agent_header)) {
      std::cerr << "Missing Agent header for agent " << agent << " in "
                << file_path << std::endl;
      return false;
    }

    auto header_vals = parseIntList(agent_header);
    if (header_vals.empty() || header_vals.front() != agent) {
      std::cerr << "Agent order mismatch in initial paths file: expected Agent "
                << agent << ", got '" << agent_header << "'" << std::endl;
      return false;
    }

    std::string locations_line;
    if (!next_data_line(locations_line)) {
      std::cerr << "Missing locations line for agent " << agent << std::endl;
      return false;
    }
    if (locations_line.rfind("Agent", 0) == 0) {
      std::cerr << "Missing locations for agent " << agent << std::endl;
      return false;
    }
    auto locations = parseIntList(locations_line);
    if (locations.empty()) {
      std::cerr << "No locations parsed for agent " << agent << std::endl;
      return false;
    }

    std::string timestamps_line;
    if (!next_data_line(timestamps_line)) {
      std::cerr << "Missing timestamps line for agent " << agent << std::endl;
      return false;
    }
    if (timestamps_line.rfind("Agent", 0) == 0) {
      has_pending_header = true;
      pending_header = timestamps_line;
      timestamps_line.clear();
    }
    auto timestamps = parseIntList(timestamps_line);

    Path path;
    path.begin_time = 0;
    path.path.resize(locations.size());
    for (int i = 0; i < (int)locations.size(); i++) {
      path.path[i].location = locations[i];
      path.path[i].is_goal = false;
    }
    path.timestamps = timestamps;
    for (int ts : path.timestamps) {
      if (ts < 0 || ts >= (int)path.path.size()) {
        std::cerr << "Timestamp out of range for agent " << agent << ": " << ts
                  << std::endl;
        return false;
      }
      path.path[ts].is_goal = true;
    }
    joined_paths[agent] = std::move(path);
  }
  return true;
}

static bool splitJoinedPathsForPBS(const std::vector<Path>& joined_paths,
                                   const std::vector<int>& goals_per_agent,
                                   const std::vector<bool>& mutable_mask,
                                   bool strict_task_mutability,
                                   const std::vector<bool>* mutable_task_mask_flat,
                                   std::vector<Path>& task_paths,
                                   std::string& error_message) {
  error_message.clear();
  task_paths.clear();
  if ((int)joined_paths.size() != (int)goals_per_agent.size() ||
      (int)joined_paths.size() != (int)mutable_mask.size()) {
    error_message = "joined path count does not match goals_per_agent size";
    return false;
  }

  int flat_task_id = 0;
  for (int agent = 0; agent < (int)joined_paths.size(); agent++) {
    const int goal_count = goals_per_agent[agent];
    if (goal_count <= 0) {
      error_message = "goal count must be positive for all agents";
      return false;
    }

    // Mutable agents are replanned in PBS root under current CT.
    // Seed with empty per-task segments to force root replanning.
    if (!strict_task_mutability && mutable_mask[agent]) {
      for (int stage = 0; stage < goal_count; stage++) {
        task_paths.emplace_back(Path());
      }
      flat_task_id += goal_count;
      continue;
    }

    const auto& joined = joined_paths[agent];
    if (joined.empty()) {
      error_message = "joined path is empty for agent " + std::to_string(agent);
      return false;
    }
    const int old_goal_count = (int)joined.timestamps.size();
    if (!strict_task_mutability && old_goal_count != goal_count) {
      error_message = "timestamp count mismatch for agent " +
                      std::to_string(agent) + " (got " +
                      std::to_string(old_goal_count) + ", expected " +
                      std::to_string(goal_count) + ")";
      return false;
    }
    if (strict_task_mutability && old_goal_count < goal_count) {
      // Missing segments are only acceptable for mutable tasks introduced in
      // this mini instance. Non-mutable slots must keep a seed segment.
      for (int stage = old_goal_count; stage < goal_count; stage++) {
        const int task_id = flat_task_id + stage;
        const bool stage_mutable =
            (mutable_task_mask_flat != nullptr &&
             task_id >= 0 &&
             task_id < (int)mutable_task_mask_flat->size() &&
             (*mutable_task_mask_flat)[task_id]);
        if (!stage_mutable) {
          error_message = "strict mutability split mismatch for agent " +
                          std::to_string(agent) + ": missing non-mutable stage " +
                          std::to_string(stage);
          return false;
        }
      }
    }

    int previous = -1;
    for (int stage = 0; stage < goal_count; stage++) {
      if (stage >= old_goal_count) {
        task_paths.emplace_back(Path());
        continue;
      }
      const int end_t = joined.timestamps[stage];
      if (end_t < 0 || end_t >= (int)joined.size()) {
        error_message = "timestamp out of range for agent " +
                        std::to_string(agent);
        return false;
      }
      if (end_t < previous) {
        error_message =
            "timestamps must be nondecreasing for agent " + std::to_string(agent);
        return false;
      }
      const int begin_t = (stage == 0 ? 0 : joined.timestamps[stage - 1]);
      if (begin_t > end_t) {
        error_message =
            "invalid segment bounds for agent " + std::to_string(agent);
        return false;
      }
      Path seg;
      seg.begin_time = begin_t;
      for (int t = begin_t; t <= end_t; t++) {
        seg.path.push_back(joined.path[t]);
      }
      task_paths.push_back(std::move(seg));
      previous = end_t;
    }
    flat_task_id += goal_count;
  }
  return true;
}

static bool normalizeLowLevelPlanner(std::string planner_name,
                                     std::string& normalized_planner) {
  for (char& ch : planner_name) {
    ch = (char)std::tolower((unsigned char)ch);
  }
  if (planner_name == "mlastar") {
    normalized_planner = "mlastar";
    return true;
  }
  if (planner_name == "sipp" || planner_name == "sipps") {
    normalized_planner = "sipps";
    return true;
  }
  return false;
}

static bool normalizeCatBackend(std::string backend_name,
                                std::string& normalized_backend) {
  for (char& ch : backend_name) {
    ch = (char)std::tolower((unsigned char)ch);
  }
  if (backend_name == "legacy") {
    normalized_backend = "legacy";
    return true;
  }
  if (backend_name == "pathtablewc" || backend_name == "path_table_wc") {
    normalized_backend = "pathtablewc";
    return true;
  }
  return false;
}

static bool parseBoolToken(std::string token, bool& value) {
  for (char& ch : token) {
    ch = (char)std::tolower((unsigned char)ch);
  }
  if (token == "1" || token == "true" || token == "yes" || token == "on") {
    value = true;
    return true;
  }
  if (token == "0" || token == "false" || token == "no" || token == "off") {
    value = false;
    return true;
  }
  return false;
}

/* Main function */
int main(int argc, char** argv) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")

      // params for the input instance and experiment settings
      ("map,m", po::value<string>()->required(), "input file for map")(
          "agents,a", po::value<string>()->required(), "input file for agents")(
          "output,o", po::value<string>(), "output file for schedule")(
          "cutoffTime,t", po::value<double>()->default_value(7200),
          "cutoff time (seconds)")(
          "agentNum,k", po::value<int>()->default_value(0), "number of agents")(
          "fixedAssignmentFile", po::value<string>()->default_value(""),
          "optional fixed assignment file produced by MAPF-PC-LNS")(
          "mutableAgentsFile", po::value<string>()->default_value(""),
          "optional mutable-agent list file (agents not listed are fixed)")(
          "mutableTasksFile", po::value<string>()->default_value(""),
          "optional mutable global task-id list; when provided with PBS, "
          "only those tasks are replannable")(
          "initialPathsFile", po::value<string>()->default_value(""),
          "optional initial joined paths file used to seed root paths")(
          "seed,d", po::value<int>()->default_value(0), "random seed")(
      "screen,s", po::value<int>()->default_value(1),
          "screen option (0: none; 1: results; 2:all)")(
          "solver", po::value<string>()->default_value("CBS"),
          "solver, CBS, PBS or PBSN")(
          "lowLevelPlanner", po::value<string>()->default_value("mlastar"),
          "low-level planner: mlastar or sipps")(
      "sippsSuboptimality", po::value<double>()->default_value(1.0),
          "SIPPS low-level suboptimality bound (>=1.0)")(
          "catBackend", po::value<string>()->default_value(""),
          "CAT backend: legacy or pathtablewc (default: env "
          "MAPFPC_CAT_BACKEND or pathtablewc)")(
          "catBackendSmallMaps", po::value<int>()->default_value(-1),
          "Apply PathTableWC backend on small maps too when enabled: "
          "1=on, 0=off, -1=env/default (env MAPFPC_CAT_BACKEND_SMALL_MAPS, "
          "default on)")
      // params for instance generators
      ("rows", po::value<int>()->default_value(0), "number of rows")(
          "pc", po::value<bool>()->default_value(false),
          "prioritize conflicts for CBS")(
          "disjoint", po::value<bool>()->default_value(false),
          "using disjoint splitting")(
          "cols", po::value<int>()->default_value(0), "number of columns")(
          "obs", po::value<int>()->default_value(0), "number of obstacles")(
          "mutex", po::value<bool>()->default_value(false), "using mutex")(
          "rectangle", po::value<bool>()->default_value(true),
          "using rectangle reasoning")(
          "corridor", po::value<bool>()->default_value(true),
          "using corridor reasoning")(
          "bypass", po::value<bool>()->default_value(true), "using bypass")(
          "stp", po::value<bool>()->default_value(false), "using stp")(
          "target", po::value<bool>()->default_value(false),
          "using target reasoning")("timestamps",
                                    po::value<bool>()->default_value(false),
                                    "using timestamps for tie-breaking")(
          "warehouseWidth", po::value<int>()->default_value(0),
          "width of working stations on both sides, for generating instances")
      // params for CBS
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    usage();
    cout << desc << endl;
    return 1;
  }

  po::notify(vm);
  std::string lowLevelPlanner;
  if (!normalizeLowLevelPlanner(vm["lowLevelPlanner"].as<string>(),
                                lowLevelPlanner)) {
    std::cerr << "Unknown lowLevelPlanner: '"
              << vm["lowLevelPlanner"].as<string>()
              << "'. Expected 'mlastar' or 'sipps'.\n";
    return -1;
  }
  const bool useSippLowLevel = (lowLevelPlanner == "sipps");
  const double sippsSuboptimality =
      std::max(1.0, vm["sippsSuboptimality"].as<double>());
  std::string catBackendRaw = vm["catBackend"].as<string>();
  if (catBackendRaw.empty()) {
    const char* envBackend = std::getenv("MAPFPC_CAT_BACKEND");
    if (envBackend != nullptr) {
      catBackendRaw = envBackend;
    } else {
      catBackendRaw = "pathtablewc";
    }
  }
  std::string catBackend;
  if (!normalizeCatBackend(catBackendRaw, catBackend) ||
      !ConstraintTable::setGlobalCATBackendByName(catBackend)) {
    std::cerr << "Unknown catBackend: '" << catBackendRaw
              << "'. Expected 'legacy' or 'pathtablewc'.\n";
    return -1;
  }
  bool catBackendSmallMaps = true;
  const int catBackendSmallMapsOpt = vm["catBackendSmallMaps"].as<int>();
  if (catBackendSmallMapsOpt == 0 || catBackendSmallMapsOpt == 1) {
    catBackendSmallMaps = (catBackendSmallMapsOpt == 1);
  } else if (catBackendSmallMapsOpt == -1) {
    const char* envSmallMaps = std::getenv("MAPFPC_CAT_BACKEND_SMALL_MAPS");
    if (envSmallMaps != nullptr && std::strlen(envSmallMaps) > 0) {
      if (!parseBoolToken(envSmallMaps, catBackendSmallMaps)) {
        std::cerr << "Invalid MAPFPC_CAT_BACKEND_SMALL_MAPS='"
                  << envSmallMaps
                  << "'. Expected one of {0,1,true,false,yes,no,on,off}.\n";
        return -1;
      }
    }
  } else {
    std::cerr << "catBackendSmallMaps must be 0, 1, or -1 (env/default)\n";
    return -1;
  }
  ConstraintTable::setGlobalCATBackendApplyOnSmallMaps(catBackendSmallMaps);
  std::cout << "CAT_BACKEND=" << catBackend << std::endl;
  std::cout << "CAT_BACKEND_SMALL_MAPS=" << (catBackendSmallMaps ? 1 : 0)
            << std::endl;
  ConstraintTable::resetCATQueryStats();
  auto emitCatQueryStats = [&]() {
    const auto stats = ConstraintTable::getCATQueryStats();
    auto to_ms = [](uint64_t ns) -> double {
      return static_cast<double>(ns) / 1e6;
    };
    std::cout << "CAT_QUERY_STATS"
              << ",backend="
              << ConstraintTable::catBackendName(
                     ConstraintTable::getGlobalCATBackend())
              << ",build_calls=" << stats.build_cat_calls
              << ",build_ms=" << to_ms(stats.build_cat_total_ns)
              << ",vertex_calls=" << stats.vertex_calls
              << ",vertex_ms=" << to_ms(stats.vertex_total_ns)
              << ",edge_calls=" << stats.edge_calls
              << ",edge_ms=" << to_ms(stats.edge_total_ns)
              << ",future_calls=" << stats.future_calls
              << ",future_ms=" << to_ms(stats.future_total_ns)
              << ",last_collision_calls=" << stats.last_collision_calls
              << ",last_collision_ms=" << to_ms(stats.last_collision_total_ns)
              << std::endl;
  };
  int seed = vm["seed"].as<int>();
  if (seed == 0) {
    seed = (int)time(0);
  }
  srand(seed);

  ///////////////////////////////////////////////////////////////////////////
  // load the instance
  TaskAssignment instance(vm["map"].as<string>(), vm["agents"].as<string>(),
                          vm["agentNum"].as<int>());

  const string fixedAssignmentFile = vm["fixedAssignmentFile"].as<string>();
  const string mutableAgentsFile = vm["mutableAgentsFile"].as<string>();
  const string mutableTasksFile = vm["mutableTasksFile"].as<string>();
  const string initialPathsFile = vm["initialPathsFile"].as<string>();
  if (!fixedAssignmentFile.empty()) {
    if (!instance.loadFixedAssignmentFromFile(fixedAssignmentFile)) {
      cerr << "Failed to load fixed assignment file: " << fixedAssignmentFile
           << endl;
      return -1;
    }
  } else {
    instance.find_greedy_plan();
  }

  const int num_agents = vm["agentNum"].as<int>();
  vector<vector<int>> task_plan = instance.getTaskPlans();
  std::vector<bool> mutable_agents_mask;
  if (!loadMutableAgentsMaskFromFile(mutableAgentsFile, num_agents,
                                     mutable_agents_mask)) {
    return -1;
  }
  std::unordered_set<int> mutable_global_tasks;
  if (!loadMutableGlobalTasksFromFile(mutableTasksFile, mutable_global_tasks)) {
    return -1;
  }
  std::vector<bool> mutable_task_mask;
  if (!mutable_global_tasks.empty()) {
    int total_tasks = 0;
    for (int agent = 0; agent < num_agents; agent++) {
      total_tasks += (int)task_plan[agent].size();
    }
    mutable_task_mask.assign(total_tasks, false);
    std::unordered_set<int> found_mutable_tasks;
    int task_id = 0;
    for (int agent = 0; agent < num_agents; agent++) {
      for (int local = 0; local < (int)task_plan[agent].size(); local++) {
        const int global_task = task_plan[agent][local];
        if (mutable_global_tasks.find(global_task) != mutable_global_tasks.end()) {
          mutable_task_mask[task_id] = true;
          found_mutable_tasks.insert(global_task);
        }
        task_id++;
      }
    }
    for (int global_task : mutable_global_tasks) {
      if (found_mutable_tasks.find(global_task) == found_mutable_tasks.end()) {
        std::cerr << "Mutable global task " << global_task
                  << " was not found in fixed assignment" << std::endl;
        return -1;
      }
    }
  }
  std::vector<std::vector<bool>> mutable_temporal_landmarks_mask;
  if (!mutable_global_tasks.empty()) {
    mutable_temporal_landmarks_mask.resize(num_agents);
    for (int agent = 0; agent < num_agents; agent++) {
      const int landmarks = std::max(1, (int)task_plan[agent].size());
      mutable_temporal_landmarks_mask[agent].assign(landmarks, false);

      // CBS mini-repair replans at agent scope (all landmarks for mutable
      // agents), so temporal checking must include the full mutable-agent
      // landmark space, not only destroyed-task landmarks.
      const bool mutable_agent =
          agent >= 0 && agent < (int)mutable_agents_mask.size() &&
          mutable_agents_mask[agent];
      if (mutable_agent) {
        for (int local = 0; local < (int)task_plan[agent].size(); local++) {
          mutable_temporal_landmarks_mask[agent][local] = true;
        }
      } else {
        // Frozen agents stay temporal-frozen except where legacy mutable-task
        // scope explicitly marks a landmark.
        for (int local = 0; local < (int)task_plan[agent].size(); local++) {
          const int global_task = task_plan[agent][local];
          if (mutable_global_tasks.find(global_task) !=
              mutable_global_tasks.end()) {
            mutable_temporal_landmarks_mask[agent][local] = true;
          }
        }
      }
    }
  }
  std::vector<Path> initial_joined_paths;
  if (!loadInitialJoinedPathsFromFile(initialPathsFile, num_agents,
                                      initial_joined_paths)) {
    return -1;
  }
  cout << "TASK ASSIGNMENTS" << endl;
  for (int i = 0; i < num_agents; i++) {
    cout << "Agent " << i << endl;
    for (int j = 0; j < task_plan[i].size(); j++) {
      cout << task_plan[i][j] << ", ";
    }
    cout << endl;
  }
  cout << "Agent " << num_agents << endl;

  if (vm["solver"].as<string>() == "CBS") {
    cout << "Invoking CBS" << endl;
    auto h = heuristics_type::ZERO;
    CBS cbs(instance, useSippLowLevel, h, vm["screen"].as<int>());

    cbs.setPrioritizeConflicts(vm["pc"].as<bool>());
    cbs.setRectangleReasoning(vm["rectangle"].as<bool>());
    cbs.setCorridorReasoning(vm["corridor"].as<bool>());
    cbs.setSTP(vm["stp"].as<bool>());
    cbs.setUsingTimestamps(vm["timestamps"].as<bool>());
    cbs.setTargetReasoning(vm["target"].as<bool>());
    cbs.setDisjointSplitting(vm["disjoint"].as<bool>());
    cbs.setBypass(vm["bypass"].as<bool>());
    cbs.setMutexReasoning(vm["mutex"].as<bool>() ? mutex_strategy::MUTEX_C
                                                 : mutex_strategy::N_MUTEX);
    cbs.setLowLevelSuboptimality(sippsSuboptimality);
    //////////////////////////////////////////////////////////////////////
    // run
    double runtime = 0;
    int min_f_val = 0;
    cbs.clear();
    cbs.setMutableAgents(mutable_agents_mask);
    if (!mutable_temporal_landmarks_mask.empty()) {
      cbs.setMutableTemporalLandmarksMask(mutable_temporal_landmarks_mask);
    }
    if (!initial_joined_paths.empty()) {
      cbs.setInitialPaths(initial_joined_paths);
    }
    const bool solved = cbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
    runtime += cbs.runtime;
    min_f_val = (int)cbs.min_f_val;
    if (!solved) {
      std::cerr << "CBS failed to find a valid solution" << std::endl;
      emitCatQueryStats();
      return -1;
    }
    cbs.randomRoot = true;
    cbs.runtime = runtime;
    if (vm.count("output"))
      cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());

    vector<Path*> paths = cbs.getPaths();
    cout << "TASK PATHS" << endl;
    for (int i = 0; i < num_agents; i++) {
      bool previousLocationWasGoal = true;
      cout << "Agent " << i << endl;
      for (int j = 0; j < paths[i]->size(); j++) {
        cout << paths[i]->at(j).location;
        if (previousLocationWasGoal) {
          cout << " @ " << j;
          previousLocationWasGoal = false;
        }
        if (std::find(paths[i]->timestamps.begin(), paths[i]->timestamps.end(),
                      j) != paths[i]->timestamps.end()) {
          previousLocationWasGoal = true;
          //   cout << " *";
        }
        cout << " -> ";
      }
      cout << endl;
    }
    cout << "Agent " << num_agents << endl;
    emitCatQueryStats();
    cbs.clearSearchEngines();

  } else if (vm["solver"].as<string>() == "PBS") {
    PBS pbs(instance, useSippLowLevel, vm["screen"].as<int>());
    pbs.setLowLevelSuboptimality(sippsSuboptimality);
    //////////////////////////////////////////////////////////////////////
    // run
    double runtime = 0;
    int min_f_val = 0;
    pbs.clear();
    pbs.setMutableAgents(mutable_agents_mask);
    if (!mutable_task_mask.empty()) {
      pbs.setMutableTasksMask(mutable_task_mask);
    }
    if (!initial_joined_paths.empty()) {
      std::vector<int> goals_per_agent(num_agents, 1);
      for (int i = 0; i < num_agents; i++) {
        goals_per_agent[i] = std::max(1, (int)task_plan[i].size());
      }
      std::vector<Path> initial_task_paths;
      std::string split_error;
      if (!splitJoinedPathsForPBS(initial_joined_paths, goals_per_agent,
                                  mutable_agents_mask,
                                  !mutable_task_mask.empty(),
                                  mutable_task_mask.empty() ? nullptr : &mutable_task_mask,
                                  initial_task_paths, split_error)) {
        std::cerr << "Failed to split initial joined paths for PBS: "
                  << split_error << std::endl;
        emitCatQueryStats();
        return -1;
      }
      pbs.setInitialTaskPaths(initial_task_paths);
    }
    const bool solved = pbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
    runtime += pbs.runtime;
    min_f_val = (int)pbs.min_f_val;
    if (!solved) {
      std::cerr << "PBS failed to find a valid solution" << std::endl;
      emitCatQueryStats();
      return -1;
    }
    pbs.randomRoot = true;
    pbs.runtime = runtime;
    if (vm.count("output"))
      pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
    vector<Path*> paths = pbs.getPaths();
    cout << "TASK PATHS" << endl;
    for (int i = 0; i < num_agents; i++) {
      bool previousLocationWasGoal = true;
      cout << "Agent " << i << endl;
      for (int j = 0; j < paths[i]->size(); j++) {
        cout << paths[i]->at(j).location;
        if (previousLocationWasGoal) {
          cout << " @ " << j;
          previousLocationWasGoal = false;
        }
        if (std::find(paths[i]->timestamps.begin(), paths[i]->timestamps.end(),
                      j) != paths[i]->timestamps.end()) {
          previousLocationWasGoal = true;
          //   cout << " *";
        }
        cout << " -> ";
      }
      cout << endl;
    }
    cout << "Agent " << num_agents << endl;
    emitCatQueryStats();
    pbs.clearSearchEngines();
  } else if (vm["solver"].as<string>() == "PBSN") {
    PBS_naive pbs(instance, useSippLowLevel, vm["screen"].as<int>());
    pbs.setLowLevelSuboptimality(sippsSuboptimality);
    //////////////////////////////////////////////////////////////////////
    // run
    double runtime = 0;
    int min_f_val = 0;
    pbs.clear();
    const bool solved = pbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
    runtime += pbs.runtime;
    min_f_val = (int)pbs.min_f_val;
    if (!solved) {
      std::cerr << "PBSN failed to find a valid solution" << std::endl;
      emitCatQueryStats();
      return -1;
    }
    pbs.randomRoot = true;
    pbs.runtime = runtime;
    if (vm.count("output"))
      pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
    emitCatQueryStats();
    pbs.clearSearchEngines();
  } else {
    cout << "Unknown solver: " << vm["solver"].as<string>() << endl;
    emitCatQueryStats();
    return -1;
  }

  return 0;
}

/*
Prints out usage help.
*/
static void usage() {
  // TODO: update the following information
  fprintf(stderr, "Usage: optimize instance exp strat [options]\n");
  fprintf(stderr, "Arguments:\n");
  fprintf(stderr, "	help		-> this list\n");
  fprintf(stderr,
          "	screen		-> screen output on(=1) or off(=0) (default: 0)\n");
  fprintf(stderr, "	instance	-> MIP in MPS format\n");
  fprintf(stderr, "	exp		-> experiment name\n");
  fprintf(stderr, "	strat		-> branching strategy:\n");
  fprintf(stderr, "				-1: CPLEX Default\n");
  fprintf(stderr, "				-2: FSB\n");
  fprintf(stderr, "				-3: Most Infeasible\n");
  fprintf(stderr, "				-4: SB\n");
  fprintf(stderr, "				-5: PC\n");
  fprintf(stderr, "				 3: Hybrid SB/PC\n");
  fprintf(stderr, "				 6: ML\n");
  fprintf(stderr, "				 7: ML + Problem Features \n");
  fprintf(stderr, "	sbnodes		-> num. of SB nodes if strat:={3,6,7}\n");
  fprintf(stderr,
          "	varPerNode	-> num. of variables per SB node if strat:={3,6,7}\n");
  fprintf(stderr,
          "	varSorting	-> variable sorting criterion if strat:={-4,3,6,7}\n");
  fprintf(stderr, "	learningAlg	-> learning algorithm if strat={6,7}\n");
  fprintf(stderr, "				 1: SVM-Rank\n");
  fprintf(stderr, "				 2: NDCG\n");
  fprintf(stderr, "				 3: Regression\n");
  fprintf(stderr, "	loss		-> SVM loss function variant, (default: 2)\n");
  fprintf(stderr, "	c		-> SVM parameter (default: 0.1)\n");
  fprintf(stderr,
          "	alpha		-> Fraction of max. SB score to get label 1 (default: "
          "0.2)\n");
  fprintf(stderr, "	diag		-> diagnostic mode (default: 0)\n");
  fprintf(stderr,
          "	root		-> root-only cuts and heuristics (default: 0)\n");
  fprintf(stderr, "	maxtime		-> time cutoff in sec. (default: 7200)\n");
  fprintf(
      stderr,
      "	restart		-> Restart after learning(=1) or not (=0), (default: 0)\n");
  fprintf(stderr,
          "	kernel		-> Add interaction features with Kernel(=1) or "
          "not(=0), (default: 1)\n");
  fprintf(stderr, "	seed		-> CPLEX random seed (default: 1)\n");
  fprintf(stderr,
          "	cutoff		-> Use instance's optimal value as cutoff(=1) or "
          "not(=0), (default: 0)\n");
  fprintf(stderr,
          "	whichpc		-> Use PC scores as search goes(=1) or after Phase "
          "1(=0), (default: 0)\n");
  fprintf(stderr,
          "whichFeatures	-> which features to include, (default 0) \n");
  fprintf(stderr, "				0: All\n");
  fprintf(stderr, "				1: Static\n");
  fprintf(stderr, "				2: Active\n");
  fprintf(stderr, "				3: Compact\n");
  fprintf(stderr,
          "	desc		-> Optional string describing this experiment\n");
  fprintf(stderr, "Exiting...\n");
}
