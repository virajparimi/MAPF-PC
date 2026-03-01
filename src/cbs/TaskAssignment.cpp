#include "TaskAssignment.h"
#include <boost/tokenizer.hpp>
#include <sstream>

TaskAssignment::TaskAssignment(const string& map_fname,
                               const string& agent_fname, int num_of_agents)
    : Instance() {
  this->map_fname = map_fname;
  this->agent_fname = agent_fname;
  this->num_of_agents = num_of_agents;

  bool succ = false;
  if (map_fname.find("kiva") != string::npos) {
    // We are going to work with KIVA instances
    succ = loadKivaMap();
  } else {
    succ = loadMap();
  }
  if (!succ) {
    cerr << "Map file " << map_fname << " not found." << endl;
    exit(-1);
  }

  if (map_fname.find("kiva") != string::npos) {
    // We are going to load KIVA tasks with implicit precedence constraints
    succ = loadKivaAgentsAndTasks();
  } else {
    succ = loadAgents();
  }
  if (!succ) {
    cerr << "Agent file " << agent_fname << " not found." << endl;
  }
}

bool TaskAssignment::loadKivaMap() {
  using namespace std;
  using namespace boost;

  ifstream file(map_fname.c_str());
  if (!file.is_open()) {
    return false;
  }

  string line;
  tokenizer<char_separator<char>>::iterator begin;

  getline(file, line);
  char_separator<char> sep(",");
  tokenizer<char_separator<char>> tokenizer(line, sep);
  begin = tokenizer.begin();
  num_of_rows = atoi((*begin).c_str()) + 2;  // Read the number of rows
  begin++;
  num_of_cols = atoi((*begin).c_str()) + 2;  // Read the number of columns

  getline(file, line);  // Workpoint number
  getline(file, line);  // Number of agents
  getline(file, line);  // Maximum time

  // Initialize the agent start locations
  int agentNum = 0, endPointNum = 0;
  start_locations.resize(num_of_agents);

  map_size = num_of_cols * num_of_rows;
  my_map.resize(map_size, false);
  for (int i = 1; i < num_of_rows - 1; i++) {
    getline(file, line);
    for (int j = 1; j < num_of_cols - 1; j++) {
      my_map[linearizeCoordinate(i, j)] = (line[j - 1] == '@');
      if (line[j - 1] == 'r') {
        // This is a robot spawn location
        start_locations[agentNum] = linearizeCoordinate(i, j);
        assert(!isObstacle(start_locations[agentNum]));
        agentNum++;
      }
      if (line[j - 1] == 'e') {
        // This is a task spawn location
        end_points.push_back(linearizeCoordinate(i, j));
        assert(!isObstacle(end_points[endPointNum]));
        endPointNum++;
      }
    }
  }

  for (int i = 0; i < num_of_rows; i++) {
    my_map[i * num_of_cols] = false;
    my_map[i * num_of_cols + num_of_cols - 1] = false;
  }
  for (int j = 1; j < num_of_cols - 1; j++) {
    my_map[j] = false;
    my_map[map_size - num_of_cols + j] = false;
  }

  assert(agentNum == num_of_agents);
  file.close();
  return true;
}

bool TaskAssignment::loadKivaAgentsAndTasks() {
  using namespace std;
  using namespace boost;

  ifstream file(agent_fname.c_str());
  if (!file.is_open()) {
    return false;
  }

  string line;
  int taskNum;
  stringstream stringLine;
  getline(file, line);
  stringLine << line;
  stringLine >> taskNum;

  num_of_tasks = 2 * taskNum;
  task_locations.resize(num_of_tasks);

  for (int i = 0; i < num_of_tasks; i += 2) {
    int releaseTime, startTask, goalTask, timeOfStartTask, timeOfGoalTask;
    getline(file, line);
    stringLine.clear();
    stringLine << line;
    stringLine >> releaseTime >> startTask >> goalTask >> timeOfStartTask >>
        timeOfGoalTask;

    startTask %= (int)end_points.size();
    goalTask %= (int)end_points.size();

    task_locations[i] = end_points[startTask];
    task_locations[i + 1] = end_points[goalTask];
    assert(!isObstacle(task_locations[i]));
    assert(!isObstacle(task_locations[i + 1]));

    temporal_dependecies.emplace_back(i, i + 1);
  }

  cout << "#agents: " << num_of_agents << "\t#tasks: " << num_of_tasks
       << "\t#dependecies: " << (int)temporal_dependecies.size() << endl;

  goal_locations.push_back(task_locations);
  search_engine =
      std::make_unique<MultiLabelSpaceTimeAStar>(*((Instance*)this), 0);
  goal_locations.clear();

  file.close();

  return true;
}

bool TaskAssignment::loadAgents() {

  cout << "Loading " << agent_fname << endl;

  using namespace std;
  using namespace boost;

  string line;
  ifstream myfile(agent_fname.c_str());
  if (!myfile.is_open())
    return false;

  getline(myfile, line);
  // My benchmark
  this->num_of_agents = num_of_agents;
  if (num_of_agents == 0) {
    cerr << "The number of agents should be larger than 0" << endl;
    exit(-1);
  }
  start_locations.resize(num_of_agents);
  // goal_locations.resize(1);
  // temporal_cons.resize(num_of_agents * num_of_agents);
  char_separator<char> sep(",");
  for (int i = 0; i < num_of_agents; i++) {
    getline(myfile, line);
    tokenizer<char_separator<char>> tok(line, sep);
    tokenizer<char_separator<char>>::iterator beg = tok.begin();
    // read start [row,col] for agent i
    auto col = atoi((*beg).c_str());
    beg++;
    auto row = atoi((*beg).c_str());

    start_locations[i] = linearizeCoordinate(row, col);
    assert(!this->isObstacle(start_locations[i]));
  }

  getline(myfile, line);
  while (!myfile.eof() && line[0] != 't') {
    getline(myfile, line);
  }
  getline(myfile, line);

  num_of_tasks = atoi(line.c_str());
  task_locations.resize(num_of_tasks);

  for (int i = 0; i < num_of_tasks; i++) {
    getline(myfile, line);
    tokenizer<char_separator<char>> tok(line, sep);
    tokenizer<char_separator<char>>::iterator beg = tok.begin();
    auto col = atoi((*beg).c_str());
    beg++;
    auto row = atoi((*beg).c_str());

    task_locations[i] = linearizeCoordinate(row, col);
    assert(!this->isObstacle(task_locations[i]));
  }

  while (!myfile.eof() && line[0] != 't') {
    getline(myfile, line);
  }
  getline(myfile, line);

  int num_of_dependencies = atoi(line.c_str());
  task_locations.resize(num_of_tasks);

  for (int i = 0; i < num_of_dependencies; i++) {
    getline(myfile, line);
    tokenizer<char_separator<char>> tok(line, sep);
    tokenizer<char_separator<char>>::iterator beg = tok.begin();
    auto pred = atoi((*beg).c_str());
    beg++;
    auto post = atoi((*beg).c_str());
    temporal_dependecies.push_back({pred, post});
  }

  cout << "#agents: " << num_of_agents << "\t#tasks: " << num_of_tasks
       << "\t#dependecies: " << num_of_dependencies << endl;

  goal_locations.push_back(task_locations);
  search_engine =
      std::make_unique<MultiLabelSpaceTimeAStar>(*((Instance*)this), 0);
  goal_locations.clear();

  myfile.close();

  return true;
}

bool TaskAssignment::loadFixedAssignmentFromFile(
    const string& assignment_fname) {
  using namespace std;

  ifstream in(assignment_fname.c_str());
  if (!in.is_open()) {
    cerr << "Failed to open fixed assignment file " << assignment_fname << endl;
    return false;
  }

  auto next_data_line = [&](string& out) -> bool {
    while (getline(in, out)) {
      const auto first = out.find_first_not_of(" \t\r\n");
      if (first == string::npos) {
        continue;
      }
      out = out.substr(first);
      if (!out.empty() && out[0] == '#') {
        continue;
      }
      return true;
    }
    return false;
  };

  string line;
  if (!next_data_line(line)) {
    cerr << "Fixed assignment file is empty: " << assignment_fname << endl;
    return false;
  }

  auto finalize_plan = [&](vector<vector<int>> parsed_plan) -> bool {
    vector<char> task_used(num_of_tasks, false);
    int total_assigned = 0;
    for (int agent = 0; agent < num_of_agents; agent++) {
      for (int task_idx : parsed_plan[agent]) {
        if (task_idx < 0 || task_idx >= num_of_tasks) {
          cerr << "Task index out of range in fixed assignment file "
               << assignment_fname << ": " << task_idx << endl;
          return false;
        }
        if (task_used[task_idx]) {
          cerr << "Duplicate task assignment in fixed assignment file "
               << assignment_fname << ": task " << task_idx << endl;
          return false;
        }
        task_used[task_idx] = true;
        total_assigned++;
      }
    }
    if (total_assigned != num_of_tasks) {
      cerr << "Fixed assignment file does not assign all tasks: assigned="
           << total_assigned << ", expected=" << num_of_tasks << " in "
           << assignment_fname << endl;
      return false;
    }
    task_plan = std::move(parsed_plan);
    return buildGoalsAndTemporalConstraintsFromTaskPlan();
  };

  // Support MAPF-PC-LNS assignment logs:
  // TASK ASSIGNMENTS
  // Agent 0
  // 1, 5, 8, ...
  if (line.rfind("TASK ASSIGNMENTS", 0) == 0 ||
      line.rfind("Agent", 0) == 0) {
    vector<vector<int>> parsed_plan(num_of_agents);
    bool has_pending_agent_header = false;
    string pending_agent_header;

    auto parse_agent_header = [&](const string& header, int expected_agent)
        -> bool {
      istringstream hss(header);
      string tag;
      string agent_token;
      if (!(hss >> tag >> agent_token) || tag != "Agent") {
        return false;
      }
      if (!agent_token.empty() && agent_token.back() == ':') {
        agent_token.pop_back();
      }
      int parsed_agent = -1;
      {
        istringstream agent_ss(agent_token);
        if (!(agent_ss >> parsed_agent)) {
          return false;
        }
      }
      if (parsed_agent != expected_agent) {
        cerr << "Agent order mismatch in fixed assignment file "
             << assignment_fname << ": expected Agent " << expected_agent
             << ", got Agent " << parsed_agent << endl;
        return false;
      }
      return true;
    };

    auto parse_task_id_row = [&](const string& row, vector<int>& out) {
      string normalized = row;
      for (char& ch : normalized) {
        if (ch == ',' || ch == ';' || ch == '\t') {
          ch = ' ';
        }
      }
      istringstream iss(normalized);
      int task_idx = -1;
      while (iss >> task_idx) {
        out.push_back(task_idx);
      }
    };

    if (line.rfind("Agent", 0) == 0) {
      has_pending_agent_header = true;
      pending_agent_header = line;
    }

    for (int agent = 0; agent < num_of_agents; agent++) {
      string agent_header;
      if (has_pending_agent_header) {
        agent_header = pending_agent_header;
        has_pending_agent_header = false;
      } else if (!next_data_line(agent_header)) {
        cerr << "Unexpected end of file while reading Agent " << agent
             << " in " << assignment_fname << endl;
        return false;
      }
      if (!parse_agent_header(agent_header, agent)) {
        cerr << "Malformed agent header '" << agent_header << "' in "
             << assignment_fname << endl;
        return false;
      }

      string tasks_line;
      if (!next_data_line(tasks_line)) {
        tasks_line.clear();  // allow trailing agents with no tasks
      }

      if (tasks_line.rfind("Agent", 0) == 0) {
        // Agent has no tasks; keep this header for next loop.
        has_pending_agent_header = true;
        pending_agent_header = tasks_line;
      } else if (tasks_line.rfind("TASK PATHS", 0) == 0 ||
                 tasks_line.rfind("temporal cons:", 0) == 0) {
        // End of assignment section.
        has_pending_agent_header = false;
      } else {
        parse_task_id_row(tasks_line, parsed_plan[agent]);
      }
    }

    return finalize_plan(std::move(parsed_plan));
  }

  // Legacy MAPF-PC assignment format:
  // <agent_count>
  // <goal_count> <start_x> <start_y> <goal1_x> <goal1_y> ...
  int file_agents = -1;
  {
    istringstream iss(line);
    if (!(iss >> file_agents)) {
      cerr << "Failed to parse agent count from fixed assignment file: "
           << assignment_fname << endl;
      return false;
    }
  }
  if (file_agents != num_of_agents) {
    cerr << "Fixed assignment file agent count mismatch: file=" << file_agents
         << ", expected=" << num_of_agents << endl;
    return false;
  }

  unordered_map<int, vector<int>> tasks_by_location;
  tasks_by_location.reserve((size_t)num_of_tasks * 2);
  for (int t = 0; t < num_of_tasks; t++) {
    tasks_by_location[task_locations[t]].push_back(t);
  }

  vector<vector<int>> parsed_plan(num_of_agents);
  vector<char> task_used(num_of_tasks, false);

  for (int agent = 0; agent < num_of_agents; agent++) {
    if (!next_data_line(line)) {
      cerr << "Unexpected end of file while reading assignments for agent "
           << agent << " in " << assignment_fname << endl;
      return false;
    }
    if (line.rfind("temporal cons:", 0) == 0) {
      cerr << "Encountered temporal constraints before reading all agents in "
           << assignment_fname << endl;
      return false;
    }

    istringstream iss(line);
    int goals = -1;
    int sx = 0, sy = 0;
    if (!(iss >> goals >> sx >> sy) || goals < 0) {
      cerr << "Malformed assignment row for agent " << agent << " in "
           << assignment_fname << endl;
      return false;
    }
    const auto expected_start = getCoordinate(start_locations[agent]);
    if (sx != expected_start.second || sy != expected_start.first) {
      cerr << "Start coordinate mismatch for agent " << agent
           << " in fixed assignment file " << assignment_fname << endl;
      return false;
    }

    parsed_plan[agent].reserve(goals);
    for (int g = 0; g < goals; g++) {
      int gx = 0, gy = 0;
      if (!(iss >> gx >> gy)) {
        cerr << "Insufficient goal coordinates for agent " << agent
             << " in fixed assignment file " << assignment_fname << endl;
        return false;
      }
      const int location = linearizeCoordinate(gy, gx);
      const auto it = tasks_by_location.find(location);
      if (it == tasks_by_location.end()) {
        cerr << "Goal location (" << gx << "," << gy
             << ") does not match any task location in " << assignment_fname
             << endl;
        return false;
      }

      int selected_task = -1;
      for (int candidate : it->second) {
        if (!task_used[candidate]) {
          selected_task = candidate;
          break;
        }
      }
      if (selected_task < 0) {
        cerr << "All tasks at location (" << gx << "," << gy
             << ") are already assigned in " << assignment_fname << endl;
        return false;
      }
      task_used[selected_task] = true;
      parsed_plan[agent].push_back(selected_task);
    }
  }
  return finalize_plan(std::move(parsed_plan));
}

bool TaskAssignment::buildGoalsAndTemporalConstraintsFromTaskPlan() {
  using namespace std;

  if ((int)task_plan.size() != num_of_agents) {
    cerr << "task_plan size does not match number of agents" << endl;
    return false;
  }

  vector<pair<int, int>> task_to_agent_and_index(num_of_tasks, {-1, -1});
  goal_locations.clear();
  goal_locations.resize(num_of_agents);

  for (int i = 0; i < num_of_agents; i++) {
    for (int j = 0; j < (int)task_plan[i].size(); j++) {
      const int task_idx = task_plan[i][j];
      if (task_idx < 0 || task_idx >= num_of_tasks) {
        cerr << "task index out of range in task_plan: " << task_idx << endl;
        return false;
      }
      if (task_to_agent_and_index[task_idx].first != -1) {
        cerr << "duplicate task assignment for task " << task_idx << endl;
        return false;
      }
      task_to_agent_and_index[task_idx] = {i, j};
      goal_locations[i].push_back(task_locations[task_idx]);
    }
    if (task_plan[i].empty()) {
      goal_locations[i].push_back(start_locations[i]);
    }
  }

  for (int task = 0; task < num_of_tasks; task++) {
    if (task_to_agent_and_index[task].first < 0) {
      cerr << "unassigned task in task_plan: " << task << endl;
      return false;
    }
  }

  temporal_cons.clear();
  temporal_cons.resize(num_of_agents * num_of_agents);
  for (const auto& dependence : temporal_dependecies) {
    int task_i, task_j;
    std::tie(task_i, task_j) = dependence;
    if (task_i < 0 || task_i >= num_of_tasks || task_j < 0 ||
        task_j >= num_of_tasks) {
      cerr << "temporal dependency task out of range: " << task_i << " -> "
           << task_j << endl;
      return false;
    }
    int agent_i, i, agent_j, j;
    std::tie(agent_i, i) = task_to_agent_and_index[task_i];
    std::tie(agent_j, j) = task_to_agent_and_index[task_j];
    if (agent_i < 0 || agent_j < 0) {
      cerr << "temporal dependency references unassigned task: " << task_i
           << " -> " << task_j << endl;
      return false;
    }
    temporal_cons[agent_i * num_of_agents + agent_j].push_back({i, j});
  }

  return true;
}

void TaskAssignment::find_greedy_plan() {

  //
  vector<int> agent_last_timestep(num_of_agents, 0);
  vector<int> agent_last_location = start_locations;
  vector<int> task_complete_timestep(num_of_tasks, -1);
  task_plan.clear();
  task_plan.resize(num_of_agents);

  unordered_map<int, vector<int>> dependent;
  for (auto dependence : temporal_dependecies) {
    int i, j;
    std::tie(i, j) = dependence;
    dependent[j].push_back(i);
  }

  std::priority_queue<pair<int, int>, std::vector<pair<int, int>>,
                      std::greater<pair<int, int>>>
      q;

  for (int i = 0; i < num_of_agents; i++) {
    q.push({0, i});
  }

  int task_cnt = 0;

  while (task_cnt < num_of_tasks) {
    int timestep, i;
    std::tie(timestep, i) = q.top();

    cout << "planing for agent " << i << " at timestep " << timestep << endl;

    int loc = agent_last_location[i];
    q.pop();

    int selected_task = -1;
    int selected_task_timestep = -1;
    for (int j = 0; j < num_of_tasks; j++) {
      if (task_complete_timestep[j] != -1) {
        continue;
      }
      int task_timestep =
          agent_last_timestep[i] + search_engine->my_heuristic[j][loc];
      bool task_ready = true;

      if (dependent.find(j) != dependent.end()) {
        for (auto k : dependent[j]) {
          if (task_complete_timestep[k] < 0) {
            task_ready = false;
            break;
          }
          task_timestep = max(task_complete_timestep[k], task_timestep);
        }
      }
      if (task_ready &&
          (selected_task == -1 || task_timestep < selected_task_timestep)) {
        selected_task = j;
        selected_task_timestep = task_timestep;
      }
    }
    // assign the task
    if (selected_task != -1) {
      cout << "assign task " << selected_task << " to agent " << i
           << " with distance "
           << search_engine->my_heuristic[selected_task][loc] << endl;
      task_plan[i].push_back(selected_task);
      agent_last_timestep[i] = selected_task_timestep;
      task_complete_timestep[selected_task] = selected_task_timestep;
      agent_last_location[i] = task_locations[selected_task];
      task_cnt++;
    }
    q.push({agent_last_timestep[i], i});
  }

  // check whether two agent has same goal location

  bool flag = true;
  while (flag) {
    flag = false;
    for (int i = 0; i < num_of_agents; i++) {
      for (int j = i + 1; j < num_of_agents; j++) {
        if (agent_last_location[i] == agent_last_location[j]) {
          assert(false);
          //   flag = true;
          //   auto task1 = task_plan[i].back();
          //   auto task2 = task_plan[j].back();
          //   if (task_complete_timestep[task1] < task_complete_timestep[task2]){
          //     task_plan[j].pop_back();
          //     task_plan[i].push_back(task2);
          //     agent_last_location[j] = task_locations[task_plan[j].back()];
          //   } else {
          //     task_plan[i].pop_back();
          //     task_plan[j].push_back(task1);
          //     agent_last_location[i] = task_locations[task_plan[i].back()];
          //   }
        }
      }
    }
  }

  if (!buildGoalsAndTemporalConstraintsFromTaskPlan()) {
    cerr << "Failed to build goal/temporal structures from greedy task plan"
         << endl;
    assert(false);
  }
}
