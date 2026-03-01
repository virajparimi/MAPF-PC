#pragma once

#include "Instance.h"
#include "SpaceTimeAStar.h"

class TaskAssignment : public Instance {

 public:
  TaskAssignment(const string& map_fname, const string& agent_fname,
                 int num_of_agents = 0);
  void find_greedy_plan();
  bool loadFixedAssignmentFromFile(const string& assignment_fname);
  vector<vector<int>> getTaskPlans() { return task_plan; }

 protected:
  // bool loadMap();
  int num_of_tasks;

  std::unique_ptr<SingleAgentSolver> search_engine;

  vector<int> task_locations, end_points;
  vector<vector<int>> task_plan;

  vector<pair<int, int>> temporal_dependecies;
  bool loadKivaMap();
  bool loadAgents();
  bool loadKivaAgentsAndTasks();
  bool buildGoalsAndTemporalConstraintsFromTaskPlan();
};
