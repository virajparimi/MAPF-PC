#pragma once

#include "ReservationTable.h"
#include "SingleAgentSolver.h"
#include <cstdlib>

class MultiLabelSIPPNode : public LLNode {
 public:
  struct secondary_compare_node {
    bool operator()(const MultiLabelSIPPNode* n1,
                    const MultiLabelSIPPNode* n2) const {
      bool use_lns2_order = true;
      if (const char* env = std::getenv("MAPFPC_LL_FOCAL_USE_LNS2")) {
        use_lns2_order = (std::atoi(env) != 0);
      }
      if (!use_lns2_order) {
        return LLNode::secondary_compare_node()(n1, n2);
      }
      if (n1->num_of_conflicts == n2->num_of_conflicts) {
        if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
          if (n1->h_val == n2->h_val) {
            return rand() % 2 == 0;
          }
          return n1->h_val >= n2->h_val;
        }
        return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
      }
      return n1->num_of_conflicts >= n2->num_of_conflicts;
    }
  };

  typedef pairing_heap<MultiLabelSIPPNode*, compare<LLNode::compare_node>>::handle_type
      open_handle_t;
  typedef pairing_heap<MultiLabelSIPPNode*,
                       compare<MultiLabelSIPPNode::secondary_compare_node>>::handle_type
      focal_handle_t;
  open_handle_t open_handle;
  focal_handle_t focal_handle;

  int high_generation = 0;
  int high_expansion = 0;
  bool collision_v = false;
  bool terminal_goal = false;

  MultiLabelSIPPNode() : LLNode() {}
  MultiLabelSIPPNode(int location, int g_val, int h_val, LLNode* parent,
                     int timestep, unsigned int stage, int num_of_conflicts,
                     int high_generation, int high_expansion, bool collision_v)
      : LLNode(location, g_val, h_val, parent, timestep, stage,
               num_of_conflicts, false),
        high_generation(high_generation),
        high_expansion(high_expansion),
        collision_v(collision_v) {}
  ~MultiLabelSIPPNode() {}

  void copy(const MultiLabelSIPPNode& other) {
    LLNode::copy(other);
    stage = other.stage;
    dist_to_next = other.dist_to_next;
    timestamps = other.timestamps;
    secondary_keys = other.secondary_keys;
    high_generation = other.high_generation;
    high_expansion = other.high_expansion;
    collision_v = other.collision_v;
    terminal_goal = other.terminal_goal;
  }

  struct NodeHasher {
    size_t operator()(const MultiLabelSIPPNode* node) const {
      // Hash fields must match EqNode fields exactly.
      size_t seed = std::hash<int>()(node->location);
      seed ^= std::hash<int>()((int)node->stage + 0x9e3779b9 + (seed << 6) +
                               (seed >> 2));
      seed ^= std::hash<int>()(node->high_generation + 0x9e3779b9 +
                               (seed << 6) + (seed >> 2));
      seed ^= std::hash<int>()((int)node->wait_at_goal + 0x9e3779b9 +
                               (seed << 6) + (seed >> 2));
      seed ^= std::hash<int>()((int)node->terminal_goal + 0x9e3779b9 +
                               (seed << 6) + (seed >> 2));
      return seed;
    }
  };

  struct EqNode {
    bool operator()(const MultiLabelSIPPNode* lhs,
                    const MultiLabelSIPPNode* rhs) const {
      return (lhs == rhs) ||
             (lhs && rhs && lhs->location == rhs->location &&
              lhs->stage == rhs->stage &&
              lhs->wait_at_goal == rhs->wait_at_goal &&
              lhs->terminal_goal == rhs->terminal_goal &&
              lhs->high_generation == rhs->high_generation);
    }
  };
};

class MultiLabelSIPP : public SingleAgentSolver {
 public:
  MultiLabelSIPP(const Instance& instance, int agent)
      : SingleAgentSolver(instance, agent), agent_(agent) {}

  Path findPath(const CBSNode& node, const ConstraintTable& initial_constraints,
                const vector<Path*>& paths, int agent,
                int lower_bound) override;
  Path findPathSegment(ConstraintTable& constraint_table, int start_time,
                       int stage, int lowerbound) override;
  int getTravelTime(int start, int end, const ConstraintTable& constraint_table,
                    int upper_bound) override;

  string getName() const override { return "SIPP"; }

 private:
  typedef pairing_heap<MultiLabelSIPPNode*,
                       compare<MultiLabelSIPPNode::compare_node>>
      heap_open_t;
  typedef pairing_heap<MultiLabelSIPPNode*,
                       compare<MultiLabelSIPPNode::secondary_compare_node>>
      heap_focal_t;

  typedef unordered_map<MultiLabelSIPPNode*, list<MultiLabelSIPPNode*>,
                        MultiLabelSIPPNode::NodeHasher,
                        MultiLabelSIPPNode::EqNode>
      hashtable_t;

  heap_open_t open_list_;
  heap_focal_t focal_list_;
  hashtable_t allNodes_table_;
  list<MultiLabelSIPPNode*> stale_nodes_;
  int min_f_val_ = 0;
  double w_ = 1.0;
  int agent_ = -1;

  inline void pushNodeToOpenAndFocal(MultiLabelSIPPNode* node);
  inline void pushNodeToFocalOnly(MultiLabelSIPPNode* node);
  inline void eraseNodeFromLists(MultiLabelSIPPNode* node);
  void updateFocalList();
  void releaseNodes();
  bool dominanceCheck(MultiLabelSIPPNode* new_node);
  void updatePath(const LLNode* goal, Path& path, bool collect_stage_timestamps);
};
