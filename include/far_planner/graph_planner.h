#ifndef GRAPH_PLANNER_H
#define GRAPH_PLANNER_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_graph.h"
#include "goal_adjustment_policy.h"

#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_set>

enum ReachVote {
    BLOCK = 0,
    REACH = 1
};

struct GraphPlannerParams {
    GraphPlannerParams() = default;
    float converge_dist;
    float adjust_radius;
    bool  enable_goal_adjustment = false;
    float adjust_check_period = 1.0f;
    float adjust_sample_spacing = 0.5f;
    int   adjust_max_candidates = 32;
    int   adjust_block_confirmations = 2;
    int   restore_confirmations = 3;
    bool  adjust_on_dynamic_obstacle = false;
    float momentum_dist;
    bool  is_autoswitch;
    int   free_thred;
    int   votes_size;
    int   momentum_thred;
};

enum class GoalPointStatus {
    UNKNOWN = 0,
    FREE,
    STATIC_BLOCKED,
    DYNAMIC_BLOCKED
};

// std::priority_queue must never order entries through NavNode::gscore: that
// value is changed while the node is already inside the heap and therefore
// silently invalidates the heap ordering.  Keep an immutable distance
// snapshot in every entry and lazily discard obsolete snapshots when popped.
constexpr float kGraphSearchCostEpsilon = 1e-6f;

struct GraphSearchQueueEntry {
    float cost = std::numeric_limits<float>::max();
    std::size_t node_id = 0;
    NavNodePtr node;
};

struct GraphSearchQueueEntryGreater {
    bool operator()(const GraphSearchQueueEntry& lhs,
                    const GraphSearchQueueEntry& rhs) const {
        if (std::fabs(lhs.cost - rhs.cost) > kGraphSearchCostEpsilon) {
            return lhs.cost > rhs.cost;
        }
        return lhs.node_id > rhs.node_id;
    }
};

inline bool IsStaleGraphSearchEntry(const GraphSearchQueueEntry& entry,
                                    const float current_cost) {
    return entry.cost > current_cost + kGraphSearchCostEpsilon;
}

inline bool ShouldRelaxGraphSearchEdge(const float candidate_cost,
                                       const float current_cost,
                                       const std::size_t candidate_parent_id,
                                       const NavNodePtr& current_parent) {
    if (candidate_cost < current_cost - kGraphSearchCostEpsilon) return true;
    if (std::fabs(candidate_cost - current_cost) >
        kGraphSearchCostEpsilon) {
        return false;
    }
    return !current_parent || candidate_parent_id < current_parent->id;
}


class GraphPlanner {
private:
ros::NodeHandle nh_;
ros::Subscriber attemptable_sub_;
GraphPlannerParams gp_params_;
NavNodePtr odom_node_ptr_  = NULL;

// goal related values
NavNodePtr goal_node_ptr_   = NULL;
Point3D origin_goal_pos_    = Point3D(0,0,0);
Point3D active_goal_pos_    = Point3D(0,0,0);
bool is_use_internav_goal_  = false;
bool command_is_free_nav_   = false;
bool is_goal_in_freespace_  = false;
bool is_terrain_associated_ = false;
bool is_goal_init_;
bool is_goal_adjusted_ = false;
bool is_active_goal_explicitly_blocked_ = false;
ros::Time last_goal_adjust_check_;
int goal_blocked_confirmations_ = 0;
int goal_restore_confirmations_ = 0;
NodePtrStack current_graph_;
bool is_free_nav_goal_;

// Nodes whose goal-only state must be reset when the command changes.
NodePtrStack evaluated_goal_candidates_;

// local terrain map for freespace adjustment
Point3D grid_center_ = Point3D(0,0,0);
std::unique_ptr<grid_ns::Grid<char>> free_terrain_grid_;

float PriorityScore(const NavNodePtr& node_ptr);

bool ReconstructPath(const NavNodePtr& goal_node_ptr,
                     const bool& is_free_nav,
                     NodePtrStack& global_path);

bool IsNodeConnectInFree(const NavNodePtr& current_node,
                         const NavNodePtr& neighbor_node);

GoalPointStatus ClassifyGoalPoint(const Point3D& point) const;

std::vector<Point3D> CollectSparseGoalAdjustmentCandidates() const;

bool IsCandidateReachable(const NavNodePtr& goal_ptr,
                          const Point3D& candidate);

bool FindGoalAdjustmentCandidate(const NavNodePtr& goal_ptr,
                                 Point3D& selected,
                                 std::size_t& evaluated);

void SetActiveGoalPosition(const NavNodePtr& goal_ptr,
                           const Point3D& position,
                           bool adjusted);

    EdgeValidationResult ValidateConnectToGoal(
        const NavNodePtr& node_ptr, const NavNodePtr& goal_node_ptr);

NodePtrStack SelectGoalConnectionCandidates(const NavNodePtr& goal_ptr) const;

void RemoveGoalConnection(const NavNodePtr& node_ptr,
                          const NavNodePtr& goal_ptr,
                          const bool clear_vote_history);

NavNodePtr NextNavWaypointFromPath(const NodePtrStack& global_path, const NavNodePtr goal_ptr);

void AttemptStatusCallBack(const std_msgs::Bool& msg);

inline bool IsInvalidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        if (node_ptr1->invalid_boundary.find(node_ptr2->id) != node_ptr1->invalid_boundary.end()) {
            return true;
        }
    }
    return false;
}

inline void ResetFreeTerrainGridOrigin(const Point3D& p) {
    Eigen::Vector3d grid_origin;
    grid_origin.x() = p.x - (free_terrain_grid_->GetResolution().x() * free_terrain_grid_->GetSize().x()) / 2.0f;
    grid_origin.y() = p.y - (free_terrain_grid_->GetResolution().y() * free_terrain_grid_->GetSize().y()) / 2.0f;
    grid_origin.z() = p.z - (free_terrain_grid_->GetResolution().z() * free_terrain_grid_->GetSize().z()) / 2.0f;
    free_terrain_grid_->SetOrigin(grid_origin);
    grid_center_ = p;
}

/* define inline functions */
inline void InitNodesStates(const NodePtrStack& graph) {
    for (const auto& node_ptr : graph) {
        node_ptr->gscore              = FARUtil::kINF;
        node_ptr->fgscore             = FARUtil::kINF;
        node_ptr->is_traversable      = false;
        node_ptr->is_free_traversable = false;
        node_ptr->parent              = NULL;
        node_ptr->free_parent         = NULL;
    }
}

inline bool IsResetBlockStatus(const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    if (node_ptr->is_odom || (node_ptr->is_near_nodes && (!node_ptr->is_finalized || node_ptr->is_frontier))) return true;
    if (!FARUtil::IsStaticEnv && node_ptr->is_near_nodes) return true;
    const auto it = node_ptr->edge_votes.find(goal_ptr->id);
    if (node_ptr->is_near_nodes && it != node_ptr->edge_votes.end() && it->second.size() < gp_params_.votes_size) {
        return true;
    }
    return false;
}

inline float EulerCost(const NavNodePtr& current_node,
                       const NavNodePtr& neighbor_node) 
{
    const auto state = current_node->edge_states.find(neighbor_node->id);
    if (state != current_node->edge_states.end() &&
        state->second.has_clearance_geometry &&
        state->second.route_cost > FARUtil::kEpsilon) {
        return state->second.route_cost;
    }
    return (current_node->position - neighbor_node->position).norm();
}

inline void GoalReset() {
    origin_goal_pos_ = Point3D(0,0,0);
    active_goal_pos_ = Point3D(0,0,0);
    is_goal_in_freespace_ = false;
    is_goal_adjusted_ = false;
    is_active_goal_explicitly_blocked_ = false;
    last_goal_adjust_check_ = ros::Time(0);
    goal_blocked_confirmations_ = 0;
    goal_restore_confirmations_ = 0;
    for (const auto& node_ptr : evaluated_goal_candidates_) {
        if (node_ptr) node_ptr->is_block_to_goal = false;
    }
    evaluated_goal_candidates_.clear();
    if (goal_node_ptr_ != NULL) {
        if (!is_use_internav_goal_) DynamicGraph::ClearGoalNodeInGraph(goal_node_ptr_);
        else goal_node_ptr_->is_goal = false;
    }
    goal_node_ptr_ = NULL;
}

public:

GraphPlanner() = default;
~GraphPlanner() = default;

void Init(const ros::NodeHandle& nh, const GraphPlannerParams& params);


/**
 * Update Global Graph
 * @param vgraph current graph
*/
void UpdaetVGraph(const NodePtrStack& vgraph);

/**
 * Update Global Graph Traversability Status and gscores
 * @param graph current graph
 * @param odom_node_ptr current odom node ptr
 * @param goal_ptr current goal node ptr
*/
void UpdateGraphTraverability(const NavNodePtr& odom_node_ptr, const NavNodePtr& goal_ptr);

/**
 * Generate path to goal based on traversibility result
 * @param goal_ptr current goal node
 * @param global_path(return) return the global path from odom node position
 * @param _nav_node_ptr(return) current navigation waypoint
 * @param _goal_p(return) current active goal position (original or safe substitute)
 * @param _is_fail(return) whether the planner fails to find the path
 * @param has_dynamic_obstacles whether a transient semantic obstacle is active
 * @param _is_retry_wait(return) whether the goal is retained while stopped for replanning
 * @param _is_succeed(return) whether the vehicle has reached the goal
 * @param _is_free_nav(return) the attemptable navigation status (True)->Non-attempts
 * @return whether or not planning success -> publish a valid path for navigation
*/

bool PathToGoal(const NavNodePtr& goal_ptr,
                NodePtrStack& global_path,
                NavNodePtr&   _nav_node_ptr,
                Point3D&      _goal_p,
                bool&         _is_fails,
                const bool&   has_dynamic_obstacles,
                bool&         _is_retry_wait,
                bool&         _is_succeed,
                bool&         _is_free_nav);

/**
 * @brief Update connectivity between goal node and navigation graph
 * @param goal_ptr new create goal node pointer
 * @param updated_graph current updated navigation graph
 */

void UpdateGoalNavNodeConnects(const NavNodePtr& goal_ptr);

/**
 * Graph planner goal update API
 * @param goal goal position
*/ 
void UpdateGoal(const Point3D& goal);


/**
 * @brief Update the free-terrain grid used to evaluate fixed-goal connectivity
 * @param center current center of grid
 * @param obsCloudIn Obstalce point cloud input
 * @param freeCloudIn Free point cloud input
 */
void UpdateFreeTerrainGrid(const Point3D& center,
                           const PointCloudPtr& obsCloudIn, 
                           const PointCloudPtr& freeCloudIn);

/**
 * @brief Refresh the goal's terrain height while preserving the commanded XY
 * @param goal_ptr current goal node pointer
 * @param is_adjust_height whether or not adjust height of goal -> (False if multi layer planning)
 */
void ReEvaluateGoalPosition(const NavNodePtr& goal_ptr, const bool& is_adjust_height);

/** Low-frequency policy update for an explicitly occupied user goal.  The
 * current active goal is still collision-checked by normal goal-edge rebuilds
 * every planning cycle; only the more expensive substitute search is rate
 * limited here. */
void UpdateGoalAdjustment(const NavNodePtr& goal_ptr,
                          const ros::Time& now);

/** Resolve the first current-path edge that owns explicit checked geometry
 * (start, goal or contour-follow) to the route used by graph search. */
bool NextContourRouteWaypoint(const NodePtrStack& global_path,
                              const Point3D& robot_position,
                              Point3D& waypoint) const;

/**
 * @brief Reset internal values and containers
 */
inline void ResetPlannerInternalValues() {
    goal_node_ptr_ = NULL; 
    odom_node_ptr_ = NULL;
    
    is_goal_init_         = false;
    is_use_internav_goal_ = false;
    is_free_nav_goal_     = false;
    is_goal_in_freespace_ = false;
    
    current_graph_.clear(); 
    evaluated_goal_candidates_.clear();
    origin_goal_pos_    = Point3D(0,0,0);
    active_goal_pos_    = Point3D(0,0,0);
    is_goal_adjusted_ = false;
    is_active_goal_explicitly_blocked_ = false;
    last_goal_adjust_check_ = ros::Time(0);
    goal_blocked_confirmations_ = 0;
    goal_restore_confirmations_ = 0;
}

const NavNodePtr& GetGoalNodePtr() const { return goal_node_ptr_;};
bool IsGoalAdjusted() const { return is_goal_adjusted_; }
Point3D GetActiveGoalPos() const { return active_goal_pos_; }

// 
Point3D GetOriginNodePos(const bool& /*is_adjusted_z*/) const {
    if (goal_node_ptr_ == NULL) return Point3D(0,0,0);
    return origin_goal_pos_;
}

};

#endif
