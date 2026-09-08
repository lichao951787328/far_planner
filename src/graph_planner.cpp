/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/graph_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

/***************************************************************************************/

const char INIT_BIT = char(0); // 0000
const char OBS_BIT  = char(1); // 0001
const char FREE_BIT = char(2); // 0010

namespace {

const char* EdgeRejectReasonName(const EdgeRejectReason reason) {
    switch (reason) {
        case EdgeRejectReason::NONE: return "accepted";
        case EdgeRejectReason::NOT_CURRENT_ADJACENT: return "not_adjacent";
        case EdgeRejectReason::UNREACHABLE: return "unreachable";
        case EdgeRejectReason::DIRECTION_REJECTED: return "direction";
        case EdgeRejectReason::DIRECTION_SPARSIFIED: return "direction_sparse";
        case EdgeRejectReason::STATIC_CLOUD_BLOCKED: return "static_cloud";
        case EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED: return "dynamic_cloud";
        case EdgeRejectReason::POLYGON_BLOCKED: return "polygon";
        case EdgeRejectReason::TERRAIN_BLOCKED: return "terrain";
        case EdgeRejectReason::OFFSET_FAILED: return "offset";
        case EdgeRejectReason::SELF_POLYGON_BLOCKED: return "self_polygon";
        case EdgeRejectReason::OTHER_STATIC_BLOCKED: return "other_static";
        case EdgeRejectReason::CLIPPED_CONTOUR: return "clipped";
        case EdgeRejectReason::VOTE_PENDING: return "vote";
    }
    return "unknown";
}

}  // namespace


void GraphPlanner::Init(const ros::NodeHandle& nh, const GraphPlannerParams& params) {
    nh_ = nh;
    gp_params_ = params;
    is_goal_init_ = false;
    current_graph_.clear();
    evaluated_goal_candidates_.clear();
    // attemptable planning listener
    attemptable_sub_ = nh_.subscribe("planning_attemptable", 5, &GraphPlanner::AttemptStatusCallBack, this);
    // initialize terrian grid
    const int col_num = std::ceil(gp_params_.adjust_radius * 2.0f / FARUtil::kLeafSize);
    Eigen::Vector3i grid_size(col_num, col_num, 1);
    Eigen::Vector3d grid_origin(0,0,0);
    Eigen::Vector3d grid_resolution(FARUtil::kLeafSize, FARUtil::kLeafSize, FARUtil::kLeafSize);
    free_terrain_grid_ = std::make_unique<grid_ns::Grid<char>>(grid_size, INIT_BIT, grid_origin, grid_resolution, 3);
}

GoalPointStatus GraphPlanner::ClassifyGoalPoint(
    const Point3D& point) const {
    // Collision authority is checked before terrain.  A wall remains an
    // explicit blocker even if the floor below it is absent from this local
    // terrain snapshot.
    if (!ContourGraph::IsPointCollisionFreeStaticLayer(point)) {
        return GoalPointStatus::STATIC_BLOCKED;
    }
    if (!ContourGraph::IsPointCollisionFreeDynamicLayer(point)) {
        return GoalPointStatus::DYNAMIC_BLOCKED;
    }
    if (!free_terrain_grid_) return GoalPointStatus::UNKNOWN;
    const Eigen::Vector3i sub = free_terrain_grid_->Pos2Sub(
        point.x, point.y, grid_center_.z);
    if (!free_terrain_grid_->InRange(sub)) {
        return GoalPointStatus::UNKNOWN;
    }
    const char cell = free_terrain_grid_->GetCellValue(sub);
    if ((cell & OBS_BIT) != 0) {
        return GoalPointStatus::STATIC_BLOCKED;
    }
    if ((cell & FREE_BIT) == 0) {
        return GoalPointStatus::UNKNOWN;
    }
    return GoalPointStatus::FREE;
}

std::vector<Point3D>
GraphPlanner::CollectSparseGoalAdjustmentCandidates() const {
    std::vector<Point3D> raw_candidates;
    if (!free_terrain_grid_ || gp_params_.adjust_radius <= 0.0f) {
        return raw_candidates;
    }

    const float radius = gp_params_.adjust_radius;
    for (int index = 0; index < free_terrain_grid_->GetCellNumber(); ++index) {
        const char cell = free_terrain_grid_->GetCellValue(index);
        if ((cell & FREE_BIT) == 0 || (cell & OBS_BIT) != 0) continue;
        const Eigen::Vector3d grid_position =
            free_terrain_grid_->Ind2Pos(index);
        Point3D candidate(static_cast<float>(grid_position.x()),
                          static_cast<float>(grid_position.y()),
                          origin_goal_pos_.z);
        const float offset =
            (candidate - origin_goal_pos_).norm_flat();
        if (offset <= FARUtil::kEpsilon ||
            offset > radius + FARUtil::kEpsilon) {
            continue;
        }
        raw_candidates.push_back(candidate);
    }

    std::sort(raw_candidates.begin(), raw_candidates.end(),
              [this](const Point3D& lhs, const Point3D& rhs) {
                  const float lhs_distance =
                      (lhs - origin_goal_pos_).norm_flat();
                  const float rhs_distance =
                      (rhs - origin_goal_pos_).norm_flat();
                  if (std::fabs(lhs_distance - rhs_distance) >
                      FARUtil::kEpsilon) {
                      return lhs_distance < rhs_distance;
                  }
                  if (std::fabs(lhs.x - rhs.x) > FARUtil::kEpsilon) {
                      return lhs.x < rhs.x;
                  }
                  return lhs.y < rhs.y;
              });

    // Reading every small grid cell is cheap.  Expensive collision and graph
    // checks below see only one representative per coarser XY bucket.
    const float spacing = std::max(
        gp_params_.adjust_sample_spacing,
        static_cast<float>(free_terrain_grid_->GetResolution().x()));
    std::unordered_set<std::uint64_t> occupied_buckets;
    std::vector<Point3D> sparse_candidates;
    sparse_candidates.reserve(raw_candidates.size());
    for (const Point3D& candidate : raw_candidates) {
        const std::int32_t bucket_x = static_cast<std::int32_t>(std::floor(
            (candidate.x - origin_goal_pos_.x) / spacing));
        const std::int32_t bucket_y = static_cast<std::int32_t>(std::floor(
            (candidate.y - origin_goal_pos_.y) / spacing));
        const std::uint64_t key =
            (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bucket_x))
             << 32) |
            static_cast<std::uint32_t>(bucket_y);
        if (occupied_buckets.insert(key).second) {
            sparse_candidates.push_back(candidate);
        }
    }
    return sparse_candidates;
}

bool GraphPlanner::IsCandidateReachable(const NavNodePtr& goal_ptr,
                                        const Point3D& candidate) {
    if (!goal_ptr) return false;
    NavNodePtr candidate_goal = std::make_shared<NavNode>(*goal_ptr);
    candidate_goal->position = candidate;
    NodePtrStack graph_candidates =
        this->SelectGoalConnectionCandidates(candidate_goal);
    graph_candidates.erase(
        std::remove_if(
            graph_candidates.begin(), graph_candidates.end(),
            [](const NavNodePtr& node_ptr) {
                return !node_ptr || node_ptr->is_goal ||
                    !node_ptr->is_traversable;
            }),
        graph_candidates.end());
    std::sort(
        graph_candidates.begin(), graph_candidates.end(),
        [&candidate](const NavNodePtr& lhs, const NavNodePtr& rhs) {
            const float lhs_distance =
                (lhs->position - candidate).norm_flat();
            const float rhs_distance =
                (rhs->position - candidate).norm_flat();
            if (std::fabs(lhs_distance - rhs_distance) >
                FARUtil::kEpsilon) {
                return lhs_distance < rhs_distance;
            }
            return lhs->id < rhs->id;
        });
    for (const NavNodePtr& node_ptr : graph_candidates) {
        const EdgeValidationResult validation =
            this->ValidateConnectToGoal(node_ptr, candidate_goal);
        if (validation.valid) return true;
    }
    return false;
}

bool GraphPlanner::FindGoalAdjustmentCandidate(
    const NavNodePtr& goal_ptr, Point3D& selected,
    std::size_t& evaluated) {
    const std::vector<Point3D> candidates =
        this->CollectSparseGoalAdjustmentCandidates();
    const std::size_t validation_limit = static_cast<std::size_t>(
        std::max(1, gp_params_.adjust_max_candidates));
    const BoundedGoalCandidateSearchResult result =
        FindFirstValidGoalAdjustmentCandidate(
            candidates, validation_limit,
            [this, &goal_ptr, &selected](Point3D candidate) {
                bool terrain_matched = false;
                const float terrain_height =
                    MapHandler::NearestTerrainHeightofNavPoint(
                        candidate, terrain_matched);
                if (!terrain_matched) return false;
                candidate.z = terrain_height + FARUtil::vehicle_height;
                if (this->ClassifyGoalPoint(candidate) !=
                    GoalPointStatus::FREE) {
                    return false;
                }
                if (!this->IsCandidateReachable(goal_ptr, candidate)) {
                    return false;
                }
                selected = candidate;
                return true;
            });
    evaluated = result.evaluated;
    return result.found;
}

void GraphPlanner::SetActiveGoalPosition(const NavNodePtr& goal_ptr,
                                         const Point3D& position,
                                         const bool adjusted) {
    if (!goal_ptr) return;
    active_goal_pos_ = position;
    goal_ptr->position = position;
    is_goal_adjusted_ = adjusted;
    is_terrain_associated_ = true;
}

void GraphPlanner::UpdaetVGraph(const NodePtrStack& vgraph) {
    current_graph_ = vgraph;
}

NodePtrStack GraphPlanner::SelectGoalConnectionCandidates(
    const NavNodePtr& goal_ptr) const {
    NodePtrStack selected;
    if (!goal_ptr) return selected;

    // Match the original FAR planner: validate the goal against every node in
    // the current graph snapshot.  Semantic source filtering replaces the
    // legacy trajectory nodes, which are intentionally absent in this build.
    selected.reserve(current_graph_.size());
    for (const auto& node_ptr : current_graph_) {
        if (!node_ptr || node_ptr == goal_ptr) continue;
        if (node_ptr->is_odom || IsGoalConnectionCandidate(*node_ptr)) {
            selected.push_back(node_ptr);
        }
    }
    return selected;
}

void GraphPlanner::RemoveGoalConnection(const NavNodePtr& node_ptr,
                                        const NavNodePtr& goal_ptr,
                                        const bool clear_vote_history) {
    if (!node_ptr || !goal_ptr || node_ptr == goal_ptr) return;
    DynamicGraph::ErasePolyEdge(node_ptr, goal_ptr);
    DynamicGraph::EraseEdge(node_ptr, goal_ptr);
    node_ptr->is_block_to_goal = true;
    if (!clear_vote_history) return;

    node_ptr->edge_votes.erase(goal_ptr->id);
    goal_ptr->edge_votes.erase(node_ptr->id);
    FARUtil::EraseNodeFromStack(goal_ptr, node_ptr->potential_edges);
    FARUtil::EraseNodeFromStack(node_ptr, goal_ptr->potential_edges);
}

void GraphPlanner::UpdateGraphTraverability(const NavNodePtr& odom_node_ptr, const NavNodePtr& goal_ptr) 
{
    if (odom_node_ptr == NULL || current_graph_.empty()) {
        ROS_ERROR("GP: Update global graph traversablity fails.");
        return;
    }
    odom_node_ptr_ = odom_node_ptr;
    this->InitNodesStates(current_graph_);
    // A newly commanded goal can be connected before the next ordinary Graph
    // snapshot includes it. Reset it explicitly so removing the per-cycle
    // whole-graph refresh cannot leave a parent/gscore from the prior search.
    if (goal_ptr) {
        goal_ptr->gscore = FARUtil::kINF;
        goal_ptr->fgscore = FARUtil::kINF;
        goal_ptr->is_traversable = false;
        goal_ptr->is_free_traversable = false;
        goal_ptr->parent = NULL;
        goal_ptr->free_parent = NULL;
    }
    // start expand the whole current_graph_
    odom_node_ptr_->gscore = 0.0;
    std::priority_queue<GraphSearchQueueEntry,
                        std::vector<GraphSearchQueueEntry>,
                        GraphSearchQueueEntryGreater> open_queue;
    IdxSet close_set;
    // Expansion from odom node to all reachable navigation node
    open_queue.push({0.0f, odom_node_ptr_->id, odom_node_ptr_});
    while (!open_queue.empty()) {
        const GraphSearchQueueEntry entry = open_queue.top();
        open_queue.pop();
        const NavNodePtr current = entry.node;
        if (!current || close_set.count(current->id) ||
            IsStaleGraphSearchEntry(entry, current->gscore)) {
            continue;
        }
        close_set.insert(current->id);
        current->is_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if ((!goal_ptr && neighbor->is_goal) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor) ||
                close_set.count(neighbor->id) ||
                this->IsInvalidBoundary(current, neighbor)) continue;
            float edist = this->EulerCost(current, neighbor);
            if (neighbor == goal_ptr && edist > FARUtil::kEpsilon && !FARUtil::IsAtSameLayer(neighbor, current)) { // check for multi layer traverse cost
                const Point3D diff_p = neighbor->position - current->position;
                float factor = std::hypotf(diff_p.x, diff_p.y) / edist;
                if (factor > FARUtil::kEpsilon) {
                    edist /= factor;
                } else {
                    continue;
                }
            }
            const float temp_gscore = current->gscore + edist;
            if (ShouldRelaxGraphSearchEdge(temp_gscore, neighbor->gscore,
                                           current->id, neighbor->parent)) {
                neighbor->parent = current;
                neighbor->gscore = temp_gscore;
                // Always enqueue a new immutable snapshot.  An older entry
                // for this node is harmless and is discarded at pop time.
                open_queue.push({temp_gscore, neighbor->id, neighbor});
            }
        }
    }
    std::priority_queue<GraphSearchQueueEntry,
                        std::vector<GraphSearchQueueEntry>,
                        GraphSearchQueueEntryGreater> fopen_queue;
    close_set.clear();
    // Expansion from odom node to all covered navigation node
    odom_node_ptr_->fgscore = 0.0;
    fopen_queue.push({0.0f, odom_node_ptr_->id, odom_node_ptr_});
    while (!fopen_queue.empty()) {
        const GraphSearchQueueEntry entry = fopen_queue.top();
        fopen_queue.pop();
        const NavNodePtr current = entry.node;
        if (!current || close_set.count(current->id) ||
            IsStaleGraphSearchEntry(entry, current->fgscore)) {
            continue;
        }
        close_set.insert(current->id);
        current->is_free_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if ((!goal_ptr && neighbor->is_goal) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor) ||
                !neighbor->is_covered || close_set.count(neighbor->id) ||
                this->IsInvalidBoundary(current, neighbor)) continue;
            const float e_dist = this->EulerCost(current, neighbor);
            if (neighbor == goal_ptr && (!is_goal_in_freespace_ || e_dist > FARUtil::kTerrainRange)) continue;
            const float temp_fgscore = current->fgscore + e_dist;
            if (ShouldRelaxGraphSearchEdge(temp_fgscore, neighbor->fgscore,
                                           current->id,
                                           neighbor->free_parent)) {
                neighbor->free_parent = current;
                neighbor->fgscore = temp_fgscore;
                fopen_queue.push({temp_fgscore, neighbor->id, neighbor});
            }
        }
    }
}

void GraphPlanner::UpdateGoalNavNodeConnects(const NavNodePtr& goal_ptr)
{
    if (goal_ptr == NULL || is_use_internav_goal_) return;

    // Goal connections are a transient query layer. Rebuild them directly
    // from this graph snapshot instead of advancing historical votes on every
    // planning timer tick. Identical inputs now produce identical edges.
    const NodePtrStack previous_candidates = evaluated_goal_candidates_;
    for (const auto& node_ptr : previous_candidates) {
        this->RemoveGoalConnection(node_ptr, goal_ptr, true);
        if (node_ptr) node_ptr->is_block_to_goal = false;
    }
    evaluated_goal_candidates_.clear();

    // Endpoint occupancy is authoritative and must stop motion immediately,
    // including during confirmation hysteresis or while no substitute exists.
    // Do not rely on every contour-corner edge independently rediscovering
    // the same endpoint blocker.
    if (is_active_goal_explicitly_blocked_) {
        ROS_WARN_THROTTLE(
            1.0,
            "GP: active goal endpoint is explicitly occupied; all goal "
            "connections are suppressed while adjustment waits or retries.");
        return;
    }

    NodePtrStack candidates = this->SelectGoalConnectionCandidates(goal_ptr);
    std::size_t accepted_goal_connections = 0;
    float farthest_goal_candidate = 0.0f;
    float farthest_goal_connection = 0.0f;
    bool has_odom_candidate = false;
    bool has_odom_connection = false;
    EdgeRejectReason odom_goal_result = EdgeRejectReason::UNREACHABLE;
    float odom_goal_distance = 0.0f;
    EdgeRejectionStats goal_rejections;

    for (const auto& node_ptr : candidates) {
        if (!node_ptr) continue;
        evaluated_goal_candidates_.push_back(node_ptr);
    }

    for (const auto& node_ptr : candidates) {
        if (!node_ptr || node_ptr == goal_ptr) continue;
        const float candidate_distance =
            (node_ptr->position - goal_ptr->position).norm_flat();
        farthest_goal_candidate = std::max(farthest_goal_candidate,
                                          candidate_distance);
        has_odom_candidate = has_odom_candidate || node_ptr->is_odom;
        EdgeValidationResult validation =
            this->ValidateConnectToGoal(node_ptr, goal_ptr);
        if (node_ptr->is_odom) {
            odom_goal_result = validation.reason;
            odom_goal_distance = candidate_distance;
        }
        if (validation.valid) {
            DynamicGraph::AddPolyEdge(node_ptr, goal_ptr);
            DynamicGraph::AddEdge(node_ptr, goal_ptr);
            GraphEdgeState& forward = node_ptr->edge_states[goal_ptr->id];
            GraphEdgeState& reverse = goal_ptr->edge_states[node_ptr->id];
            forward.source = reverse.source = GraphEdgeSource::GOAL_CONNECT;
            forward.validation_mode = reverse.validation_mode =
                EdgeValidationMode::VISIBILITY;
            forward.has_clearance_geometry =
                reverse.has_clearance_geometry = true;
            forward.route_start = validation.route_start;
            forward.route_end = validation.route_end;
            reverse.route_start = validation.route_end;
            reverse.route_end = validation.route_start;
            forward.route_cost = reverse.route_cost = validation.route_cost;
            forward.static_valid = reverse.static_valid = true;
            forward.dynamic_blocked = reverse.dynamic_blocked = false;
            forward.topology_blocked = reverse.topology_blocked = false;
            forward.active = reverse.active = true;
            node_ptr->is_block_to_goal = false;
            ++accepted_goal_connections;
            farthest_goal_connection = std::max(farthest_goal_connection,
                                                candidate_distance);
            has_odom_connection = has_odom_connection || node_ptr->is_odom;
        } else {
            this->RemoveGoalConnection(node_ptr, goal_ptr, true);
            // This bit is diagnostic only. It must never gate a later
            // snapshot because the obstacle layer can change immediately.
            node_ptr->is_block_to_goal = false;
            goal_rejections.Count(validation.reason);
        }
    }

    ROS_INFO_THROTTLE(
        5.0,
        "GP goal connections: candidates=%zu accepted=%zu odom_candidate=%s odom_edge=%s odom_result=%s odom_goal_distance=%.2fm farthest_candidate=%.2fm farthest_edge=%.2fm reject[unreachable=%zu direction=%zu static_cloud=%zu dynamic_cloud=%zu polygon=%zu terrain=%zu vote=%zu]",
        candidates.size(), accepted_goal_connections,
        has_odom_candidate ? "yes" : "no",
        has_odom_connection ? "yes" : "no",
        has_odom_candidate ? EdgeRejectReasonName(odom_goal_result)
                           : "not_candidate",
        odom_goal_distance,
        farthest_goal_candidate, farthest_goal_connection,
        goal_rejections.unreachable,
        goal_rejections.direction_rejected,
        goal_rejections.static_cloud_blocked,
        goal_rejections.dynamic_cloud_blocked,
        goal_rejections.polygon_blocked,
        goal_rejections.terrain_blocked,
        goal_rejections.vote_pending);

    if (FARUtil::IsDebug) {
        ROS_INFO_THROTTLE(1.0,
            "GP: validating %zu FAR-style full-graph goal candidates.",
            candidates.size());
    }
}

EdgeValidationResult GraphPlanner::ValidateConnectToGoal(
    const NavNodePtr& node_ptr, const NavNodePtr& goal_node_ptr) {
    EdgeValidationResult result;
    if (!node_ptr || !goal_node_ptr || !node_ptr->is_traversable) {
        result.reason = EdgeRejectReason::UNREACHABLE;
        return result;
    }
    if (node_ptr->is_odom) {
        // Odom and goal are robot-centre points, not obstacle anchors.  A
        // direct edge must cover the complete line and must not inherit the
        // contour-corner endpoint collision exclusion.
        result = ContourGraph::ValidateDirectOdomGoalEdgeWithRoute(
            node_ptr, goal_node_ptr, true);
    } else {
        // Surface direction proposes the corner projection inside
        // ContourGraph, but it is not an acceptance gate. Persistent and
        // multi-contour junctions can have a stale or incomplete direction
        // pair; checked static/dynamic geometry remains authoritative.
        result = ContourGraph::ValidateGoalEdgeWithRoute(
            node_ptr, goal_node_ptr);
    }
    if (result.valid &&
        !DynamicGraph::IsOnTerrainRoute(result.route_start,
                                        result.route_end)) {
        result.valid = false;
        result.reason = EdgeRejectReason::TERRAIN_BLOCKED;
    }
    return result;
}

bool GraphPlanner::PathToGoal(const NavNodePtr& goal_ptr,
                              NodePtrStack& global_path,
                              NavNodePtr& _nav_node_ptr,
                              Point3D& _goal_p,
                              bool& _is_fail,
                              const bool& has_dynamic_obstacles,
                              bool& _is_retry_wait,
                              bool& _is_succeed,
                              bool& _is_free_nav) 
{
    if (!is_goal_init_) return false;
    if (odom_node_ptr_ == NULL || goal_ptr == NULL || current_graph_.empty()) {
        ROS_ERROR("GP: Graph or Goal is not initialized correctly.");
        return false;
    }
    _is_fail = false, _is_retry_wait = false, _is_succeed = false;
    global_path.clear();
    _goal_p = goal_ptr->position;
    // When obstacle-aware goal adjustment is active, the substitute is the
    // explicitly selected safe destination.  The original command remains
    // stored separately for restoration and visualization.
    if (!is_active_goal_explicitly_blocked_ &&
        (odom_node_ptr_->position - goal_ptr->position).norm() <
            gp_params_.converge_dist)
    {
        if (FARUtil::IsDebug) ROS_INFO("GP: *********** Goal Reached! ***********");
        global_path.push_back(odom_node_ptr_);
        _goal_p = goal_ptr->position;
        _is_succeed = true;
        global_path.push_back(goal_ptr);
        _nav_node_ptr = goal_ptr;
        _is_free_nav = is_free_nav_goal_;
        this->GoalReset();
        is_goal_init_ = false;
        return true;
    }

    //check free navigation command
    if (!command_is_free_nav_) is_free_nav_goal_ = false;
    else if (goal_ptr->is_free_traversable) is_free_nav_goal_ = true;
    // auto-switch model based on command and navigation status
    if (gp_params_.is_autoswitch && command_is_free_nav_) {
        is_free_nav_goal_ = goal_ptr->is_free_traversable;
    }
    _is_free_nav = is_free_nav_goal_;
    const NavNodePtr reach_nav_node = is_free_nav_goal_ ? goal_ptr->free_parent : goal_ptr->parent;
    if (reach_nav_node != NULL) { // valid path found
        NodePtrStack cur_path;
        if (this->ReconstructPath(goal_ptr, is_free_nav_goal_, cur_path)) {
            _nav_node_ptr = this->NextNavWaypointFromPath(cur_path, goal_ptr);
            global_path = cur_path;
            return true;
        }
    } else { // no valid path found
        if (gp_params_.is_autoswitch && is_free_nav_goal_ &&
            goal_ptr->parent != NULL) {
            is_free_nav_goal_ = false;
            NodePtrStack attemptable_path;
            if (this->ReconstructPath(goal_ptr, false, attemptable_path)) {
                global_path = attemptable_path;
                _nav_node_ptr = this->NextNavWaypointFromPath(global_path, goal_ptr);
                _is_free_nav = false;
                return true;
            }
        }
            // A distant goal can initially lie beyond all mapped contour
            // vertices, and a temporary static-graph stitch can also be
            // absent for one snapshot. Stop safely but retain the command;
            // the next semantic/Graph update gets another chance to connect.
            _is_retry_wait = true;
            _is_fail = true;
            if (FARUtil::IsDebug) {
                if (has_dynamic_obstacles) {
                    ROS_WARN_THROTTLE(1.0,
                        "GP: goal temporarily unreachable while dynamic obstacles are active; retaining it for replanning.");
                } else {
                    ROS_WARN_THROTTLE(1.0,
                        "GP: goal currently has no reachable Graph connection; retaining it for replanning.");
                }
            }
            return false;
    }
    // ReconstructPath can fail transiently while graph connectivity changes
    // between semantic snapshots. Keep the destination but never reuse the
    // previous path.
    _is_retry_wait = true;
    _is_fail = true;
    if (FARUtil::IsDebug) {
        ROS_WARN_THROTTLE(1.0,
            "GP: current Graph path reconstruction failed; retaining goal for replanning.");
    }
    return false;
}

bool GraphPlanner::ReconstructPath(const NavNodePtr& goal_node_ptr,
                                   const bool& is_free_nav,
                                   NodePtrStack& global_path)
{
    if (goal_node_ptr == NULL || (!is_free_nav && goal_node_ptr->parent == NULL) || (is_free_nav && goal_node_ptr->free_parent == NULL)) {
        ROS_ERROR("GP: Critical! reconstruct path error: goal node or its parent equals to NULL.");
        return false;
    }
    global_path.clear();
    NavNodePtr check_ptr = goal_node_ptr;
    global_path.push_back(check_ptr);
    if (is_free_nav) {
        while (true) {
            const NavNodePtr parent_ptr = check_ptr->free_parent;
            if (parent_ptr->free_direct != NodeFreeDirect::CONCAVE) {
                global_path.push_back(parent_ptr);
            }
            if (parent_ptr->free_parent == NULL) break;
            check_ptr = parent_ptr;
        }
    } else {
        while (true) {
            const NavNodePtr parent_ptr = check_ptr->parent;
            if (parent_ptr->free_direct != NodeFreeDirect::CONCAVE) {
                global_path.push_back(parent_ptr);
            }
            if (parent_ptr->parent == NULL) break;
            check_ptr = parent_ptr;
        } 
    }
    std::reverse(global_path.begin(), global_path.end()); 
    return true;
}

NavNodePtr GraphPlanner::NextNavWaypointFromPath(const NodePtrStack& global_path, const NavNodePtr goal_ptr) {
    if (global_path.size() < 2) {
        ROS_ERROR("GP: global path size less than 2.");
        return goal_ptr;
    }
    NavNodePtr nav_point_ptr;
    const std::size_t path_size = global_path.size();
    std::size_t nav_idx = 1;
    nav_point_ptr = global_path[nav_idx];
    float dist = (nav_point_ptr->position - odom_node_ptr_->position).norm();
    while (dist < gp_params_.converge_dist) {
        nav_idx ++;
        if (nav_idx < path_size) {
            nav_point_ptr = global_path[nav_idx];
            dist = (nav_point_ptr->position - odom_node_ptr_->position).norm();
        } else break;
    }
    return nav_point_ptr;
}

bool GraphPlanner::NextContourRouteWaypoint(
    const NodePtrStack& global_path, const Point3D& robot_position,
    Point3D& waypoint) const {
    if (global_path.size() < 2) return false;
    for (std::size_t index = 1; index < global_path.size(); ++index) {
        const NavNodePtr& previous = global_path[index - 1];
        const NavNodePtr& next = global_path[index];
        if (!previous || !next) continue;
        if ((next->position - robot_position).norm() <
            gp_params_.converge_dist) {
            continue;
        }
        const auto state = previous->edge_states.find(next->id);
        if (state == previous->edge_states.end() ||
            !state->second.IsActive() ||
            !state->second.has_clearance_geometry) {
            return false;
        }
        const float route_point_converge = std::max(
            0.15f, gp_params_.converge_dist * 0.4f);
        waypoint = state->second.route_start;
        if ((waypoint - robot_position).norm() < route_point_converge) {
            waypoint = state->second.route_end;
        }
        return true;
    }
    return false;
}

void GraphPlanner::UpdateGoal(const Point3D& goal) {
    this->GoalReset();
    is_use_internav_goal_ = false;
    DynamicGraph::CreateNavNodeFromPoint(goal, goal_node_ptr_, false, false, true);
    DynamicGraph::AddNodeToGraph(goal_node_ptr_);
    if (FARUtil::IsDebug) ROS_INFO("GP: *********** new goal updated ***********");
    is_goal_init_          = true;
    is_terrain_associated_ = false;
    // Keep the user command independent from the graph node selected as the
    // connection endpoint.  In particular, dynamic contours may change the
    // latter's parents and the current waypoint, but not this destination.
    origin_goal_pos_       = goal;
    active_goal_pos_       = goal;
    is_goal_adjusted_      = false;
    is_active_goal_explicitly_blocked_ = false;
    last_goal_adjust_check_ = ros::Time(0);
    goal_blocked_confirmations_ = 0;
    goal_restore_confirmations_ = 0;
    is_free_nav_goal_      = command_is_free_nav_;
    if (!FARUtil::IsMultiLayer) {
        goal_node_ptr_->position.z = MapHandler::NearestTerrainHeightofNavPoint(origin_goal_pos_, is_terrain_associated_) + FARUtil::vehicle_height;
        // Terrain association is the canonical 2.5D height of the same goal;
        // its XY coordinates remain exactly those commanded by the user.
        origin_goal_pos_.z = goal_node_ptr_->position.z;
        active_goal_pos_.z = goal_node_ptr_->position.z;
    }
    this->ResetFreeTerrainGridOrigin(goal_node_ptr_->position);
}

void GraphPlanner::ReEvaluateGoalPosition(const NavNodePtr& goal_ptr, const bool& is_adjust_height)
{
    if (is_use_internav_goal_) return; // return if using an exsiting internav node as goal
    if (is_adjust_height) {
        bool origin_terrain_matched = false;
        const float origin_terrain_height =
            MapHandler::NearestTerrainHeightofNavPoint(
                origin_goal_pos_, origin_terrain_matched);
        if (origin_terrain_matched) {
            origin_goal_pos_.z =
                origin_terrain_height + FARUtil::vehicle_height;
        }
        Point3D height_query = is_goal_adjusted_
            ? active_goal_pos_ : origin_goal_pos_;
        bool active_terrain_matched = false;
        const float active_terrain_height =
            MapHandler::NearestTerrainHeightofNavPoint(
                height_query, active_terrain_matched);
        if (active_terrain_matched) {
            active_goal_pos_.z =
                active_terrain_height + FARUtil::vehicle_height;
            is_terrain_associated_ = true;
        }
    }
    if (!is_goal_adjusted_) active_goal_pos_ = origin_goal_pos_;
    goal_ptr->position = active_goal_pos_;
}

void GraphPlanner::UpdateGoalAdjustment(const NavNodePtr& goal_ptr,
                                        const ros::Time& now) {
    if (!goal_ptr || !is_goal_init_ ||
        !gp_params_.enable_goal_adjustment ||
        gp_params_.adjust_radius <= 0.0f || FARUtil::IsMultiLayer) {
        is_active_goal_explicitly_blocked_ = false;
        return;
    }

    const GoalPointStatus active_status =
        this->ClassifyGoalPoint(active_goal_pos_);
    is_active_goal_explicitly_blocked_ =
        active_status == GoalPointStatus::STATIC_BLOCKED ||
        active_status == GoalPointStatus::DYNAMIC_BLOCKED;
    const bool active_became_blocked = is_goal_adjusted_ &&
        (active_status == GoalPointStatus::STATIC_BLOCKED ||
         active_status == GoalPointStatus::DYNAMIC_BLOCKED);
    const bool clock_rewound = !last_goal_adjust_check_.isZero() &&
        now < last_goal_adjust_check_;
    const bool period_elapsed = last_goal_adjust_check_.isZero() ||
        gp_params_.adjust_check_period <= 0.0f || clock_rewound ||
        (now - last_goal_adjust_check_).toSec() >=
            gp_params_.adjust_check_period;
    if (!period_elapsed && !active_became_blocked) return;
    last_goal_adjust_check_ = now;

    const GoalPointStatus origin_status =
        this->ClassifyGoalPoint(origin_goal_pos_);

    if (is_goal_adjusted_) {
        bool original_reachable = false;
        if (origin_status == GoalPointStatus::FREE) {
            original_reachable =
                this->IsCandidateReachable(goal_ptr, origin_goal_pos_);
        }
        if (origin_status == GoalPointStatus::FREE && original_reachable) {
            goal_restore_confirmations_ = std::min(
                std::max(1, gp_params_.restore_confirmations),
                goal_restore_confirmations_ + 1);
        } else {
            goal_restore_confirmations_ = 0;
        }
        if (goal_restore_confirmations_ >=
            std::max(1, gp_params_.restore_confirmations)) {
            const Point3D previous = active_goal_pos_;
            this->SetActiveGoalPosition(goal_ptr, origin_goal_pos_, false);
            is_active_goal_explicitly_blocked_ = false;
            goal_blocked_confirmations_ = 0;
            goal_restore_confirmations_ = 0;
            ROS_INFO("GP: original goal is explicitly free and reachable; "
                     "restored (%.2f, %.2f, %.2f) from adjusted goal "
                     "(%.2f, %.2f, %.2f).",
                     origin_goal_pos_.x, origin_goal_pos_.y,
                     origin_goal_pos_.z, previous.x, previous.y, previous.z);
            return;
        }

        if (!active_became_blocked) return;
        Point3D replacement;
        std::size_t evaluated = 0;
        if (this->FindGoalAdjustmentCandidate(
                goal_ptr, replacement, evaluated)) {
            const Point3D previous = active_goal_pos_;
            this->SetActiveGoalPosition(goal_ptr, replacement, true);
            is_active_goal_explicitly_blocked_ = false;
            ROS_WARN("GP: adjusted goal became occupied; replaced "
                     "(%.2f, %.2f) with (%.2f, %.2f) after %zu complete "
                     "candidate checks.",
                     previous.x, previous.y, replacement.x, replacement.y,
                     evaluated);
        } else {
            ROS_WARN_THROTTLE(
                1.0,
                "GP: adjusted goal is occupied and no safe reachable "
                "replacement was found within %.2f m after %zu complete "
                "candidate checks; normal goal-edge validation will stop "
                "the robot.",
                gp_params_.adjust_radius, evaluated);
        }
        return;
    }

    const bool adjustment_trigger =
        origin_status == GoalPointStatus::STATIC_BLOCKED ||
        (gp_params_.adjust_on_dynamic_obstacle &&
         origin_status == GoalPointStatus::DYNAMIC_BLOCKED);
    if (!adjustment_trigger) {
        goal_blocked_confirmations_ = 0;
        return;
    }
    goal_blocked_confirmations_ = std::min(
        std::max(1, gp_params_.adjust_block_confirmations),
        goal_blocked_confirmations_ + 1);
    if (goal_blocked_confirmations_ <
        std::max(1, gp_params_.adjust_block_confirmations)) {
        ROS_WARN_THROTTLE(
            1.0,
            "GP: original goal is explicitly occupied (%d/%d confirmations); "
            "retaining it while normal edge validation stops the robot.",
            goal_blocked_confirmations_,
            std::max(1, gp_params_.adjust_block_confirmations));
        return;
    }

    Point3D replacement;
    std::size_t evaluated = 0;
    if (!this->FindGoalAdjustmentCandidate(
            goal_ptr, replacement, evaluated)) {
        ROS_WARN_THROTTLE(
            1.0,
            "GP: original goal is occupied but no safe reachable substitute "
            "was found within %.2f m after %zu complete candidate checks; "
            "retaining the command for a later retry.",
            gp_params_.adjust_radius, evaluated);
        return;
    }

    this->SetActiveGoalPosition(goal_ptr, replacement, true);
    is_active_goal_explicitly_blocked_ = false;
    goal_restore_confirmations_ = 0;
    ROS_WARN("GP: original goal (%.2f, %.2f, %.2f) is occupied; selected "
             "reachable substitute (%.2f, %.2f, %.2f), offset %.2f m, "
             "after %zu complete candidate checks.",
             origin_goal_pos_.x, origin_goal_pos_.y, origin_goal_pos_.z,
             replacement.x, replacement.y, replacement.z,
             (replacement - origin_goal_pos_).norm_flat(), evaluated);
}

void GraphPlanner::AttemptStatusCallBack(const std_msgs::Bool& msg) {
    if (command_is_free_nav_ && msg.data) { // current goal is not attemptable
        if (FARUtil::IsDebug) ROS_WARN("GP: switch to attemptable planning mode.");
        command_is_free_nav_ = false;
    } 
    if (!command_is_free_nav_ && !msg.data) { // current attemptable planning
        if (FARUtil::IsDebug) ROS_WARN("GP: planning without attempting.");
        command_is_free_nav_ = true; 
    }
}

void GraphPlanner::UpdateFreeTerrainGrid(const Point3D& center,
                                         const PointCloudPtr& obsCloudIn, 
                                         const PointCloudPtr& freeCloudIn) 
{
    // reset grid
    const Point3D origin_center(origin_goal_pos_.x, origin_goal_pos_.y, center.z);
    this->ResetFreeTerrainGridOrigin(origin_center);
    free_terrain_grid_->ReInitGrid(INIT_BIT);
    // evaluate goal freespace status
    is_goal_in_freespace_ = false;
    if (!freeCloudIn->empty() || !obsCloudIn->empty()) { // process terrain cloud
        const int C_IF = FARUtil::kObsInflate;
        for (const auto& point : obsCloudIn->points) { // Set obstacle cloud in free terrain grid
            Eigen::Vector3i c_sub = free_terrain_grid_->Pos2Sub(point.x, point.y, grid_center_.z);
            for (int i = -C_IF; i <= C_IF; i++) {
                for (int j = -C_IF; j <= C_IF; j++) {
                    Eigen::Vector3i sub = c_sub;
                    sub.x() += i, sub.y() += j, sub.z() = 0;
                    if (free_terrain_grid_->InRange(sub)) {
                        const int ind = free_terrain_grid_->Sub2Ind(sub);
                        free_terrain_grid_->GetCell(ind) = free_terrain_grid_->GetCell(ind) | OBS_BIT;
                    }
                }
            }
        }
        for (const auto& point : freeCloudIn->points) { // Set free cloud in free terrain grid
            Eigen::Vector3i c_sub = free_terrain_grid_->Pos2Sub(point.x, point.y, grid_center_.z);
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    Eigen::Vector3i sub = c_sub;
                    sub.x() += i, sub.y() += j, sub.z() = 0;
                    if (free_terrain_grid_->InRange(sub)) {
                        const int ind = free_terrain_grid_->Sub2Ind(sub);
                        free_terrain_grid_->GetCell(ind) = free_terrain_grid_->GetCell(ind) | FREE_BIT;
                        is_goal_in_freespace_ = true;
                    }
                }
            }
        }
    }
    if (!is_goal_in_freespace_) { // check for freespace status if no terrain clouds around
        float min_dist = FARUtil::kINF;
        for (const auto& node_ptr : current_graph_) {
            if (!node_ptr->is_navpoint) continue;
            const float cur_dist = (node_ptr->position - center).norm_flat();
            if (cur_dist < FARUtil::kLocalPlanRange && FARUtil::IsAtSameLayer(node_ptr, goal_node_ptr_)) {
                if (cur_dist < min_dist) {
                    goal_node_ptr_->position.z = node_ptr->position.z;
                    min_dist = cur_dist;
                }
                is_goal_in_freespace_ = true;
            }
        }
    }
}
