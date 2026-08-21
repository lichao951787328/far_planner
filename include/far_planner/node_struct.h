#ifndef NODE_STRUCT_H
#define NODE_STRUCT_H

#include "point_struct.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

enum NodeType {
    NOT_DEFINED = 0,
    GROUND      = 1,
    AIR         = 2
};

enum NodeFreeDirect {
  UNKNOW  =  0,
  CONVEX  =  1,
  CONCAVE =  2,
  PILLAR  =  3
};

// Source and lifetime of a visibility-graph node.  Obstacle vertices must keep
// their semantic origin; otherwise a dynamic-object corner is indistinguishable
// from a persistent wall corner once it has entered the graph.
enum class GraphNodeSource {
    UNKNOWN = 0,
    ODOM,
    GOAL,
    STATIC_CANDIDATE,
    STATIC_GLOBAL,
    DYNAMIC_LOCAL,
    PATH_HISTORY
};

enum class StaticNodeEvidence {
    UNKNOWN = 0,
    STATIC_OCCUPIED,
    EXPLICIT_FREE
};

enum class GraphEdgeSource {
    UNKNOWN = 0,
    STATIC_VISIBILITY,
    STATIC_CONTOUR,
    DYNAMIC_LOCAL,
    STITCH,
    ODOM_CONNECT,
    GOAL_CONNECT
};

enum class EdgeValidationMode {
    VISIBILITY = 0,
    CONTOUR_FOLLOW
};

// A persistent obstacle and the navigation topology inferred from its contour
// have different lifetimes.  In particular, an old wall voxel can remain
// occupied after a cropped, false wall-end relation has been disproved.
enum class ContourTopologyObservation {
    UNOBSERVED = 0,
    CONFIRMED,
    CONTRADICTED
};

enum class EdgeRejectReason {
    NONE = 0,
    NOT_CURRENT_ADJACENT,
    UNREACHABLE,
    DIRECTION_REJECTED,
    DIRECTION_SPARSIFIED,
    STATIC_CLOUD_BLOCKED,
    DYNAMIC_CLOUD_BLOCKED,
    POLYGON_BLOCKED,
    TERRAIN_BLOCKED,
    OFFSET_FAILED,
    SELF_POLYGON_BLOCKED,
    OTHER_STATIC_BLOCKED,
    CLIPPED_CONTOUR,
    VOTE_PENDING
};

struct GraphEdgeState {
    GraphEdgeSource source = GraphEdgeSource::UNKNOWN;
    EdgeValidationMode validation_mode = EdgeValidationMode::VISIBILITY;
    bool static_valid = true;
    bool dynamic_blocked = false;
    // Reserved for a committed topology block. A single local contour miss
    // must not set this flag: doing so cuts the usable graph before the
    // replacement contour topology has been built and validated.
    bool topology_blocked = false;
    bool active = true;
    bool has_clearance_geometry = false;
    Point3D route_start;
    Point3D route_end;
    float route_cost = 0.0f;
    int current_contour_misses = 0;
    // Consecutive physical failures of an ordinary static visibility edge.
    // The edge is masked on the first failure, but its identity is retained
    // until this counter reaches the configured removal threshold.
    int static_visibility_misses = 0;

    bool IsActive() const {
        return static_valid && !dynamic_blocked && !topology_blocked && active;
    }
};

inline bool ApplyContourTopologyObservation(
    GraphEdgeState& state, const ContourTopologyObservation observation,
    const int remove_after_misses) {
    switch (observation) {
        case ContourTopologyObservation::CONFIRMED:
            state.topology_blocked = false;
            state.current_contour_misses = 0;
            return false;
        case ContourTopologyObservation::CONTRADICTED:
            // Accumulate evidence FAR-style, but keep the old relation usable
            // until the graph owner atomically commits a safe replacement.
            // Physical collision validation remains authoritative and can
            // still set static_valid=false immediately.
            state.topology_blocked = false;
            state.current_contour_misses = std::min(
                std::max(1, remove_after_misses),
                state.current_contour_misses + 1);
            return state.current_contour_misses >=
                   std::max(1, remove_after_misses);
        case ContourTopologyObservation::UNOBSERVED:
            return false;
    }
    return false;
}

/** A contour identity and its physical validity have different lifetimes.
 * Failed geometry is blocked immediately for safety, but the accumulated
 * contour relation is retained so one noisy snapshot cannot erase topology. */
inline void ApplyContourStaticValidationObservation(
    GraphEdgeState& state, const bool route_is_statically_valid) {
    state.static_valid = route_is_statically_valid;
    state.active = true;
}

/** Ordinary visibility edges use a two-stage failure policy: one bad static
 * observation blocks search immediately, while only consecutive bad
 * observations authorize deleting the accumulated edge identity. */
inline bool ApplyVisibilityStaticValidationObservation(
    GraphEdgeState& state, const bool route_is_statically_valid,
    const int remove_after_misses) {
    state.active = true;
    state.static_valid = route_is_statically_valid;
    if (route_is_statically_valid) {
        state.static_visibility_misses = 0;
        return false;
    }
    state.static_visibility_misses = std::min(
        std::max(1, remove_after_misses),
        state.static_visibility_misses + 1);
    return state.static_visibility_misses >=
           std::max(1, remove_after_misses);
}

inline bool IsStaticGeometryRejectReason(const EdgeRejectReason reason) {
    return reason == EdgeRejectReason::STATIC_CLOUD_BLOCKED ||
           reason == EdgeRejectReason::POLYGON_BLOCKED ||
           reason == EdgeRejectReason::SELF_POLYGON_BLOCKED ||
           reason == EdgeRejectReason::OTHER_STATIC_BLOCKED ||
           reason == EdgeRejectReason::CLIPPED_CONTOUR ||
           reason == EdgeRejectReason::TERRAIN_BLOCKED;
}

/** These outcomes are topology-selection/vote state, not a current physical
 * obstacle. They may prevent creating a brand-new edge, but must not erase an
 * already validated static visibility edge. */
inline bool IsRecoverableStaticVisibilitySelectionReason(
    const EdgeRejectReason reason) {
    return reason == EdgeRejectReason::VOTE_PENDING ||
           reason == EdgeRejectReason::DIRECTION_SPARSIFIED;
}

struct EdgeValidationResult {
    bool valid = false;
    bool dynamic_blocked = false;
    EdgeRejectReason reason = EdgeRejectReason::NONE;
    Point3D route_start;
    Point3D route_end;
    float route_cost = 0.0f;
    float projection_distance = 0.0f;
};

struct EdgeDiagnostic {
    std::size_t first_id = 0;
    std::size_t second_id = 0;
    Point3D start;
    Point3D end;
    EdgeValidationMode mode = EdgeValidationMode::VISIBILITY;
    EdgeRejectReason reason = EdgeRejectReason::NONE;
};

struct EdgeRejectionStats {
    std::size_t unreachable = 0;
    std::size_t direction_rejected = 0;
    std::size_t static_cloud_blocked = 0;
    std::size_t dynamic_cloud_blocked = 0;
    std::size_t polygon_blocked = 0;
    std::size_t terrain_blocked = 0;
    std::size_t offset_failed = 0;
    std::size_t vote_pending = 0;

    void Count(const EdgeRejectReason reason) {
        switch (reason) {
            case EdgeRejectReason::UNREACHABLE:
                ++unreachable;
                break;
            case EdgeRejectReason::DIRECTION_REJECTED:
            case EdgeRejectReason::DIRECTION_SPARSIFIED:
                ++direction_rejected;
                break;
            case EdgeRejectReason::STATIC_CLOUD_BLOCKED:
                ++static_cloud_blocked;
                break;
            case EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED:
                ++dynamic_cloud_blocked;
                break;
            case EdgeRejectReason::POLYGON_BLOCKED:
            case EdgeRejectReason::SELF_POLYGON_BLOCKED:
            case EdgeRejectReason::OTHER_STATIC_BLOCKED:
            case EdgeRejectReason::CLIPPED_CONTOUR:
                ++polygon_blocked;
                break;
            case EdgeRejectReason::TERRAIN_BLOCKED:
                ++terrain_blocked;
                break;
            case EdgeRejectReason::OFFSET_FAILED:
            case EdgeRejectReason::NOT_CURRENT_ADJACENT:
                ++offset_failed;
                break;
            case EdgeRejectReason::VOTE_PENDING:
                ++vote_pending;
                break;
            case EdgeRejectReason::NONE:
                break;
        }
    }
};

typedef std::pair<Point3D, Point3D> PointPair;

namespace LiDARMODEL {
    /* array resolution: 1 degree */
    static const int kHorizontalFOV = 360;  
    static const int kVerticalFOV = 31; 
    static const float kAngleResX = 0.2;
    static const float kAngleResY = 2.0;    
} // NODEPARAMS

struct Polygon
{
  Polygon() = default;
  std::size_t N;
  std::vector<Point3D> vertices;
  bool is_robot_inside;
  bool is_pillar;
  bool is_boundary_clipped = false;
  float perimeter;
  GraphNodeSource source = GraphNodeSource::UNKNOWN;
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;

struct CTNode
{
    CTNode() = default;
    Point3D position;
    bool is_global_match;
    bool is_contour_necessary;
    bool is_ground_associate;
    // True when the vertex lies in the guard band of the local contour
    // raster.  Such a vertex may be part of an occupied obstacle display, but
    // cannot prove a physical wall end or a persistent contour adjacency.
    bool is_boundary_clipped = false;
    std::size_t nav_node_id;
    NodeFreeDirect free_direct;
    GraphNodeSource source = GraphNodeSource::UNKNOWN;

    PointPair surf_dirs;
    PolygonPtr poly_ptr;
    std::shared_ptr<CTNode> front;
    std::shared_ptr<CTNode> back;

    std::vector<std::shared_ptr<CTNode>> connect_nodes;
};

typedef std::shared_ptr<CTNode> CTNodePtr;
typedef std::vector<CTNodePtr> CTNodeStack;

struct NavNode
{
    NavNode() = default;
    std::size_t id;
    Point3D position;
    PointPair surf_dirs;
    std::deque<Point3D> pos_filter_vec;
    std::deque<PointPair> surf_dirs_vec;
    CTNodePtr ctnode;
    bool is_active;
    bool is_block_frontier;
    bool is_contour_match;
    bool is_odom;
    bool is_goal;
    bool is_near_nodes;
    bool is_wide_near;
    bool is_merged;
    bool is_covered;
    bool is_frontier;
    bool is_finalized;
    bool is_navpoint;
    bool is_boundary;
    int  clear_dumper_count;
    std::deque<int> frontier_votes;
    std::unordered_set<std::size_t> invalid_boundary;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;
    std::vector<std::shared_ptr<NavNode>> poly_connects;
    std::vector<std::shared_ptr<NavNode>> contour_connects;
    std::unordered_map<std::size_t, std::deque<int>> contour_votes;
    std::unordered_map<std::size_t, std::deque<int>> edge_votes;
    std::unordered_map<std::size_t, GraphEdgeState> edge_states;
    std::vector<std::shared_ptr<NavNode>> potential_contours;
    std::vector<std::shared_ptr<NavNode>> potential_edges;
    std::vector<std::shared_ptr<NavNode>> trajectory_connects;
    std::unordered_map<std::size_t, std::size_t> trajectory_votes;
    std::unordered_map<std::size_t, std::size_t> terrain_votes;
    NodeType node_type; 
    NodeFreeDirect free_direct;
    GraphNodeSource source = GraphNodeSource::UNKNOWN;
    // Updated exactly once per accepted semantic-map snapshot.  Candidates
    // become persistent after three positive observations; confirmed static
    // nodes are deleted only after three misses while inside the update zone.
    int static_seen_count = 0;
    int static_missed_count = 0;
    bool observed_in_semantic_snapshot = false;
    // A vertex created where the finite local contour raster cuts an
    // obstacle. It is useful as a FAR-style temporary wall end for exploring
    // toward unknown space, but it must never become persistent global map
    // structure and must disappear with the next snapshot that omits it.
    bool is_transient_contour_endpoint = false;
    // Physical static occupancy may persist after a previously inferred wall
    // end is shown by a newer contour to be ordinary wall surface.  A node is
    // not made unsearchable merely because its single-frame contour identity
    // disappeared: its individually validated incident edges remain safe to
    // use until a stable replacement topology can be committed atomically.
    bool topology_blocked = false;
    int topology_missed_count = 0;
    // planner members
    bool is_block_to_goal;
    bool is_traversable;
    bool is_free_traversable;
    float gscore, fgscore;
    std::shared_ptr<NavNode> parent;
    std::shared_ptr<NavNode> free_parent;
    
};

/** Navigation-corner lifetime is independent of physical occupancy. A
 * current straight wall may contradict an old wall-end node while the wall
 * voxel itself correctly remains occupied. */
inline bool ApplyContourNodeTopologyObservation(
    NavNode& node, const ContourTopologyObservation observation,
    const int remove_after_misses) {
    switch (observation) {
        case ContourTopologyObservation::CONFIRMED:
            node.topology_blocked = false;
            // FAR-style recoverable damper: one good observation cancels one
            // bad observation instead of rewriting all temporal evidence.
            node.topology_missed_count = std::max(
                0, node.topology_missed_count - 1);
            return false;
        case ContourTopologyObservation::CONTRADICTED:
            node.topology_missed_count = std::min(
                std::max(1, remove_after_misses),
                node.topology_missed_count + 1);
            // Unlike an obsolete contour edge, an obsolete vertex must not be
            // blocked before its replacement graph is ready.  Blocking a cut
            // vertex is equivalent to deleting it and can split the current
            // reachable graph.  The caller performs replacement/connectivity
            // checks after all new nodes and edges have been built, then
            // removes the old node in one commit.
            node.topology_blocked = false;
            return node.topology_missed_count >=
                   std::max(1, remove_after_misses);
        case ContourTopologyObservation::UNOBSERVED:
            return false;
    }
    return false;
}

/** Odom and goal are transient query endpoints, not obstacle geometry. */
inline bool IsGraphQueryEndpoint(const NavNode& node) {
    return node.is_odom || node.is_goal ||
           node.source == GraphNodeSource::ODOM ||
           node.source == GraphNodeSource::GOAL;
}

/** Only obstacle corners whose free-space side can be used by visibility
 * search may enter persistent static topology. Ordinary concave contour
 * samples remain obstacle evidence, not routing vertices. */
inline bool IsPersistentStaticRoutingVertex(const NavNode& node) {
    return !node.is_transient_contour_endpoint &&
           (node.is_boundary || node.free_direct == NodeFreeDirect::CONVEX ||
            node.free_direct == NodeFreeDirect::PILLAR);
}

/** Unknown semantic terrain is traversable for every graph edge by policy. */
inline bool RequiresKnownTerrainForGraphConnection(
    const NavNode& first, const NavNode& second) {
    (void)first;
    (void)second;
    return false;
}

enum class GraphLifecycleAction {
    KEEP = 0,
    PROMOTE_STATIC,
    REMOVE
};

inline bool IsStaticPromotionEvidenceReady(
    const StaticNodeEvidence evidence,
    const bool require_global_evidence) {
    return !require_global_evidence ||
           evidence == StaticNodeEvidence::STATIC_OCCUPIED;
}

inline bool IsGraphNodeSearchEligible(const NavNode& node) {
    if (node.is_merged || node.is_navpoint || node.topology_blocked ||
        node.source == GraphNodeSource::PATH_HISTORY) {
        return false;
    }
    // An unconfirmed static corner belongs only to the current local overlay.
    // It is searchable while present in this semantic snapshot, but does not
    // become persistent global structure until lifecycle promotion.
    if (node.source == GraphNodeSource::STATIC_CANDIDATE) {
        return node.observed_in_semantic_snapshot;
    }
    return node.source == GraphNodeSource::ODOM ||
           node.source == GraphNodeSource::GOAL ||
           node.source == GraphNodeSource::STATIC_GLOBAL ||
           node.source == GraphNodeSource::DYNAMIC_LOCAL;
}

/** A crop-generated static contour endpoint may reuse only its own transient
 * identity. It must never overwrite a confirmed physical corner merely
 * because both happen to be close to the current raster boundary. */
inline bool IsContourEndpointLifetimeMatchCompatible(
    const bool contour_is_static, const bool contour_is_boundary_clipped,
    const NavNode& node) {
    if (!contour_is_static || !contour_is_boundary_clipped) return true;
    return node.source == GraphNodeSource::STATIC_CANDIDATE &&
           node.is_transient_contour_endpoint;
}

// A goal may connect only to obstacle-contour vertices that belong to the
// persistent static graph or the current local overlay. Exploration
// frontiers are not required: a known door-frame corner is useful even when
// it is not on the known/unknown boundary. Historical robot poses are
// rejected by IsGraphNodeSearchEligible().
inline bool IsGoalConnectionCandidate(const NavNode& node) {
    if (!IsGraphNodeSearchEligible(node) || node.is_odom || node.is_goal) {
        return false;
    }
    const bool is_obstacle_graph_node =
        node.source == GraphNodeSource::STATIC_GLOBAL ||
        node.source == GraphNodeSource::STATIC_CANDIDATE ||
        node.source == GraphNodeSource::DYNAMIC_LOCAL;
    if (!is_obstacle_graph_node) return false;

    // CONVEX and PILLAR nodes represent usable visibility-graph vertices.
    // Explicit navigation-boundary vertices are retained as well because
    // their free direction may not have been classified as a contour corner.
    return node.is_boundary || node.free_direct == NodeFreeDirect::CONVEX ||
           node.free_direct == NodeFreeDirect::PILLAR;
}

/** Candidate policy for the transient robot-start query layer.
 *
 * A confirmed static corner is global knowledge and remains a valid start
 * connection target after it leaves the moving semantic window.  In
 * contrast, unconfirmed static and dynamic vertices describe only the latest
 * local overlay and therefore require a current contour observation.  This
 * keeps the start query global without accidentally retaining stale dynamic
 * or crop-generated vertices. */
inline bool IsStartConnectionCandidate(const NavNode& node) {
    if (!IsGoalConnectionCandidate(node)) return false;
    if (node.source == GraphNodeSource::STATIC_GLOBAL) return true;
    return node.observed_in_semantic_snapshot && node.is_contour_match &&
           static_cast<bool>(node.ctnode);
}

/** Local overlay vertices are meaningful only inside their observation or
 * stitch window. Confirmed static vertices are global and must not have an
 * already validated start edge removed solely because of distance. */
inline bool ShouldPruneStartConnectionForRange(
    const NavNode& node, const float distance,
    const float dynamic_range, const float static_stitch_range) {
    if (node.source == GraphNodeSource::STATIC_GLOBAL) return false;
    const float range = node.source == GraphNodeSource::DYNAMIC_LOCAL
        ? dynamic_range : static_stitch_range;
    return distance > range;
}

inline bool IsGraphEdgeSearchEligible(const NavNode& from,
                                      const NavNode& to) {
    if (!IsGraphNodeSearchEligible(to)) return false;
    const auto from_state = from.edge_states.find(to.id);
    const auto to_state = to.edge_states.find(from.id);
    return from_state != from.edge_states.end() &&
           to_state != to.edge_states.end() && from_state->second.IsActive() &&
           to_state->second.IsActive();
}

/** A persistent corner without a usable obstacle-graph edge becomes a
 * permanent orphan in the global graph.  Odom and goal edges are deliberately
 * ignored: they are rebuilt for each query and cannot prove that the corner
 * belongs to reusable map topology. */
inline bool HasActiveSearchEligibleIncidentEdge(const NavNode& node) {
    for (const auto& neighbor : node.connect_nodes) {
        if (neighbor && !IsGraphQueryEndpoint(*neighbor) &&
            IsGraphEdgeSearchEligible(node, *neighbor)) {
            return true;
        }
    }
    return false;
}

/** Pure lifecycle policy shared by production code and regression tests. */
inline GraphLifecycleAction AdvanceStaticNodeLifecycle(
    NavNode& node, const bool observed,
    const StaticNodeEvidence evidence, const float robot_distance,
    const float update_radius, const float stitch_radius,
    const int confirm_frames, const int remove_frames,
    const bool promotion_ready = true) {
    const bool is_static = node.source == GraphNodeSource::STATIC_CANDIDATE ||
                           node.source == GraphNodeSource::STATIC_GLOBAL;
    if (!is_static) return GraphLifecycleAction::KEEP;
    if (node.is_transient_contour_endpoint) {
        if (observed) {
            node.static_seen_count = 0;
            node.static_missed_count = 0;
            return GraphLifecycleAction::KEEP;
        }
        return GraphLifecycleAction::REMOVE;
    }
    if (observed) {
        node.static_missed_count = 0;
        node.static_seen_count = std::min(
            std::max(1, confirm_frames), node.static_seen_count + 1);
        if (node.source == GraphNodeSource::STATIC_CANDIDATE &&
            node.static_seen_count >= std::max(1, confirm_frames) &&
            promotion_ready) {
            node.source = GraphNodeSource::STATIC_GLOBAL;
            return GraphLifecycleAction::PROMOTE_STATIC;
        }
        return GraphLifecycleAction::KEEP;
    }
    if (robot_distance <= update_radius) {
        if (evidence == StaticNodeEvidence::EXPLICIT_FREE) {
            ++node.static_missed_count;
            if (node.static_missed_count >= std::max(1, remove_frames)) {
                return GraphLifecycleAction::REMOVE;
            }
        } else {
            // Occlusion, unknown space, contour simplification and a still
            // occupied semantic voxel are not deletion evidence.
            node.static_missed_count = 0;
        }
    } else if (node.source == GraphNodeSource::STATIC_CANDIDATE &&
               robot_distance > stitch_radius) {
        return GraphLifecycleAction::REMOVE;
    }
    return GraphLifecycleAction::KEEP;
}

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::pair<NavNodePtr, NavNodePtr> NavEdge;

/** Collect the active component reachable from a graph endpoint. Query edges
 * may be excluded when the caller needs reusable obstacle topology rather
 * than a connection that exists only for the current robot/goal query. */
inline std::unordered_set<std::size_t> ActiveReachableNodeIds(
    const NavNodePtr& start, const bool reusable_edges_only) {
    std::unordered_set<std::size_t> visited;
    if (!start || !IsGraphNodeSearchEligible(*start)) return visited;
    std::vector<NavNodePtr> pending{start};
    visited.insert(start->id);
    while (!pending.empty()) {
        const NavNodePtr current = pending.back();
        pending.pop_back();
        for (const auto& neighbor : current->connect_nodes) {
            if (!neighbor || visited.count(neighbor->id) ||
                (reusable_edges_only &&
                 (IsGraphQueryEndpoint(*current) ||
                  IsGraphQueryEndpoint(*neighbor))) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                continue;
            }
            visited.insert(neighbor->id);
            pending.push_back(neighbor);
        }
    }
    return visited;
}

/** Reachability inside an already selected graph snapshot. This second-stage
 * traversal is necessary when persistent matching history is intentionally
 * excluded: a path may not borrow a filtered node and leave its descendants
 * as an apparent island in the published search graph. */
inline std::unordered_set<std::size_t> ActiveReachableNodeIdsWithin(
    const NavNodePtr& start,
    const std::unordered_set<std::size_t>& allowed_ids) {
    std::unordered_set<std::size_t> visited;
    if (!start || !allowed_ids.count(start->id) ||
        !IsGraphNodeSearchEligible(*start)) {
        return visited;
    }
    std::vector<NavNodePtr> pending{start};
    visited.insert(start->id);
    while (!pending.empty()) {
        const NavNodePtr current = pending.back();
        pending.pop_back();
        for (const auto& neighbor : current->connect_nodes) {
            if (!neighbor || !allowed_ids.count(neighbor->id) ||
                visited.count(neighbor->id) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                continue;
            }
            visited.insert(neighbor->id);
            pending.push_back(neighbor);
        }
    }
    return visited;
}

/** Static promotion cannot depend on a dynamic object or on odom/goal query
 * edges. This traversal follows only active, reusable static routing nodes. */
inline std::unordered_set<std::size_t> ActiveStaticRoutingReachableNodeIds(
    const NavNodePtr& start) {
    std::unordered_set<std::size_t> visited;
    const auto is_static_routing = [](const NavNodePtr& node) {
        return node &&
            (node->source == GraphNodeSource::STATIC_CANDIDATE ||
             node->source == GraphNodeSource::STATIC_GLOBAL) &&
            IsPersistentStaticRoutingVertex(*node) &&
            IsGraphNodeSearchEligible(*node);
    };
    if (!is_static_routing(start)) return visited;
    std::vector<NavNodePtr> pending{start};
    visited.insert(start->id);
    while (!pending.empty()) {
        const NavNodePtr current = pending.back();
        pending.pop_back();
        for (const auto& neighbor : current->connect_nodes) {
            if (!is_static_routing(neighbor) ||
                visited.count(neighbor->id) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                continue;
            }
            visited.insert(neighbor->id);
            pending.push_back(neighbor);
        }
    }
    return visited;
}

/** Build the static component that can be committed atomically this frame.
 * Confirmed globals are always allowed, while a candidate may be traversed
 * only when it independently passed every non-topological promotion gate.
 * This prevents a mature candidate from borrowing an unready candidate as a
 * temporary bridge to the confirmed graph. */
inline std::unordered_set<std::size_t>
ActiveTransactionalStaticRoutingNodeIds(
    const std::vector<NavNodePtr>& starts,
    const std::unordered_set<std::size_t>& eligible_candidate_ids) {
    std::unordered_set<std::size_t> visited;
    const auto is_allowed = [&eligible_candidate_ids](
        const NavNodePtr& node) {
        if (!node || !IsPersistentStaticRoutingVertex(*node) ||
            !IsGraphNodeSearchEligible(*node)) {
            return false;
        }
        if (node->source == GraphNodeSource::STATIC_GLOBAL) return true;
        return node->source == GraphNodeSource::STATIC_CANDIDATE &&
               eligible_candidate_ids.count(node->id) > 0;
    };
    std::vector<NavNodePtr> pending;
    for (const auto& start : starts) {
        if (is_allowed(start) && visited.insert(start->id).second) {
            pending.push_back(start);
        }
    }
    while (!pending.empty()) {
        const NavNodePtr current = pending.back();
        pending.pop_back();
        for (const auto& neighbor : current->connect_nodes) {
            if (!is_allowed(neighbor) || visited.count(neighbor->id) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                continue;
            }
            visited.insert(neighbor->id);
            pending.push_back(neighbor);
        }
    }
    return visited;
}

/** Grow one atomic promotion transaction from the persisted static main
 * component.  Once a confirmed graph exists, map growth is anchored by its
 * reusable topology rather than by a transient odom visibility edge. */
inline std::unordered_set<std::size_t>
PersistentMainTransactionNodeIds(
    const std::vector<NavNodePtr>& confirmed_nodes,
    const std::unordered_set<std::size_t>& persistent_main_ids,
    const std::unordered_set<std::size_t>& eligible_candidate_ids) {
    std::vector<NavNodePtr> main_seeds;
    main_seeds.reserve(persistent_main_ids.size());
    for (const auto& node : confirmed_nodes) {
        if (node && node->source == GraphNodeSource::STATIC_GLOBAL &&
            persistent_main_ids.count(node->id)) {
            main_seeds.push_back(node);
        }
    }
    return ActiveTransactionalStaticRoutingNodeIds(
        main_seeds, eligible_candidate_ids);
}

inline bool HasActiveStaticRoutingPathToAny(
    const NavNodePtr& start,
    const std::unordered_set<std::size_t>& target_ids) {
    if (!start || target_ids.empty()) return false;
    const std::unordered_set<std::size_t> reachable =
        ActiveStaticRoutingReachableNodeIds(start);
    for (const std::size_t id : target_ids) {
        if (reachable.count(id)) return true;
    }
    return false;
}

inline bool HasActiveReusablePathToAny(
    const NavNodePtr& start,
    const std::unordered_set<std::size_t>& target_ids) {
    if (!start || target_ids.empty()) return false;
    const std::unordered_set<std::size_t> reachable =
        ActiveReachableNodeIds(start, true);
    for (const std::size_t id : target_ids) {
        if (reachable.count(id)) return true;
    }
    return false;
}

inline bool ShouldCommitStaticCornerReplacement(
    const bool contradiction_mature, const bool replacement_topology_stable,
    const bool removal_preserves_connectivity) {
    return contradiction_mature && replacement_topology_stable &&
           removal_preserves_connectivity;
}

/** A replacement contour relation is usable only when both current static
 * endpoints have passed FAR's position/direction stabilization and the edge
 * stores the exact collision-validated geometry used by search/waypoint. */
inline bool IsStableValidatedContourReplacement(
    const NavNodePtr& first, const NavNodePtr& second,
    const NavNodePtr& obsolete, const PolygonPtr& current_polygon) {
    if (!first || !second || !obsolete || !current_polygon ||
        first == second || first == obsolete || second == obsolete) {
        return false;
    }
    const auto is_stable_current_static = [&current_polygon](
        const NavNodePtr& node) {
        return node && node->source == GraphNodeSource::STATIC_GLOBAL &&
               node->observed_in_semantic_snapshot && node->is_contour_match &&
               node->ctnode && node->ctnode->poly_ptr == current_polygon &&
               node->is_finalized && IsGraphNodeSearchEligible(*node);
    };
    if (!is_stable_current_static(first) ||
        !is_stable_current_static(second)) {
        return false;
    }
    if (std::find(first->contour_connects.begin(),
                  first->contour_connects.end(), second) ==
        first->contour_connects.end()) {
        return false;
    }
    const auto forward = first->edge_states.find(second->id);
    const auto reverse = second->edge_states.find(first->id);
    return forward != first->edge_states.end() &&
           reverse != second->edge_states.end() &&
           forward->second.validation_mode ==
               EdgeValidationMode::CONTOUR_FOLLOW &&
           reverse->second.validation_mode ==
               EdgeValidationMode::CONTOUR_FOLLOW &&
           forward->second.has_clearance_geometry &&
           reverse->second.has_clearance_geometry &&
           forward->second.IsActive() && reverse->second.IsActive();
}

/** Removing a vertex is safe only if all currently active static neighbours
 * remain mutually reachable without it. This protects articulation vertices
 * while allowing obsolete leaves and already-bypassed corners to disappear. */
inline bool RemovalPreservesActiveStaticConnectivity(
    const NavNodePtr& obsolete,
    const std::vector<NavNodePtr>& static_graph) {
    if (!obsolete) return false;
    const auto is_static = [](const NavNodePtr& node) {
        return node &&
            (node->source == GraphNodeSource::STATIC_CANDIDATE ||
             node->source == GraphNodeSource::STATIC_GLOBAL);
    };
    std::vector<NavNodePtr> active_neighbors;
    for (const auto& neighbor : obsolete->connect_nodes) {
        if (!is_static(neighbor) ||
            !IsGraphEdgeSearchEligible(*obsolete, *neighbor)) {
            continue;
        }
        active_neighbors.push_back(neighbor);
    }
    if (active_neighbors.size() <= 1) return true;

    std::unordered_set<std::size_t> allowed_ids;
    for (const auto& node : static_graph) {
        if (is_static(node) && node != obsolete &&
            IsGraphNodeSearchEligible(*node)) {
            allowed_ids.insert(node->id);
        }
    }
    if (!allowed_ids.count(active_neighbors.front()->id)) return false;

    std::vector<NavNodePtr> stack{active_neighbors.front()};
    std::unordered_set<std::size_t> visited{
        active_neighbors.front()->id};
    while (!stack.empty()) {
        const NavNodePtr current = stack.back();
        stack.pop_back();
        for (const auto& neighbor : current->connect_nodes) {
            if (!neighbor || neighbor == obsolete ||
                !allowed_ids.count(neighbor->id) ||
                visited.count(neighbor->id) ||
                !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                continue;
            }
            visited.insert(neighbor->id);
            stack.push_back(neighbor);
        }
    }
    for (const auto& neighbor : active_neighbors) {
        if (!visited.count(neighbor->id)) return false;
    }
    return true;
}

/** Compare the complete component reachable from the current robot before and
 * after excluding a proposed obsolete vertex.  No currently reachable search
 * node (persistent static, current dynamic, or goal) may be lost. */
inline bool RemovalPreservesCurrentReachability(
    const NavNodePtr& obsolete, const NavNodePtr& current_robot,
    const std::vector<NavNodePtr>& search_graph) {
    if (!obsolete || !current_robot || obsolete == current_robot ||
        !IsGraphNodeSearchEligible(*current_robot)) {
        return false;
    }
    std::unordered_map<std::size_t, NavNodePtr> allowed;
    for (const auto& node : search_graph) {
        if (node && IsGraphNodeSearchEligible(*node)) {
            allowed[node->id] = node;
        }
    }
    allowed[current_robot->id] = current_robot;

    const auto reachable = [&allowed](const NavNodePtr& root,
                                      const NavNodePtr& excluded) {
        std::unordered_set<std::size_t> visited;
        if (!root || root == excluded || !allowed.count(root->id)) {
            return visited;
        }
        std::vector<NavNodePtr> stack{root};
        visited.insert(root->id);
        while (!stack.empty()) {
            const NavNodePtr current = stack.back();
            stack.pop_back();
            for (const auto& neighbor : current->connect_nodes) {
                if (!neighbor || neighbor == excluded ||
                    !allowed.count(neighbor->id) ||
                    visited.count(neighbor->id) ||
                    !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                    continue;
                }
                visited.insert(neighbor->id);
                stack.push_back(neighbor);
            }
        }
        return visited;
    };

    const std::unordered_set<std::size_t> before =
        reachable(current_robot, NavNodePtr());
    if (!before.count(obsolete->id)) {
        // Removing a node outside the robot's current component cannot cut
        // that component; its static-neighbour protection is handled by the
        // caller when no robot-rooted comparison is available.
        return true;
    }
    const std::unordered_set<std::size_t> after =
        reachable(current_robot, obsolete);
    for (const std::size_t id : before) {
        if (id != obsolete->id && !after.count(id)) return false;
    }
    return true;
}

/** Return true only when a static edge can be replaced without separating its
 * endpoints. The alternate route must contain at least one current, stable,
 * collision-validated contour edge; an incidental visibility shortcut is not
 * sufficient proof that replacement contour topology is ready. */
inline bool HasActiveStaticAlternatePathWithoutEdge(
    const NavNodePtr& first, const NavNodePtr& second,
    const std::vector<NavNodePtr>& static_graph) {
    if (!first || !second || first == second) return false;
    const auto is_confirmed_static = [](const NavNodePtr& node) {
        return node && node->source == GraphNodeSource::STATIC_GLOBAL &&
               IsGraphNodeSearchEligible(*node);
    };
    if (!is_confirmed_static(first) || !is_confirmed_static(second)) {
        return false;
    }

    std::unordered_set<std::size_t> allowed_ids;
    for (const auto& node : static_graph) {
        if (is_confirmed_static(node)) allowed_ids.insert(node->id);
    }
    if (!allowed_ids.count(first->id) || !allowed_ids.count(second->id)) {
        return false;
    }

    struct SearchState {
        NavNodePtr node;
        bool crossed_current_contour = false;
    };
    std::vector<SearchState> stack{{first, false}};
    std::unordered_set<std::size_t> visited_without_contour{first->id};
    std::unordered_set<std::size_t> visited_with_contour;
    while (!stack.empty()) {
        const SearchState search_state = stack.back();
        stack.pop_back();
        const NavNodePtr& current = search_state.node;
        for (const auto& neighbor : current->connect_nodes) {
            if (!neighbor || !allowed_ids.count(neighbor->id)) {
                continue;
            }
            const bool is_removed_pair =
                (current == first && neighbor == second) ||
                (current == second && neighbor == first);
            if (is_removed_pair ||
                !IsGraphEdgeSearchEligible(*current, *neighbor)) {
                continue;
            }
            const auto edge_it = current->edge_states.find(neighbor->id);
            const bool current_validated_contour =
                edge_it != current->edge_states.end() &&
                edge_it->second.validation_mode ==
                    EdgeValidationMode::CONTOUR_FOLLOW &&
                edge_it->second.has_clearance_geometry &&
                current->observed_in_semantic_snapshot &&
                neighbor->observed_in_semantic_snapshot &&
                current->is_contour_match && neighbor->is_contour_match &&
                current->is_finalized && neighbor->is_finalized;
            const bool crossed_current_contour =
                search_state.crossed_current_contour ||
                current_validated_contour;
            if (neighbor == second && crossed_current_contour) return true;
            auto& visited = crossed_current_contour
                ? visited_with_contour : visited_without_contour;
            if (!visited.insert(neighbor->id).second) continue;
            stack.push_back({neighbor, crossed_current_contour});
        }
    }
    return false;
}

/**
 * Order-independent primitive used by visibility-edge sparsification.  A
 * candidate is dominated only by a strictly closer candidate in the same
 * angular sector; equal distances use node id as a stable tie-breaker.
 */
inline bool IsCloserVisibilityCandidateInDirection(
    const NavNode& from, const NavNode& target, const NavNode& alternative,
    const float direction_cosine, const float epsilon = 1e-6f) {
    const Point3D target_diff = target.position - from.position;
    const Point3D alternative_diff = alternative.position - from.position;
    const float target_dist = target_diff.norm();
    const float alternative_dist = alternative_diff.norm();
    if (target_dist < epsilon || alternative_dist < epsilon) return false;
    if (target_diff.normalize() * alternative_diff.normalize() <=
        direction_cosine) return false;
    return alternative_dist < target_dist - epsilon ||
           (std::fabs(alternative_dist - target_dist) <= epsilon &&
            alternative.id < target.id);
}

struct nodeptr_equal
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->id == n2->id;
  }
};

struct navedge_hash
{
  std::size_t operator() (const NavEdge& nav_edge) const
  {
    return boost::hash<std::pair<std::size_t, std::size_t>>()({nav_edge.first->id, nav_edge.second->id});
  }
};

struct nodeptr_hash
{
  std::size_t operator() (const NavNodePtr& n_ptr) const
  {
    return std::hash<std::size_t>()(n_ptr->id);
  }
};

struct nodeptr_gcomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->gscore > n2->gscore;
  }
};

struct nodeptr_fgcomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->fgscore > n2->fgscore;
  }
};

struct nodeptr_icomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->position.intensity < n2->position.intensity;
  }
};

#endif
