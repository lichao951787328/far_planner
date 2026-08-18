#include <gtest/gtest.h>

#include "far_planner/node_struct.h"

namespace {

NavNode MakeNode(const GraphNodeSource source) {
    NavNode node;
    node.source = source;
    node.is_merged = false;
    node.is_navpoint = source == GraphNodeSource::PATH_HISTORY;
    node.is_odom = source == GraphNodeSource::ODOM;
    node.is_goal = source == GraphNodeSource::GOAL;
    node.is_boundary = false;
    node.is_frontier = false;
    node.is_finalized = false;
    node.observed_in_semantic_snapshot = false;
    node.free_direct = NodeFreeDirect::UNKNOW;
    return node;
}

NavNodePtr MakeNodePtr(const std::size_t id,
                       const GraphNodeSource source) {
    NavNodePtr node = std::make_shared<NavNode>(MakeNode(source));
    node->id = id;
    node->observed_in_semantic_snapshot = true;
    return node;
}

void ConnectActiveStaticEdge(const NavNodePtr& first,
                             const NavNodePtr& second) {
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    first->connect_nodes.push_back(second);
    second->connect_nodes.push_back(first);
    GraphEdgeState state;
    state.source = GraphEdgeSource::STATIC_VISIBILITY;
    state.static_valid = true;
    state.active = true;
    first->edge_states[second->id] = state;
    second->edge_states[first->id] = state;
}

PolygonPtr MakeStaticPolygon() {
    PolygonPtr polygon = std::make_shared<Polygon>();
    polygon->source = GraphNodeSource::STATIC_CANDIDATE;
    polygon->is_pillar = false;
    polygon->is_boundary_clipped = false;
    return polygon;
}

void MatchStableNodeToPolygon(const NavNodePtr& node,
                              const PolygonPtr& polygon) {
    ASSERT_TRUE(node);
    ASSERT_TRUE(polygon);
    node->source = GraphNodeSource::STATIC_GLOBAL;
    node->is_finalized = true;
    node->is_contour_match = true;
    node->observed_in_semantic_snapshot = true;
    node->ctnode = std::make_shared<CTNode>();
    node->ctnode->poly_ptr = polygon;
    node->ctnode->source = GraphNodeSource::STATIC_CANDIDATE;
}

void ConnectValidatedContourRoute(const NavNodePtr& first,
                                  const NavNodePtr& second) {
    ConnectActiveStaticEdge(first, second);
    first->contour_connects.push_back(second);
    second->contour_connects.push_back(first);
    GraphEdgeState& forward = first->edge_states[second->id];
    GraphEdgeState& reverse = second->edge_states[first->id];
    forward.validation_mode = reverse.validation_mode =
        EdgeValidationMode::CONTOUR_FOLLOW;
    forward.has_clearance_geometry = reverse.has_clearance_geometry = true;
}

TEST(GraphLifecyclePolicy, StaticCandidateNeedsThreeObservations) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(node, true,
                  StaticNodeEvidence::STATIC_OCCUPIED,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_FALSE(IsGraphNodeSearchEligible(node));
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(node, true,
                  StaticNodeEvidence::STATIC_OCCUPIED,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_FALSE(IsGraphNodeSearchEligible(node));
    EXPECT_EQ(GraphLifecycleAction::PROMOTE_STATIC,
              AdvanceStaticNodeLifecycle(node, true,
                  StaticNodeEvidence::STATIC_OCCUPIED,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_EQ(GraphNodeSource::STATIC_GLOBAL, node.source);
    EXPECT_TRUE(IsGraphNodeSearchEligible(node));
}

TEST(GraphLifecyclePolicy, StaticCandidateWaitsForPromotionReadiness) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    for (int frame = 0; frame < 3; ++frame) {
        EXPECT_EQ(GraphLifecycleAction::KEEP,
                  AdvanceStaticNodeLifecycle(
                      node, true, StaticNodeEvidence::STATIC_OCCUPIED,
                      5.0f, 12.0f, 15.0f, 3, 3, false));
    }
    EXPECT_EQ(GraphNodeSource::STATIC_CANDIDATE, node.source);
    EXPECT_EQ(3, node.static_seen_count);

    EXPECT_EQ(GraphLifecycleAction::PROMOTE_STATIC,
              AdvanceStaticNodeLifecycle(
                  node, true, StaticNodeEvidence::STATIC_OCCUPIED,
                  5.0f, 12.0f, 15.0f, 3, 3, true));
    EXPECT_EQ(GraphNodeSource::STATIC_GLOBAL, node.source);
}

TEST(GraphLifecyclePolicy, GlobalEvidenceGateRejectsUnknownOrFree) {
    EXPECT_TRUE(IsStaticPromotionEvidenceReady(
        StaticNodeEvidence::STATIC_OCCUPIED, true));
    EXPECT_FALSE(IsStaticPromotionEvidenceReady(
        StaticNodeEvidence::UNKNOWN, true));
    EXPECT_FALSE(IsStaticPromotionEvidenceReady(
        StaticNodeEvidence::EXPLICIT_FREE, true));
    EXPECT_TRUE(IsStaticPromotionEvidenceReady(
        StaticNodeEvidence::UNKNOWN, false));
}

TEST(GraphLifecyclePolicy, ActiveIncidentEdgeMustBeSearchEligible) {
    NavNodePtr candidate =
        MakeNodePtr(101, GraphNodeSource::STATIC_CANDIDATE);
    NavNodePtr neighbor =
        MakeNodePtr(102, GraphNodeSource::STATIC_GLOBAL);
    EXPECT_FALSE(HasActiveSearchEligibleIncidentEdge(*candidate));

    ConnectActiveStaticEdge(candidate, neighbor);
    EXPECT_TRUE(HasActiveSearchEligibleIncidentEdge(*candidate));

    candidate->edge_states[neighbor->id].static_valid = false;
    neighbor->edge_states[candidate->id].static_valid = false;
    EXPECT_FALSE(HasActiveSearchEligibleIncidentEdge(*candidate));
}

TEST(GraphLifecyclePolicy, QueryEdgeCannotPromoteStaticOrphan) {
    NavNodePtr candidate =
        MakeNodePtr(103, GraphNodeSource::STATIC_CANDIDATE);
    NavNodePtr odom = MakeNodePtr(104, GraphNodeSource::ODOM);
    ConnectActiveStaticEdge(candidate, odom);

    EXPECT_TRUE(IsGraphEdgeSearchEligible(*candidate, *odom));
    EXPECT_FALSE(HasActiveSearchEligibleIncidentEdge(*candidate));
}

TEST(GraphLifecyclePolicy, OrdinaryConcaveIsNotPersistentRoutingVertex) {
    NavNode concave = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    concave.free_direct = NodeFreeDirect::CONCAVE;
    EXPECT_FALSE(IsPersistentStaticRoutingVertex(concave));

    NavNode convex = concave;
    convex.free_direct = NodeFreeDirect::CONVEX;
    EXPECT_TRUE(IsPersistentStaticRoutingVertex(convex));

    NavNode pillar = concave;
    pillar.free_direct = NodeFreeDirect::PILLAR;
    EXPECT_TRUE(IsPersistentStaticRoutingVertex(pillar));

    NavNode explicit_boundary = concave;
    explicit_boundary.is_boundary = true;
    EXPECT_TRUE(IsPersistentStaticRoutingVertex(explicit_boundary));
}

TEST(GraphLifecyclePolicy, StaticMainComponentPathCannotBorrowQueryOrDynamicNode) {
    NavNodePtr candidate =
        MakeNodePtr(111, GraphNodeSource::STATIC_CANDIDATE);
    NavNodePtr confirmed =
        MakeNodePtr(112, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr odom = MakeNodePtr(113, GraphNodeSource::ODOM);
    NavNodePtr dynamic =
        MakeNodePtr(114, GraphNodeSource::DYNAMIC_LOCAL);
    candidate->free_direct = NodeFreeDirect::CONVEX;
    confirmed->free_direct = NodeFreeDirect::CONVEX;
    dynamic->free_direct = NodeFreeDirect::CONVEX;
    std::unordered_set<std::size_t> anchors{confirmed->id};

    ConnectActiveStaticEdge(candidate, odom);
    ConnectActiveStaticEdge(odom, confirmed);
    EXPECT_FALSE(HasActiveStaticRoutingPathToAny(candidate, anchors));

    ConnectActiveStaticEdge(candidate, dynamic);
    ConnectActiveStaticEdge(dynamic, confirmed);
    EXPECT_FALSE(HasActiveStaticRoutingPathToAny(candidate, anchors));

    ConnectActiveStaticEdge(candidate, confirmed);
    EXPECT_TRUE(HasActiveStaticRoutingPathToAny(candidate, anchors));
}

TEST(GraphLifecyclePolicy, PromotionTransactionCannotBorrowUnreadyCandidate) {
    NavNodePtr confirmed =
        MakeNodePtr(115, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr unready_bridge =
        MakeNodePtr(116, GraphNodeSource::STATIC_CANDIDATE);
    NavNodePtr ready_leaf =
        MakeNodePtr(117, GraphNodeSource::STATIC_CANDIDATE);
    confirmed->free_direct = NodeFreeDirect::CONVEX;
    unready_bridge->free_direct = NodeFreeDirect::CONVEX;
    ready_leaf->free_direct = NodeFreeDirect::CONVEX;
    ConnectActiveStaticEdge(confirmed, unready_bridge);
    ConnectActiveStaticEdge(unready_bridge, ready_leaf);

    const auto without_bridge = ActiveTransactionalStaticRoutingNodeIds(
        {confirmed}, {ready_leaf->id});
    EXPECT_TRUE(without_bridge.count(confirmed->id));
    EXPECT_FALSE(without_bridge.count(unready_bridge->id));
    EXPECT_FALSE(without_bridge.count(ready_leaf->id));

    const auto atomic_chain = ActiveTransactionalStaticRoutingNodeIds(
        {confirmed}, {unready_bridge->id, ready_leaf->id});
    EXPECT_TRUE(atomic_chain.count(confirmed->id));
    EXPECT_TRUE(atomic_chain.count(unready_bridge->id));
    EXPECT_TRUE(atomic_chain.count(ready_leaf->id));
}

TEST(GraphLifecyclePolicy,
     PersistentMainPromotionExpansionDoesNotRequireCurrentOdomEdge) {
    NavNodePtr main = MakeNodePtr(121, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr detached = MakeNodePtr(122, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr attached_candidate =
        MakeNodePtr(123, GraphNodeSource::STATIC_CANDIDATE);
    NavNodePtr detached_candidate =
        MakeNodePtr(124, GraphNodeSource::STATIC_CANDIDATE);
    main->free_direct = NodeFreeDirect::CONVEX;
    detached->free_direct = NodeFreeDirect::CONVEX;
    attached_candidate->free_direct = NodeFreeDirect::CONVEX;
    detached_candidate->free_direct = NodeFreeDirect::CONVEX;
    ConnectActiveStaticEdge(main, attached_candidate);
    ConnectActiveStaticEdge(detached, detached_candidate);

    const std::vector<NavNodePtr> confirmed{main, detached};
    const std::unordered_set<std::size_t> persistent_main_ids{main->id};
    const std::unordered_set<std::size_t> ready_candidates{
        attached_candidate->id, detached_candidate->id};
    const auto transaction = PersistentMainTransactionNodeIds(
        confirmed, persistent_main_ids, ready_candidates);

    EXPECT_TRUE(transaction.count(main->id));
    EXPECT_TRUE(transaction.count(attached_candidate->id));
    EXPECT_FALSE(transaction.count(detached->id));
    EXPECT_FALSE(transaction.count(detached_candidate->id));
}

TEST(GraphLifecyclePolicy, PublishedReachabilityCannotBorrowFilteredHistory) {
    NavNodePtr odom = MakeNodePtr(118, GraphNodeSource::ODOM);
    NavNodePtr detached_history =
        MakeNodePtr(119, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr local_leaf =
        MakeNodePtr(120, GraphNodeSource::STATIC_CANDIDATE);
    detached_history->free_direct = NodeFreeDirect::CONVEX;
    local_leaf->free_direct = NodeFreeDirect::CONVEX;
    ConnectActiveStaticEdge(odom, detached_history);
    ConnectActiveStaticEdge(detached_history, local_leaf);

    const auto full = ActiveReachableNodeIds(odom, false);
    ASSERT_TRUE(full.count(local_leaf->id));

    const std::unordered_set<std::size_t> published_ids{
        odom->id, local_leaf->id};
    const auto published =
        ActiveReachableNodeIdsWithin(odom, published_ids);
    EXPECT_TRUE(published.count(odom->id));
    EXPECT_FALSE(published.count(local_leaf->id));
}

TEST(GraphLifecyclePolicy, CroppedContourEndpointNeverBecomesGlobalHistory) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    node.is_transient_contour_endpoint = true;
    for (int frame = 0; frame < 5; ++frame) {
        EXPECT_EQ(GraphLifecycleAction::KEEP,
                  AdvanceStaticNodeLifecycle(
                      node, true, StaticNodeEvidence::STATIC_OCCUPIED,
                      1.0f, 20.0f, 28.5f, 3, 3));
        EXPECT_EQ(GraphNodeSource::STATIC_CANDIDATE, node.source);
    }
    EXPECT_EQ(GraphLifecycleAction::REMOVE,
              AdvanceStaticNodeLifecycle(
                  node, false, StaticNodeEvidence::STATIC_OCCUPIED,
                  1.0f, 20.0f, 28.5f, 3, 3));
}

TEST(GraphLifecyclePolicy, CroppedEndpointCannotBorrowConfirmedCornerIdentity) {
    NavNode global = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode ordinary_candidate = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    NavNode transient_candidate =
        MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    transient_candidate.is_transient_contour_endpoint = true;

    EXPECT_FALSE(IsContourEndpointLifetimeMatchCompatible(
        true, true, global));
    EXPECT_FALSE(IsContourEndpointLifetimeMatchCompatible(
        true, true, ordinary_candidate));
    EXPECT_TRUE(IsContourEndpointLifetimeMatchCompatible(
        true, true, transient_candidate));
    EXPECT_TRUE(IsContourEndpointLifetimeMatchCompatible(
        true, false, global));
    EXPECT_TRUE(IsContourEndpointLifetimeMatchCompatible(
        false, true, global));
}

TEST(GraphLifecyclePolicy, CurrentCroppedEndpointIsATerminalCandidateOnlyNow) {
    NavNode endpoint = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    endpoint.is_transient_contour_endpoint = true;
    endpoint.observed_in_semantic_snapshot = true;
    endpoint.free_direct = NodeFreeDirect::CONVEX;

    EXPECT_TRUE(IsGraphNodeSearchEligible(endpoint));
    EXPECT_TRUE(IsGoalConnectionCandidate(endpoint));

    endpoint.observed_in_semantic_snapshot = false;
    EXPECT_FALSE(IsGraphNodeSearchEligible(endpoint));
    EXPECT_FALSE(IsGoalConnectionCandidate(endpoint));
}

TEST(GraphLifecyclePolicy, StartQueryUsesConfirmedStaticOutsideCurrentSnapshot) {
    NavNode global = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    global.free_direct = NodeFreeDirect::CONVEX;
    global.observed_in_semantic_snapshot = false;
    global.is_contour_match = false;
    global.ctnode.reset();

    EXPECT_TRUE(IsStartConnectionCandidate(global));
}

TEST(GraphLifecyclePolicy, StartQueryKeepsLocalLayersSnapshotBound) {
    NavNode candidate = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    candidate.free_direct = NodeFreeDirect::CONVEX;
    candidate.observed_in_semantic_snapshot = true;
    candidate.is_contour_match = true;
    candidate.ctnode = std::make_shared<CTNode>();
    EXPECT_TRUE(IsStartConnectionCandidate(candidate));

    candidate.observed_in_semantic_snapshot = false;
    EXPECT_FALSE(IsStartConnectionCandidate(candidate));

    NavNode dynamic = MakeNode(GraphNodeSource::DYNAMIC_LOCAL);
    dynamic.free_direct = NodeFreeDirect::CONVEX;
    dynamic.observed_in_semantic_snapshot = true;
    dynamic.is_contour_match = true;
    dynamic.ctnode = std::make_shared<CTNode>();
    EXPECT_TRUE(IsStartConnectionCandidate(dynamic));

    dynamic.is_contour_match = false;
    EXPECT_FALSE(IsStartConnectionCandidate(dynamic));
}

TEST(GraphLifecyclePolicy, StartConnectionRangePruningPreservesGlobalStatic) {
    const float dynamic_range = 20.0f;
    const float static_stitch_range = 28.5f;
    NavNode global = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode candidate = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    NavNode dynamic = MakeNode(GraphNodeSource::DYNAMIC_LOCAL);

    EXPECT_FALSE(ShouldPruneStartConnectionForRange(
        global, 100.0f, dynamic_range, static_stitch_range));
    EXPECT_FALSE(ShouldPruneStartConnectionForRange(
        candidate, 28.0f, dynamic_range, static_stitch_range));
    EXPECT_TRUE(ShouldPruneStartConnectionForRange(
        candidate, 29.0f, dynamic_range, static_stitch_range));
    EXPECT_FALSE(ShouldPruneStartConnectionForRange(
        dynamic, 19.5f, dynamic_range, static_stitch_range));
    EXPECT_TRUE(ShouldPruneStartConnectionForRange(
        dynamic, 20.5f, dynamic_range, static_stitch_range));
}

TEST(GraphLifecyclePolicy, ConfirmedStaticNeedsThreeLocalMisses) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(node, false,
                  StaticNodeEvidence::EXPLICIT_FREE,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(node, false,
                  StaticNodeEvidence::EXPLICIT_FREE,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_EQ(GraphLifecycleAction::REMOVE,
              AdvanceStaticNodeLifecycle(node, false,
                  StaticNodeEvidence::EXPLICIT_FREE,
                  5.0f, 12.0f, 15.0f, 3, 3));
}

TEST(GraphLifecyclePolicy, LeavingUpdateZoneDoesNotDeleteConfirmedStatic) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    for (int frame = 0; frame < 10; ++frame) {
        EXPECT_EQ(GraphLifecycleAction::KEEP,
                  AdvanceStaticNodeLifecycle(node, false,
                                             StaticNodeEvidence::UNKNOWN, 13.0f,
                                             12.0f, 15.0f, 3, 3));
    }
    EXPECT_EQ(0, node.static_missed_count);
    EXPECT_TRUE(IsGraphNodeSearchEligible(node));
}

TEST(GraphLifecyclePolicy, UnconfirmedDetectionDoesNotBecomeGlobalHistory) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    AdvanceStaticNodeLifecycle(node, true,
                               StaticNodeEvidence::STATIC_OCCUPIED,
                               5.0f, 12.0f, 15.0f, 3, 3);
    EXPECT_EQ(GraphLifecycleAction::REMOVE,
              AdvanceStaticNodeLifecycle(node, false,
                                         StaticNodeEvidence::UNKNOWN, 15.1f,
                                         12.0f, 15.0f, 3, 3));
}

TEST(GraphLifecyclePolicy, UnknownOrOccupiedSpaceNeverDeletesStatic) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    for (int frame = 0; frame < 100; ++frame) {
        const StaticNodeEvidence evidence = frame % 2 == 0
            ? StaticNodeEvidence::UNKNOWN
            : StaticNodeEvidence::STATIC_OCCUPIED;
        EXPECT_EQ(GraphLifecycleAction::KEEP,
                  AdvanceStaticNodeLifecycle(node, false, evidence,
                                             5.0f, 12.0f, 15.0f, 3, 3));
    }
    EXPECT_EQ(0, node.static_missed_count);
}

TEST(GraphLifecyclePolicy, FreeEvidenceMustBeConsecutive) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(node, false,
                  StaticNodeEvidence::EXPLICIT_FREE,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_EQ(1, node.static_missed_count);
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(node, false,
                  StaticNodeEvidence::UNKNOWN,
                  5.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_EQ(0, node.static_missed_count);
}

TEST(GraphLifecyclePolicy, SearchGraphAcceptsOnlyCurrentPlanningSources) {
    EXPECT_TRUE(IsGraphNodeSearchEligible(MakeNode(GraphNodeSource::ODOM)));
    EXPECT_TRUE(IsGraphNodeSearchEligible(MakeNode(GraphNodeSource::GOAL)));
    EXPECT_TRUE(IsGraphNodeSearchEligible(MakeNode(GraphNodeSource::STATIC_GLOBAL)));
    EXPECT_TRUE(IsGraphNodeSearchEligible(MakeNode(GraphNodeSource::DYNAMIC_LOCAL)));
    NavNode current_static = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    EXPECT_FALSE(IsGraphNodeSearchEligible(current_static));
    current_static.observed_in_semantic_snapshot = true;
    EXPECT_TRUE(IsGraphNodeSearchEligible(current_static));
    EXPECT_FALSE(IsGraphNodeSearchEligible(MakeNode(GraphNodeSource::PATH_HISTORY)));
    EXPECT_FALSE(IsGraphNodeSearchEligible(MakeNode(GraphNodeSource::UNKNOWN)));
}

TEST(GraphLifecyclePolicy, GoalCandidatesAreContourVerticesNotHistory) {
    NavNode static_corner = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    static_corner.free_direct = NodeFreeDirect::CONVEX;
    EXPECT_TRUE(IsGoalConnectionCandidate(static_corner));

    NavNode dynamic_corner = MakeNode(GraphNodeSource::DYNAMIC_LOCAL);
    dynamic_corner.free_direct = NodeFreeDirect::PILLAR;
    EXPECT_TRUE(IsGoalConnectionCandidate(dynamic_corner));

    NavNode current_static = MakeNode(GraphNodeSource::STATIC_CANDIDATE);
    current_static.free_direct = NodeFreeDirect::CONVEX;
    EXPECT_FALSE(IsGoalConnectionCandidate(current_static));
    current_static.observed_in_semantic_snapshot = true;
    EXPECT_TRUE(IsGoalConnectionCandidate(current_static));

    NavNode known_non_frontier = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    known_non_frontier.free_direct = NodeFreeDirect::CONVEX;
    known_non_frontier.is_frontier = false;
    EXPECT_TRUE(IsGoalConnectionCandidate(known_non_frontier));

    NavNode concave = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    concave.free_direct = NodeFreeDirect::CONCAVE;
    EXPECT_FALSE(IsGoalConnectionCandidate(concave));

    NavNode history = MakeNode(GraphNodeSource::PATH_HISTORY);
    history.free_direct = NodeFreeDirect::CONVEX;
    EXPECT_FALSE(IsGoalConnectionCandidate(history));

    NavNode unclassified = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    unclassified.free_direct = NodeFreeDirect::UNKNOW;
    EXPECT_FALSE(IsGoalConnectionCandidate(unclassified));
    unclassified.is_boundary = true;
    EXPECT_TRUE(IsGoalConnectionCandidate(unclassified));
}

TEST(GraphLifecyclePolicy, DynamicBlockChangesOnlyEdgeActivity) {
    NavNode first = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode second = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    first.id = 1;
    second.id = 2;
    GraphEdgeState state;
    state.source = GraphEdgeSource::STATIC_VISIBILITY;
    state.static_valid = true;
    first.edge_states[second.id] = state;
    second.edge_states[first.id] = state;
    EXPECT_TRUE(IsGraphEdgeSearchEligible(first, second));

    first.edge_states[second.id].dynamic_blocked = true;
    second.edge_states[first.id].dynamic_blocked = true;
    EXPECT_FALSE(IsGraphEdgeSearchEligible(first, second));
    EXPECT_TRUE(first.edge_states[second.id].static_valid);

    first.edge_states[second.id].dynamic_blocked = false;
    second.edge_states[first.id].dynamic_blocked = false;
    EXPECT_TRUE(IsGraphEdgeSearchEligible(first, second));

    first.edge_states[second.id].static_valid = false;
    second.edge_states[first.id].static_valid = false;
    EXPECT_FALSE(IsGraphEdgeSearchEligible(first, second));
}

TEST(GraphLifecyclePolicy, ContourContradictionWaitsForAtomicReplacement) {
    GraphEdgeState state;
    state.validation_mode = EdgeValidationMode::CONTOUR_FOLLOW;
    state.static_valid = true;
    state.active = true;

    EXPECT_FALSE(ApplyContourTopologyObservation(
        state, ContourTopologyObservation::CONTRADICTED, 3));
    EXPECT_FALSE(state.topology_blocked);
    EXPECT_TRUE(state.IsActive());
    EXPECT_EQ(1, state.current_contour_misses);

    EXPECT_FALSE(ApplyContourTopologyObservation(
        state, ContourTopologyObservation::CONTRADICTED, 3));
    EXPECT_FALSE(ApplyContourTopologyObservation(
        state, ContourTopologyObservation::UNOBSERVED, 3));
    EXPECT_EQ(2, state.current_contour_misses);
    EXPECT_TRUE(ApplyContourTopologyObservation(
        state, ContourTopologyObservation::CONTRADICTED, 3));
}

TEST(GraphLifecyclePolicy, CurrentContourConfirmationRestoresTopology) {
    GraphEdgeState state;
    ApplyContourTopologyObservation(
        state, ContourTopologyObservation::CONTRADICTED, 3);
    ASSERT_TRUE(state.IsActive());

    EXPECT_FALSE(ApplyContourTopologyObservation(
        state, ContourTopologyObservation::CONFIRMED, 3));
    EXPECT_FALSE(state.topology_blocked);
    EXPECT_EQ(0, state.current_contour_misses);
    EXPECT_TRUE(state.IsActive());
}

TEST(GraphLifecyclePolicy, OneBadGeometryFrameBlocksButDoesNotEraseHistory) {
    GraphEdgeState state;
    state.validation_mode = EdgeValidationMode::CONTOUR_FOLLOW;
    state.has_clearance_geometry = true;
    state.current_contour_misses = 2;
    state.route_cost = 4.0f;

    ApplyContourStaticValidationObservation(state, false);
    EXPECT_FALSE(state.IsActive());
    EXPECT_TRUE(state.has_clearance_geometry);
    EXPECT_EQ(2, state.current_contour_misses);
    EXPECT_FLOAT_EQ(4.0f, state.route_cost);

    ApplyContourStaticValidationObservation(state, true);
    EXPECT_TRUE(state.IsActive());
    EXPECT_TRUE(state.has_clearance_geometry);
}

TEST(GraphLifecyclePolicy, VisibilityFailureBlocksImmediatelyAndDeletesAfterThree) {
    GraphEdgeState state;
    state.source = GraphEdgeSource::STATIC_VISIBILITY;
    state.validation_mode = EdgeValidationMode::VISIBILITY;

    EXPECT_FALSE(ApplyVisibilityStaticValidationObservation(
        state, false, 3));
    EXPECT_FALSE(state.IsActive());
    EXPECT_EQ(1, state.static_visibility_misses);
    EXPECT_FALSE(ApplyVisibilityStaticValidationObservation(
        state, false, 3));
    EXPECT_EQ(2, state.static_visibility_misses);
    EXPECT_TRUE(ApplyVisibilityStaticValidationObservation(
        state, false, 3));
    EXPECT_EQ(3, state.static_visibility_misses);
}

TEST(GraphLifecyclePolicy, VisibilityGoodFrameRestoresRetainedEdge) {
    GraphEdgeState state;
    ApplyVisibilityStaticValidationObservation(state, false, 3);
    ASSERT_FALSE(state.IsActive());
    EXPECT_FALSE(ApplyVisibilityStaticValidationObservation(
        state, true, 3));
    EXPECT_TRUE(state.IsActive());
    EXPECT_EQ(0, state.static_visibility_misses);
}

TEST(GraphLifecyclePolicy, VisibilityDebounceAppliesOnlyToStaticGeometry) {
    EXPECT_TRUE(IsStaticGeometryRejectReason(
        EdgeRejectReason::STATIC_CLOUD_BLOCKED));
    EXPECT_TRUE(IsStaticGeometryRejectReason(
        EdgeRejectReason::POLYGON_BLOCKED));
    EXPECT_TRUE(IsStaticGeometryRejectReason(
        EdgeRejectReason::TERRAIN_BLOCKED));
    EXPECT_FALSE(IsStaticGeometryRejectReason(
        EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED));
    EXPECT_FALSE(IsStaticGeometryRejectReason(
        EdgeRejectReason::DIRECTION_SPARSIFIED));
    EXPECT_FALSE(IsStaticGeometryRejectReason(
        EdgeRejectReason::VOTE_PENDING));
    EXPECT_TRUE(IsRecoverableStaticVisibilitySelectionReason(
        EdgeRejectReason::VOTE_PENDING));
    EXPECT_TRUE(IsRecoverableStaticVisibilitySelectionReason(
        EdgeRejectReason::DIRECTION_SPARSIFIED));
    EXPECT_FALSE(IsRecoverableStaticVisibilitySelectionReason(
        EdgeRejectReason::STATIC_CLOUD_BLOCKED));
}

TEST(GraphLifecyclePolicy, ContourEdgeRemovalNeedsConfirmedStaticBypass) {
    NavNodePtr first = MakeNodePtr(1, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr second = MakeNodePtr(2, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr replacement_first =
        MakeNodePtr(3, GraphNodeSource::STATIC_GLOBAL);
    NavNodePtr replacement_second =
        MakeNodePtr(4, GraphNodeSource::STATIC_GLOBAL);
    ConnectActiveStaticEdge(first, second);
    std::vector<NavNodePtr> graph{
        first, second, replacement_first, replacement_second};

    EXPECT_FALSE(HasActiveStaticAlternatePathWithoutEdge(
        first, second, graph));

    // A visibility-only bypass preserves connectivity, but is not proof that
    // this frame built replacement contour topology.
    ConnectActiveStaticEdge(first, replacement_first);
    ConnectActiveStaticEdge(replacement_first, replacement_second);
    ConnectActiveStaticEdge(replacement_second, second);
    EXPECT_FALSE(HasActiveStaticAlternatePathWithoutEdge(
        first, second, graph));

    replacement_first->is_contour_match = true;
    replacement_second->is_contour_match = true;
    replacement_first->is_finalized = true;
    replacement_second->is_finalized = true;
    GraphEdgeState& forward =
        replacement_first->edge_states[replacement_second->id];
    GraphEdgeState& reverse =
        replacement_second->edge_states[replacement_first->id];
    forward.validation_mode = reverse.validation_mode =
        EdgeValidationMode::CONTOUR_FOLLOW;
    forward.has_clearance_geometry = reverse.has_clearance_geometry = true;
    EXPECT_TRUE(HasActiveStaticAlternatePathWithoutEdge(
        first, second, graph));

    replacement_second->source = GraphNodeSource::STATIC_CANDIDATE;
    EXPECT_FALSE(HasActiveStaticAlternatePathWithoutEdge(
        first, second, graph));
}

TEST(GraphLifecyclePolicy, RetainedStaticObstacleCanBeTopologicallyUnsearchable) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    node.topology_blocked = true;
    EXPECT_FALSE(IsGraphNodeSearchEligible(node));

    // Physical occupancy/lifecycle state is independent from corner topology.
    EXPECT_EQ(GraphLifecycleAction::KEEP,
              AdvanceStaticNodeLifecycle(
                  node, false, StaticNodeEvidence::STATIC_OCCUPIED,
                  2.0f, 12.0f, 15.0f, 3, 3));
    EXPECT_EQ(GraphNodeSource::STATIC_GLOBAL, node.source);
    EXPECT_FALSE(IsGraphNodeSearchEligible(node));

    node.topology_blocked = false;
    EXPECT_TRUE(IsGraphNodeSearchEligible(node));
}

TEST(GraphLifecyclePolicy, ObsoleteCornerTopologyIsRemovedAfterThreeFrames) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    EXPECT_FALSE(ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONTRADICTED, 3));
    // A possibly obsolete vertex stays searchable until a replacement can be
    // committed; blocking it early is equivalent to deleting a cut vertex.
    EXPECT_FALSE(node.topology_blocked);
    EXPECT_TRUE(IsGraphNodeSearchEligible(node));
    EXPECT_FALSE(ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONTRADICTED, 3));
    EXPECT_TRUE(ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONTRADICTED, 3));
    EXPECT_FALSE(node.topology_blocked);
}

TEST(GraphLifecyclePolicy, CurrentCornerConfirmationRecoversOneMiss) {
    NavNode node = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONTRADICTED, 3);
    ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONTRADICTED, 3);
    EXPECT_FALSE(ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONFIRMED, 3));
    EXPECT_FALSE(node.topology_blocked);
    EXPECT_EQ(1, node.topology_missed_count);
    EXPECT_FALSE(ApplyContourNodeTopologyObservation(
        node, ContourTopologyObservation::CONFIRMED, 3));
    EXPECT_EQ(0, node.topology_missed_count);
}

TEST(GraphLifecyclePolicy, ReplacementRequiresAllThreePrecommitGuards) {
    EXPECT_FALSE(ShouldCommitStaticCornerReplacement(false, true, true));
    EXPECT_FALSE(ShouldCommitStaticCornerReplacement(true, false, true));
    EXPECT_FALSE(ShouldCommitStaticCornerReplacement(true, true, false));
    EXPECT_TRUE(ShouldCommitStaticCornerReplacement(true, true, true));
}

TEST(GraphLifecyclePolicy, ReplacementRouteRequiresStableCurrentEndpoints) {
    const PolygonPtr polygon = MakeStaticPolygon();
    const NavNodePtr obsolete =
        MakeNodePtr(1, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr first =
        MakeNodePtr(2, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr second =
        MakeNodePtr(3, GraphNodeSource::STATIC_GLOBAL);
    MatchStableNodeToPolygon(first, polygon);
    MatchStableNodeToPolygon(second, polygon);
    ConnectValidatedContourRoute(first, second);

    EXPECT_TRUE(IsStableValidatedContourReplacement(
        first, second, obsolete, polygon));
    second->is_finalized = false;
    EXPECT_FALSE(IsStableValidatedContourReplacement(
        first, second, obsolete, polygon));
    second->is_finalized = true;
    second->edge_states[first->id].dynamic_blocked = true;
    EXPECT_FALSE(IsStableValidatedContourReplacement(
        first, second, obsolete, polygon));
}

TEST(GraphLifecyclePolicy, StaticArticulationCornerIsProtected) {
    const NavNodePtr left = MakeNodePtr(1, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr obsolete =
        MakeNodePtr(2, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr right = MakeNodePtr(3, GraphNodeSource::STATIC_GLOBAL);
    ConnectActiveStaticEdge(left, obsolete);
    ConnectActiveStaticEdge(obsolete, right);

    const std::vector<NavNodePtr> graph{left, obsolete, right};
    EXPECT_FALSE(RemovalPreservesActiveStaticConnectivity(obsolete, graph));
}

TEST(GraphLifecyclePolicy, VerifiedStaticBypassAllowsAtomicReplacement) {
    const NavNodePtr left = MakeNodePtr(1, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr obsolete =
        MakeNodePtr(2, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr right = MakeNodePtr(3, GraphNodeSource::STATIC_GLOBAL);
    ConnectActiveStaticEdge(left, obsolete);
    ConnectActiveStaticEdge(obsolete, right);
    ConnectActiveStaticEdge(left, right);

    const std::vector<NavNodePtr> graph{left, obsolete, right};
    EXPECT_TRUE(RemovalPreservesActiveStaticConnectivity(obsolete, graph));
}

TEST(GraphLifecyclePolicy, DynamicallyBlockedBypassDoesNotPermitReplacement) {
    const NavNodePtr left = MakeNodePtr(1, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr obsolete =
        MakeNodePtr(2, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr right = MakeNodePtr(3, GraphNodeSource::STATIC_GLOBAL);
    ConnectActiveStaticEdge(left, obsolete);
    ConnectActiveStaticEdge(obsolete, right);
    ConnectActiveStaticEdge(left, right);
    left->edge_states[right->id].dynamic_blocked = true;
    right->edge_states[left->id].dynamic_blocked = true;

    const std::vector<NavNodePtr> graph{left, obsolete, right};
    EXPECT_FALSE(RemovalPreservesActiveStaticConnectivity(obsolete, graph));
}

TEST(GraphLifecyclePolicy, RobotReachabilityMustSurviveCornerRemoval) {
    const NavNodePtr robot = MakeNodePtr(1, GraphNodeSource::ODOM);
    const NavNodePtr obsolete =
        MakeNodePtr(2, GraphNodeSource::STATIC_GLOBAL);
    const NavNodePtr branch =
        MakeNodePtr(3, GraphNodeSource::DYNAMIC_LOCAL);
    const NavNodePtr retained =
        MakeNodePtr(4, GraphNodeSource::STATIC_GLOBAL);
    ConnectActiveStaticEdge(robot, obsolete);
    ConnectActiveStaticEdge(obsolete, branch);
    ConnectActiveStaticEdge(robot, retained);

    const std::vector<NavNodePtr> graph{
        robot, obsolete, branch, retained};
    EXPECT_FALSE(RemovalPreservesCurrentReachability(
        obsolete, robot, graph));

    ConnectActiveStaticEdge(retained, branch);
    EXPECT_TRUE(RemovalPreservesCurrentReachability(
        obsolete, robot, graph));
}

TEST(GraphConnectionPolicy, DirectionPruningIsGeometricAndStable) {
    NavNode origin = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode near = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode far = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode side = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    origin.id = 10;
    near.id = 11;
    far.id = 12;
    side.id = 13;
    origin.position = Point3D(0.0f, 0.0f, 0.0f);
    near.position = Point3D(2.0f, 0.0f, 0.0f);
    far.position = Point3D(4.0f, 0.05f, 0.0f);
    side.position = Point3D(0.0f, 2.0f, 0.0f);

    const float ten_degree_cos = std::cos(10.0f * M_PI / 180.0f);
    EXPECT_TRUE(IsCloserVisibilityCandidateInDirection(
        origin, far, near, ten_degree_cos));
    EXPECT_FALSE(IsCloserVisibilityCandidateInDirection(
        origin, near, far, ten_degree_cos));
    EXPECT_FALSE(IsCloserVisibilityCandidateInDirection(
        origin, side, near, ten_degree_cos));
    EXPECT_FALSE(IsCloserVisibilityCandidateInDirection(
        origin, near, side, ten_degree_cos));
}

TEST(GraphConnectionPolicy, EqualDistanceUsesStableNodeIdTieBreak) {
    NavNode origin = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode first = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode second = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    origin.id = 1;
    first.id = 20;
    second.id = 10;
    origin.position = Point3D(0.0f, 0.0f, 0.0f);
    first.position = Point3D(2.0f, 0.0f, 0.0f);
    second.position = Point3D(2.0f, 0.0f, 0.0f);

    EXPECT_TRUE(IsCloserVisibilityCandidateInDirection(
        origin, first, second, 0.99f));
    EXPECT_FALSE(IsCloserVisibilityCandidateInDirection(
        origin, second, first, 0.99f));
}

TEST(GraphConnectionPolicy, UnknownTerrainDoesNotRejectAnyEdgeType) {
    NavNode odom = MakeNode(GraphNodeSource::ODOM);
    NavNode goal = MakeNode(GraphNodeSource::GOAL);
    NavNode static_corner = MakeNode(GraphNodeSource::STATIC_GLOBAL);
    NavNode dynamic_corner = MakeNode(GraphNodeSource::DYNAMIC_LOCAL);

    EXPECT_TRUE(IsGraphQueryEndpoint(odom));
    EXPECT_TRUE(IsGraphQueryEndpoint(goal));
    EXPECT_FALSE(IsGraphQueryEndpoint(static_corner));
    EXPECT_FALSE(RequiresKnownTerrainForGraphConnection(
        odom, static_corner));
    EXPECT_FALSE(RequiresKnownTerrainForGraphConnection(
        dynamic_corner, goal));
    EXPECT_FALSE(RequiresKnownTerrainForGraphConnection(
        static_corner, dynamic_corner));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
