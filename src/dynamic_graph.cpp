/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/dynamic_graph.h"

#include <cmath>

/***************************************************************************************/

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    semantic_update_in_progress_ = false;
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
    id_tracker_     = 1;
    last_connect_pos_ = Point3D(0,0,0);
    /* Initialize Terrian Planner */
    tp_params_.world_frame  = FARUtil::worldFrameId;
    tp_params_.voxel_size   = FARUtil::kLeafSize;
    tp_params_.radius       = FARUtil::kNearDist * 2.0f;
    tp_params_.inflate_size = FARUtil::kObsInflate;
    terrain_planner_.Init(nh, tp_params_);
}

void DynamicGraph::UpdateRobotPosition(const Point3D& robot_pos) {
    robot_pos_ = robot_pos;
    terrain_planner_.SetLocalTerrainObsCloud(FARUtil::surround_obs_cloud_);
    if (odom_node_ptr_ == NULL) {
        this->CreateNavNodeFromPoint(robot_pos_, odom_node_ptr_, true);
        this->AddNodeToGraph(odom_node_ptr_);
        if (FARUtil::IsDebug) ROS_INFO("DG: Odom node has been initilaized.");
    } else {
        this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
    }
    FARUtil::odom_pos = odom_node_ptr_->position;
    terrain_planner_.VisualPaths();
}

// 先清空内部 new_nodes_，如果输入为空直接返回 false。
// 遍历每个 ctnode_ptr（也就是 new_ctnodes_ 里的点），走 IsAValidNewNode 筛选。
// 通过筛选后，CreateNewNavNodeFromContour 把 CTNode 转成 NavNode，并继承轮廓属性（位置、free_direct、surf_dirs）。
// 若最终 new_nodes_ 非空，返回 true；随后主流程再用 GetNewNodes 取出并交给 UpdateNavGraph。
bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
    NavNodePtr new_node_ptr = NULL;
    new_nodes_.clear();
    if (new_ctnodes.empty()) return false;
    // Historical robot poses are intentionally not graph vertices.  Only
    // obstacle-derived corners from the current semantic snapshot are created.
    for (const auto& ctnode_ptr : new_ctnodes) {
        if (!ctnode_ptr) continue;
        const float robot_distance =
            (ctnode_ptr->position - robot_pos_).norm_flat();
        // Current dynamic vertices use the full sensed local snapshot;
        // persistent static creation uses its separately configured radius.
        if (ctnode_ptr->source == GraphNodeSource::DYNAMIC_LOCAL) {
            // Dynamic contours already come from the current robot-local
            // semantic snapshot. Do not crop them again with a static-graph
            // persistence radius.
            if (robot_distance > FARUtil::kSensorRange) continue;
        } else {
            if (robot_distance > dg_params_.static_update_radius ||
                robot_distance > dg_params_.static_stitch_radius) continue;
        }
        bool is_near_new = false;
        if (this->IsAValidNewNode(ctnode_ptr, is_near_new)) {
            this->CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
            if (!is_near_new) {
                new_node_ptr->is_block_frontier = true;
            }
            new_nodes_.push_back(new_node_ptr);
        }
    }
    if (new_nodes_.empty()) return false;
    else return true;
}

void DynamicGraph::RemoveNodeFromGraph(const NavNodePtr& node_ptr) {
    if (!node_ptr || node_ptr->is_odom || node_ptr->is_goal) return;
    ClearNodeConnectInGraph(node_ptr);
    ClearContourConnectionInGraph(node_ptr);
    ClearTrajectoryConnectInGraph(node_ptr);
    RemoveNodeIdFromMap(node_ptr);
    ClearNodeFromInternalStack(node_ptr);
    out_contour_nodes_map_.erase(node_ptr);
    FARUtil::EraseNodeFromStack(node_ptr, globalGraphNodes_);
    FARUtil::EraseNodeFromStack(node_ptr, staticCandidateGraphNodes_);
    FARUtil::EraseNodeFromStack(node_ptr, dynamicLocalGraphNodes_);
}

void DynamicGraph::BeginSemanticGraphUpdate() {
    semantic_update_in_progress_ = true;
    // Keep dynamic identities only long enough to match the next accepted
    // snapshot. FinalizeDynamicGraphUpdate removes every unmatched vertex in
    // that same update, so no disappeared obstacle becomes global history.
    for (const auto& node_ptr : dynamicLocalGraphNodes_) {
        if (!node_ptr) continue;
        node_ptr->observed_in_semantic_snapshot = false;
        node_ptr->is_contour_match = false;
        node_ptr->ctnode = NULL;
    }

    NodePtrStack static_copy = globalGraphNodes_;
    static_copy.insert(static_copy.end(), staticCandidateGraphNodes_.begin(),
                       staticCandidateGraphNodes_.end());
    for (const auto& node_ptr : static_copy) {
        if (!node_ptr) continue;
        if (node_ptr->source == GraphNodeSource::PATH_HISTORY ||
            node_ptr->is_navpoint) {
            this->RemoveNodeFromGraph(node_ptr);
            continue;
        }
        if (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
            node_ptr->source == GraphNodeSource::STATIC_GLOBAL) {
            node_ptr->observed_in_semantic_snapshot = false;
            for (auto& edge_state : node_ptr->edge_states) {
                edge_state.second.dynamic_blocked = false;
            }
        }
    }
    cur_internav_ptr_ = NULL;
    last_internav_ptr_ = NULL;
    internav_near_nodes_.clear();
    surround_internav_nodes_.clear();
}

void DynamicGraph::FinalizeDynamicGraphUpdate() {
    const NodePtrStack dynamic_copy = dynamicLocalGraphNodes_;
    for (const auto& node_ptr : dynamic_copy) {
        if (!node_ptr) continue;
        if (!node_ptr->observed_in_semantic_snapshot ||
            !node_ptr->is_contour_match || !node_ptr->ctnode) {
            this->RemoveNodeFromGraph(node_ptr);
            continue;
        }

        // Smooth only the routing vertex. Collision validation continues to
        // use the exact latest semantic dynamic cloud, so this cannot hide a
        // newly occupied voxel. A bounded EMA follows a moving object while
        // suppressing voxel/approxPolyDP quantisation jitter.
        const float alpha = std::max(
            0.0f, std::min(1.0f, dg_params_.dynamic_position_alpha));
        node_ptr->position =
            node_ptr->position * (1.0f - alpha) +
            node_ptr->ctnode->position * alpha;
        node_ptr->free_direct = node_ptr->ctnode->free_direct;
        node_ptr->surf_dirs = node_ptr->ctnode->surf_dirs;
        node_ptr->is_finalized = false;
        node_ptr->pos_filter_vec.clear();
        node_ptr->pos_filter_vec.push_back(node_ptr->position);
        node_ptr->surf_dirs_vec.clear();
        node_ptr->surf_dirs_vec.push_back(node_ptr->surf_dirs);
    }
}

bool DynamicGraph::HasStableReplacementTopology(
    const NavNodePtr& obsolete,
    const PolygonPtr& current_polygon) const {
    if (!obsolete || !current_polygon) return false;
    NodePtrStack current_polygon_nodes;
    for (const auto& candidate : globalGraphNodes_) {
        if (!candidate || candidate == obsolete ||
            candidate->source != GraphNodeSource::STATIC_GLOBAL ||
            !candidate->observed_in_semantic_snapshot ||
            !candidate->is_contour_match || !candidate->ctnode ||
            candidate->ctnode->poly_ptr != current_polygon) {
            continue;
        }
        current_polygon_nodes.push_back(candidate);
    }
    for (const auto& first : current_polygon_nodes) {
        for (const auto& second : first->contour_connects) {
            if (second && first->id < second->id &&
                IsStableValidatedCo ntourReplacement(
                    first, second, obsolete, current_polygon)) {
                // The validated edge must replace the local contour section
                // containing the obsolete corner.  A valid edge elsewhere on
                // the same large polygon is not replacement evidence.
                const PointPair replacement_chord(
                    first->ctnode->position, second->ctnode->position);
                const float replacement_tolerance = std::max(
                    FARUtil::kMatchDist,
                    FARUtil::kNavClearDist + FARUtil::kLeafSize);
                if (FARUtil::DistanceToLineSeg2D(
                        obsolete->position, replacement_chord) >
                    replacement_tolerance) {
                    continue;
                }
                return true;
            }
        }
    }
    return false;
}

bool DynamicGraph::RemovalPreservesCurrentGraphConnectivity(
    const NavNodePtr& obsolete) const {
    NodePtrStack search_graph = globalGraphNodes_;
    search_graph.insert(search_graph.end(),
                        staticCandidateGraphNodes_.begin(),
                        staticCandidateGraphNodes_.end());
    search_graph.insert(search_graph.end(), dynamicLocalGraphNodes_.begin(),
                        dynamicLocalGraphNodes_.end());
    if (odom_node_ptr_) {
        return RemovalPreservesCurrentReachability(
            obsolete, odom_node_ptr_, search_graph);
    }
    return RemovalPreservesActiveStaticConnectivity(obsolete, search_graph);
}

void DynamicGraph::UpdateStaticCornerTopology() {
    if (!semantic_update_in_progress_) return;

    NodePtrStack topology_nodes = globalGraphNodes_;
    topology_nodes.insert(topology_nodes.end(),
                          staticCandidateGraphNodes_.begin(),
                          staticCandidateGraphNodes_.end());
    const int removal_frames = std::max(
        1, dg_params_.static_topology_remove_frames);
    const float observation_tolerance = std::max(
        FARUtil::kLeafSize * 2.0f,
        std::min(FARUtil::kMatchDist, FARUtil::kNavClearDist));
    const float endpoint_guard = std::max(
        FARUtil::kNavClearDist, observation_tolerance * 1.5f);
    NodePtrStack remove_nodes;
    std::size_t contradicted = 0;
    std::size_t waiting_replacement = 0;
    std::size_t articulation_protected = 0;

    for (const auto& candidate : topology_nodes) {
        if (!candidate ||
            (candidate->source != GraphNodeSource::STATIC_CANDIDATE &&
             candidate->source != GraphNodeSource::STATIC_GLOBAL) ||
            !ContourGraph::IsPointInsideReliableContourWindow(
                candidate->position)) {
            continue;
        }
        if (candidate->is_contour_match) {
            ApplyContourNodeTopologyObservation(
                *candidate, ContourTopologyObservation::CONFIRMED,
                removal_frames);
            continue;
        }

        PolygonPtr current_polygon;
        if (!ContourGraph::IsPointConfirmedOnCurrentStaticSegmentInterior(
                candidate->position, observation_tolerance, endpoint_guard,
                &current_polygon)) {
            continue;
        }
        ++contradicted;
        const bool contradiction_mature =
            ApplyContourNodeTopologyObservation(
                *candidate, ContourTopologyObservation::CONTRADICTED,
                removal_frames);
        if (!contradiction_mature) continue;

        const bool replacement_ready =
            this->HasStableReplacementTopology(candidate, current_polygon);
        if (!replacement_ready) {
            ++waiting_replacement;
            continue;
        }
        const bool connectivity_safe =
            this->RemovalPreservesCurrentGraphConnectivity(candidate);
        if (!connectivity_safe) {
            ++articulation_protected;
            continue;
        }
        if (ShouldCommitStaticCornerReplacement(
                contradiction_mature, replacement_ready,
                connectivity_safe)) {
            remove_nodes.push_back(candidate);
        }
    }

    for (const auto& obsolete : remove_nodes) {
        ROS_INFO("DG: atomically replacing obsolete static corner %zu after "
                 "%d strong contour contradictions; replacement contour "
                 "topology is stable and static connectivity is preserved.",
                 obsolete->id, obsolete->topology_missed_count);
        this->RemoveNodeFromGraph(obsolete);
    }
    ROS_INFO_THROTTLE(
        5.0,
        "DG static topology replacement: contradicted=%zu removed=%zu "
        "waiting_replacement=%zu articulation_protected=%zu",
        contradicted, remove_nodes.size(), waiting_replacement,
        articulation_protected);
}

void DynamicGraph::CommitMatureContourEdgeReplacements() {
    const int removal_frames = std::max(
        1, dg_params_.static_remove_frames);
    NodePtrStack static_graph = globalGraphNodes_;
    static_graph.insert(static_graph.end(),
                        staticCandidateGraphNodes_.begin(),
                        staticCandidateGraphNodes_.end());

    std::vector<NavEdge> remove_edges;
    std::size_t mature = 0;
    std::size_t physically_blocked = 0;
    std::size_t waiting_replacement = 0;
    for (const auto& first : static_graph) {
        if (!first || first->source != GraphNodeSource::STATIC_GLOBAL) {
            continue;
        }
        const NodePtrStack contour_copy = first->contour_connects;
        for (const auto& second : contour_copy) {
            if (!second || first->id >= second->id ||
                second->source != GraphNodeSource::STATIC_GLOBAL) {
                continue;
            }
            const auto state_it = first->edge_states.find(second->id);
            if (state_it == first->edge_states.end() ||
                state_it->second.current_contour_misses < removal_frames) {
                continue;
            }
            ++mature;
            // New occupied geometry is hard safety evidence. Once the
            // contradiction is mature, an edge whose validated route is no
            // longer free must be removed even if the graph consequently
            // reports the region unreachable.
            if (!state_it->second.static_valid) {
                ++physically_blocked;
                remove_edges.emplace_back(first, second);
                continue;
            }
            if (HasActiveStaticAlternatePathWithoutEdge(
                    first, second, static_graph)) {
                remove_edges.emplace_back(first, second);
            } else {
                ++waiting_replacement;
            }
        }
    }

    for (const auto& edge : remove_edges) {
        const NavNodePtr& first = edge.first;
        const NavNodePtr& second = edge.second;
        if (!DeleteContourConnect(first, second)) continue;
        if (!FARUtil::IsTypeInStack(second, first->poly_connects)) {
            EraseEdge(first, second);
        } else {
            AddEdge(first, second);
        }
    }
    ROS_INFO_THROTTLE(
        5.0,
        "DG contour edge replacement: mature=%zu removed=%zu "
        "physically_blocked=%zu waiting_replacement=%zu",
        mature, remove_edges.size(), physically_blocked,
        waiting_replacement);
}

void DynamicGraph::CommitSemanticGraphUpdate(
    const std::function<StaticNodeEvidence(const Point3D&)>& evidence_query) {
    NodePtrStack remove_nodes;
    NodePtrStack promote_nodes;
    std::size_t promotion_waiting_finalization = 0;
    std::size_t promotion_waiting_edge = 0;
    NodePtrStack static_nodes = globalGraphNodes_;
    static_nodes.insert(static_nodes.end(), staticCandidateGraphNodes_.begin(),
                        staticCandidateGraphNodes_.end());
    for (const auto& node_ptr : static_nodes) {
        if (!node_ptr) continue;
        const bool is_static =
            node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
            node_ptr->source == GraphNodeSource::STATIC_GLOBAL;
        if (!is_static) continue;
        const float distance = (node_ptr->position - robot_pos_).norm_flat();
        const StaticNodeEvidence evidence =
            node_ptr->observed_in_semantic_snapshot
                ? StaticNodeEvidence::STATIC_OCCUPIED
                : (evidence_query
                       ? evidence_query(node_ptr->position)
                       : StaticNodeEvidence::UNKNOWN);
        const bool finalization_ready =
            !dg_params_.static_promotion_requires_finalized ||
            node_ptr->is_finalized;
        const bool edge_ready =
            !dg_params_.static_promotion_requires_active_edge ||
            HasActiveSearchEligibleIncidentEdge(*node_ptr);
        const bool promotion_ready = finalization_ready && edge_ready;
        const GraphLifecycleAction action = AdvanceStaticNodeLifecycle(
            *node_ptr, node_ptr->observed_in_semantic_snapshot, evidence,
            distance,
            dg_params_.static_update_radius, dg_params_.static_stitch_radius,
            dg_params_.static_confirm_frames, dg_params_.static_remove_frames,
            promotion_ready);
        if (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE &&
            node_ptr->observed_in_semantic_snapshot &&
            node_ptr->static_seen_count >=
                std::max(1, dg_params_.static_confirm_frames)) {
            if (!finalization_ready) ++promotion_waiting_finalization;
            if (!edge_ready) ++promotion_waiting_edge;
        }
        if (action == GraphLifecycleAction::PROMOTE_STATIC) {
            promote_nodes.push_back(node_ptr);
            ROS_INFO_STREAM("DG: promoted semantic static node " << node_ptr->id
                            << " after " << node_ptr->static_seen_count
                            << " observations; FAR position/direction "
                            << "stabilization remains independent.");
        } else if (action == GraphLifecycleAction::REMOVE) {
            remove_nodes.push_back(node_ptr);
        }
    }
    for (const auto& node_ptr : promote_nodes) {
        FARUtil::EraseNodeFromStack(node_ptr, staticCandidateGraphNodes_);
        if (!FARUtil::IsTypeInStack(node_ptr, globalGraphNodes_)) {
            globalGraphNodes_.push_back(node_ptr);
        }
        for (const auto& neighbor : node_ptr->connect_nodes) {
            if (!neighbor) continue;
            GraphEdgeSource source = GraphEdgeSource::STITCH;
            if (FARUtil::IsTypeInStack(neighbor,
                                       node_ptr->contour_connects)) {
                source = GraphEdgeSource::STATIC_CONTOUR;
            } else if (neighbor->source == GraphNodeSource::STATIC_GLOBAL) {
                source = GraphEdgeSource::STATIC_VISIBILITY;
            }
            node_ptr->edge_states[neighbor->id].source = source;
            neighbor->edge_states[node_ptr->id].source = source;
        }
    }
    ROS_INFO_THROTTLE(
        5.0,
        "DG static promotion gate: promoted=%zu waiting_finalized=%zu waiting_active_edge=%zu",
        promote_nodes.size(), promotion_waiting_finalization,
        promotion_waiting_edge);

    // Replacement is a transaction, not an early filtering action.  At this
    // point all current contour nodes and validated route geometries exist and
    // this frame's mature candidates have entered the persistent static
    // layer.  An old corner can therefore be removed only against the exact
    // graph that will survive this semantic snapshot.
    this->UpdateStaticCornerTopology();
    this->CommitMatureContourEdgeReplacements();

    for (const auto& node_ptr : remove_nodes) this->RemoveNodeFromGraph(node_ptr);
    semantic_update_in_progress_ = false;
}

// 清理候选坏点
// 在非冻结模式下，它先遍历扩展近邻节点 extend_match_nodes_，用 ReEvaluateCorner 复检节点有效性；连续不通过的会通过 SetNodeToClear 放入 clear_node。
// 位置： src/far_planner/src/dynamic_graph.cpp
// 动态环境下复检轨迹连边
// 如果是动态环境，还会对 internav 相关的 trajectory 连边做地形可达性复检，失败就累计失效票，成功就回收失效票。
// 位置： src/far_planner/src/dynamic_graph.cpp
// 清掉已合并/待删除节点，并补近邻集合
// 调用 ClearMergedNodesInGraph 清理内部栈中的 merged 节点，再把 margin 中已匹配的节点补回 near/wide near。
// 位置： src/far_planner/src/dynamic_graph.cpp
// 先更新 odom 到周边节点的连接
// 对 wide_near_nodes_ + new_nodes 做 odom 连通检查：能连就加 poly edge + edge，不能连就删除。
// 位置： src/far_planner/src/dynamic_graph.cpp
// 把本帧新节点正式加入全局图
// 非冻结模式下，new_nodes 会被加入 globalGraphNodes_，并加入 near 集合；如果是 navpoint 还更新当前 internav；如果来自轮廓点还会回填 CT-Nav 匹配关系。
// 位置： src/far_planner/src/dynamic_graph.cpp
// 处理超范围轮廓节点的回连
// 对 out_contour_nodes_ 尝试找可匹配近邻并记录/删除 contour vote，避免老轮廓孤立。
// 位置： src/far_planner/src/dynamic_graph.cpp
// 大规模重连 near 节点
// 分三块：
// near 节点与“外部历史连边”复检
// near 节点两两之间复检
// near 节点与 out contour 的 contour 关系复检，再做 TopTwoContourConnector 稳定连接
// 位置： src/far_planner/src/dynamic_graph.cpp
// 评估覆盖与 frontier 状态
// 最后给 near 节点更新 is_covered 和 is_frontier，供后续规划决策使用。
// 位置： src/far_planner/src/dynamic_graph.cpp
void DynamicGraph::UpdateNavGraph(const NodePtrStack& new_nodes,
                                  const bool& is_freeze_vgraph,
                                  NodePtrStack& clear_node) 
{
    // clear false positive node detection
    clear_node.clear();
    contour_edge_diagnostics_.clear();
    if (!is_freeze_vgraph) {
        for (const auto& node_ptr : extend_match_nodes_) {
            if (FARUtil::IsStaticNode(node_ptr) || node_ptr == cur_internav_ptr_) continue;
            if (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
                node_ptr->source == GraphNodeSource::STATIC_GLOBAL) {
                if (node_ptr->is_contour_match) this->ReEvaluateCorner(node_ptr);
                continue;
            }
            if (node_ptr->source == GraphNodeSource::DYNAMIC_LOCAL ||
                node_ptr->source == GraphNodeSource::PATH_HISTORY ||
                node_ptr->is_navpoint) continue;
            if (!this->ReEvaluateCorner(node_ptr)) {
                if (this->SetNodeToClear(node_ptr)) {
                    clear_node.push_back(node_ptr);
                }
            } else {
                this->ReduceDumperCounter(node_ptr);
            }
        }
        // re-evaluate trajectory edge using terrain planner
        if (!FARUtil::IsStaticEnv && cur_internav_ptr_ != NULL) {
            NodePtrStack internav_check_nodes = surround_internav_nodes_;
            if (!FARUtil::IsTypeInStack(cur_internav_ptr_, internav_check_nodes)) {
                internav_check_nodes.push_back(cur_internav_ptr_);
            }
            for (const auto& sur_internav_ptr : internav_check_nodes) {
                const NodePtrStack copy_traj_connects = sur_internav_ptr->trajectory_connects;
                for (const auto& tnode_ptr : copy_traj_connects) {
                if (this->ReEvaluateConnectUsingTerrian(sur_internav_ptr, tnode_ptr)) {
                        this->RecordValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    } else {
                        this->RemoveInValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    }
                }   
            }     
        }
    }
    // clear merged nodes in stacks
    this->ClearMergedNodesInGraph();
    // add matched margin nodes into near and wide near nodes
    this->UpdateNearNodesWithMatchedMarginNodes(margin_near_nodes_, near_nav_nodes_, wide_near_nodes_);

    if (!is_freeze_vgraph) {
        // Adding new nodes to near nodes stack
        for (const auto& new_node_ptr : new_nodes) {
            this->AddNodeToGraph(new_node_ptr);
            new_node_ptr->is_near_nodes = true;
            near_nav_nodes_.push_back(new_node_ptr);
            if (new_node_ptr->is_navpoint) this->UpdateCurInterNavNode(new_node_ptr);
            if (new_node_ptr->ctnode != NULL) {
                ContourGraph::MatchCTNodeWithNavNode(new_node_ptr->ctnode, new_node_ptr);
            }
        }
        // connect outrange contour nodes
        for (const auto& out_node_ptr : out_contour_nodes_) {
            const NavNodePtr matched_node = ContourGraph::MatchOutrangeNodeWithCTNode(out_node_ptr, near_nav_nodes_);
            const auto it = out_contour_nodes_map_.find(out_node_ptr);
            if (matched_node != NULL) {
                this->RecordContourVote(out_node_ptr, matched_node);
                it->second.second.insert(matched_node);

            }
            for (const auto& reached_node_ptr : it->second.second) {
                if (reached_node_ptr != matched_node) {
                    this->DeleteContourVote(out_node_ptr, reached_node_ptr);
                }
            }
        }
        // reconnect between near nodes
        NodePtrStack outside_break_nodes;
        for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
            const NavNodePtr nav_ptr1 = near_nav_nodes_[i];
            if (nav_ptr1->is_odom) continue;
            // re-evaluate nodes which are not in near
            const NodePtrStack copy_connect_nodes = nav_ptr1->connect_nodes;
            for (const auto& cnode : copy_connect_nodes) {
                if (cnode->is_odom || cnode->is_near_nodes || FARUtil::IsOutsideGoal(cnode) || FARUtil::IsTypeInStack(cnode, nav_ptr1->contour_connects)) continue;
                if (!this->UpdateGraphEdge(nav_ptr1, cnode, false)) {
                    outside_break_nodes.push_back(cnode);
                }
            }
            for (const auto& oc_node_ptr : out_contour_nodes_) {
                if (!oc_node_ptr->is_contour_match || !nav_ptr1->is_contour_match) continue;
                if (ContourGraph::IsNavNodesConnectFromContour(nav_ptr1, oc_node_ptr)) {
                    this->RecordContourVote(nav_ptr1, oc_node_ptr);
                } else {
                    this->DeleteContourVote(nav_ptr1, oc_node_ptr);
                }
            }
        }

        // Validate the complete local visibility candidate set before changing
        // adjacency.  The old one-pass implementation called
        // IsSimilarConnectInDirection() while connect_nodes was still being
        // mutated, so the result depended on pair iteration order.
        struct ValidatedPair {
            NavNodePtr first;
            NavNodePtr second;
        };
        std::vector<ValidatedPair> valid_pairs;
        std::vector<ValidatedPair> all_pairs;
        for (std::size_t i = 0; i < near_nav_nodes_.size(); ++i) {
            const NavNodePtr first = near_nav_nodes_[i];
            if (!first || first->is_odom) continue;
            for (std::size_t j = i + 1; j < near_nav_nodes_.size(); ++j) {
                const NavNodePtr second = near_nav_nodes_[j];
                if (!second || second->is_odom || first == second) continue;
                all_pairs.push_back({first, second});
                const bool both_static =
                    (first->source == GraphNodeSource::STATIC_CANDIDATE ||
                     first->source == GraphNodeSource::STATIC_GLOBAL) &&
                    (second->source == GraphNodeSource::STATIC_CANDIDATE ||
                     second->source == GraphNodeSource::STATIC_GLOBAL);
                if (this->IsValidConnect(first, second, true,
                                         !both_static, false)) {
                    valid_pairs.push_back({first, second});
                }
            }
        }

        // Contour votes above are now complete for the whole snapshot.  Commit
        // contour identities before visibility pruning so one edge type cannot
        // accidentally suppress or erase the other.
        for (const auto& node_ptr : near_nav_nodes_) {
            if (node_ptr && !node_ptr->is_odom) {
                this->TopTwoContourConnector(node_ptr);
            }
        }

        const auto is_contour_pair = [this](const NavNodePtr& first,
                                             const NavNodePtr& second) {
            return this->IsBoundaryConnect(first, second) ||
                   ContourGraph::IsNavNodesConnectFromContour(first, second);
        };
        const auto has_shorter_in_direction =
            [this, &valid_pairs, &is_contour_pair](const NavNodePtr& from,
                                                   const NavNodePtr& to) {
                for (const auto& candidate : valid_pairs) {
                    NavNodePtr other;
                    if (candidate.first == from) other = candidate.second;
                    else if (candidate.second == from) other = candidate.first;
                    else continue;
                    if (!other || other == to ||
                        is_contour_pair(from, other)) continue;
                    if (from->is_covered && to->is_covered &&
                        !other->is_covered) continue;
                    if (IsCloserVisibilityCandidateInDirection(
                            *from, *to, *other, CONNECT_ANGLE_COS,
                            FARUtil::kEpsilon)) {
                        return true;
                    }
                }
                return false;
            };

        std::unordered_set<NavEdge, navedge_hash> selected_pairs;
        for (const auto& pair : valid_pairs) {
            const bool keep = is_contour_pair(pair.first, pair.second) ||
                (!has_shorter_in_direction(pair.first, pair.second) &&
                 !has_shorter_in_direction(pair.second, pair.first));
            if (!keep) continue;
            NavEdge edge(pair.first, pair.second);
            if (pair.first->id > pair.second->id) {
                edge = NavEdge(pair.second, pair.first);
            }
            selected_pairs.insert(edge);
        }
        for (const auto& pair : all_pairs) {
            NavEdge edge(pair.first, pair.second);
            if (pair.first->id > pair.second->id) {
                edge = NavEdge(pair.second, pair.first);
            }
            this->ApplyValidatedGraphEdge(
                pair.first, pair.second, selected_pairs.count(edge) > 0);
        }
        // update out range break nodes connects
        for (const auto& node_ptr : near_nav_nodes_) {
            for (const auto& ob_node_ptr : outside_break_nodes) {
                this->UpdateGraphEdge(node_ptr, ob_node_ptr, false);
            }
        }

        // Contour connectors can be added after the pairwise visibility pass.
        // Apply the per-snapshot dynamic mask once more so every static edge,
        // including a contour edge, has a consistent active state.
        NodePtrStack static_edge_check_nodes = globalGraphNodes_;
        static_edge_check_nodes.insert(static_edge_check_nodes.end(),
                                       staticCandidateGraphNodes_.begin(),
                                       staticCandidateGraphNodes_.end());
        for (const auto& node_ptr : static_edge_check_nodes) {
            if (!node_ptr ||
                (node_ptr->source != GraphNodeSource::STATIC_CANDIDATE &&
                 node_ptr->source != GraphNodeSource::STATIC_GLOBAL)) continue;
            for (const auto& neighbor : node_ptr->connect_nodes) {
                if (!neighbor || node_ptr->id >= neighbor->id) continue;
                if (neighbor->source != GraphNodeSource::STATIC_CANDIDATE &&
                    neighbor->source != GraphNodeSource::STATIC_GLOBAL) continue;
                const auto state_it =
                    node_ptr->edge_states.find(neighbor->id);
                const bool has_route_geometry =
                    state_it != node_ptr->edge_states.end() &&
                    state_it->second.validation_mode ==
                        EdgeValidationMode::CONTOUR_FOLLOW &&
                    state_it->second.has_clearance_geometry;
                if (has_route_geometry && semantic_update_in_progress_ &&
                    ContourGraph::DoesSegmentIntersectReliableContourWindow(
                        state_it->second.route_start,
                        state_it->second.route_end)) {
                    const bool static_route_free =
                        ContourGraph::IsRouteConnectFreeStaticLayer(
                            state_it->second.route_start,
                            state_it->second.route_end);
                    node_ptr->edge_states[neighbor->id].static_valid =
                        static_route_free;
                    neighbor->edge_states[node_ptr->id].static_valid =
                        static_route_free;
                    if (!static_route_free) {
                        contour_edge_diagnostics_.push_back({
                            node_ptr->id, neighbor->id,
                            state_it->second.route_start,
                            state_it->second.route_end,
                            EdgeValidationMode::CONTOUR_FOLLOW,
                            EdgeRejectReason::STATIC_CLOUD_BLOCKED});
                    }
                }
                const bool blocked = has_route_geometry
                    ? !ContourGraph::IsRouteConnectFreeDynamicLayer(
                          state_it->second.route_start,
                          state_it->second.route_end)
                    : !ContourGraph::IsNavNodesConnectFreeDynamicLayer(
                          node_ptr, neighbor);
                if (blocked) {
                    node_ptr->edge_states[neighbor->id].dynamic_blocked = true;
                    neighbor->edge_states[node_ptr->id].dynamic_blocked = true;
                } else {
                    node_ptr->edge_states[neighbor->id].dynamic_blocked = false;
                    neighbor->edge_states[node_ptr->id].dynamic_blocked = false;
                }
            }
        }
        // Analysisig frontier nodes
        for (const auto& node_ptr : near_nav_nodes_) {
            if (this->IsNodeFullyCovered(node_ptr)) {
                node_ptr->is_covered = true;
            } else {
                node_ptr->is_covered = false;
            }
            if (this->IsFrontierNode(node_ptr)) {
                node_ptr->is_frontier = true;
            } else {
                node_ptr->is_frontier = false;
            }
        }

        std::size_t accepted_contour_edges = 0;
        for (const auto& node_ptr : near_nav_nodes_) {
            if (!node_ptr) continue;
            for (const auto& neighbor : node_ptr->contour_connects) {
                if (!neighbor || node_ptr->id >= neighbor->id) continue;
                const auto state = node_ptr->edge_states.find(neighbor->id);
                if (state != node_ptr->edge_states.end() &&
                    state->second.validation_mode ==
                        EdgeValidationMode::CONTOUR_FOLLOW &&
                    state->second.has_clearance_geometry &&
                    state->second.IsActive()) {
                    ++accepted_contour_edges;
                }
            }
        }
        std::size_t not_adjacent = 0;
        std::size_t static_cloud = 0;
        std::size_t self_polygon = 0;
        std::size_t other_static = 0;
        std::size_t dynamic_cloud = 0;
        std::size_t terrain = 0;
        std::size_t vote = 0;
        std::size_t offset = 0;
        std::size_t clipped = 0;
        for (const auto& diagnostic : contour_edge_diagnostics_) {
            switch (diagnostic.reason) {
                case EdgeRejectReason::NOT_CURRENT_ADJACENT:
                    ++not_adjacent;
                    break;
                case EdgeRejectReason::STATIC_CLOUD_BLOCKED:
                    ++static_cloud;
                    break;
                case EdgeRejectReason::SELF_POLYGON_BLOCKED:
                    ++self_polygon;
                    break;
                case EdgeRejectReason::OTHER_STATIC_BLOCKED:
                case EdgeRejectReason::POLYGON_BLOCKED:
                    ++other_static;
                    break;
                case EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED:
                    ++dynamic_cloud;
                    break;
                case EdgeRejectReason::TERRAIN_BLOCKED:
                    ++terrain;
                    break;
                case EdgeRejectReason::VOTE_PENDING:
                    ++vote;
                    break;
                case EdgeRejectReason::OFFSET_FAILED:
                    ++offset;
                    break;
                case EdgeRejectReason::CLIPPED_CONTOUR:
                    ++clipped;
                    break;
                default:
                    break;
            }
        }
        ROS_INFO_THROTTLE(
            5.0,
            "DG contour-follow edges: active=%zu reject[not_adjacent=%zu clipped=%zu static_cloud=%zu self_polygon=%zu other_static=%zu dynamic=%zu terrain=%zu offset=%zu vote=%zu]",
            accepted_contour_edges, not_adjacent, clipped, static_cloud, self_polygon,
            other_static, dynamic_cloud, terrain, offset, vote);
    }

    // Topology is now complete, including nodes created by this snapshot.
    // Refresh the transient start-query layer last so it sees exactly the
    // graph that the following path search will use.
    this->UpdateOdomConnections();
}

void DynamicGraph::UpdateOdomConnections() {
    if (!odom_node_ptr_) return;

    // Odom belongs only to the current search snapshot. Rebuild all incident
    // edges from its current position; an edge from an older pose must never
    // survive a lightweight odom refresh.
    this->ClearNodeConnectInGraph(odom_node_ptr_);

    // Transient start-query layer. Confirmed static corners are global map
    // knowledge, so do not crop them back to the moving semantic window.
    // Current static candidates and dynamic vertices remain local-only. Every
    // candidate, near or far, still passes the complete persistent-static,
    // current-dynamic, global-contour and terrain validation below.
    NodePtrStack candidates = globalGraphNodes_;
    candidates.insert(candidates.end(), staticCandidateGraphNodes_.begin(),
                      staticCandidateGraphNodes_.end());
    candidates.insert(candidates.end(), dynamicLocalGraphNodes_.begin(),
                      dynamicLocalGraphNodes_.end());

    std::unordered_set<std::size_t> checked_candidates;
    std::size_t accepted_connections = 0;
    std::size_t validated_candidates = 0;
    std::size_t persistent_candidates = 0;
    std::size_t local_candidates = 0;
    std::size_t not_start_candidate = 0;
    std::size_t not_topology_connected = 0;
    EdgeRejectionStats rejections;
    float farthest_candidate = 0.0f;
    float farthest_connection = 0.0f;
    for (const auto& candidate : candidates) {
        if (!candidate || candidate->is_odom || candidate->is_goal ||
            !checked_candidates.insert(candidate->id).second) continue;
        if (!IsStartConnectionCandidate(*candidate)) {
            ++not_start_candidate;
            continue;
        }
        // A start edge to a corner that has no reusable graph edge produces
        // exactly the two-node dead end seen in the SSMI replay: odom and one
        // blue orphan.  Preserve that corner for future matching, but do not
        // select it as a query anchor until the map topology reconnects it.
        if (!HasActiveSearchEligibleIncidentEdge(*candidate)) {
            ++not_topology_connected;
            continue;
        }
        if (candidate->source == GraphNodeSource::STATIC_GLOBAL) {
            ++persistent_candidates;
        } else {
            ++local_candidates;
        }
        ++validated_candidates;
        const float distance =
            (candidate->position - odom_node_ptr_->position).norm_flat();
        farthest_candidate = std::max(farthest_candidate, distance);

        EdgeValidationResult validation =
            ContourGraph::ValidateVisibilityEdgeWithRoute(
                odom_node_ptr_, candidate, true);
        EdgeRejectReason reject_reason = validation.reason;
        if (validation.valid &&
            !IsOnTerrainRoute(validation.route_start,
                              validation.route_end)) {
            validation.valid = false;
            reject_reason = EdgeRejectReason::TERRAIN_BLOCKED;
        }

        if (validation.valid) {
            AddPolyEdge(odom_node_ptr_, candidate);
            AddEdge(odom_node_ptr_, candidate);
            GraphEdgeState& forward =
                odom_node_ptr_->edge_states[candidate->id];
            GraphEdgeState& reverse =
                candidate->edge_states[odom_node_ptr_->id];
            forward.source = reverse.source = GraphEdgeSource::ODOM_CONNECT;
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
            ++accepted_connections;
            farthest_connection = std::max(farthest_connection, distance);
        } else {
            rejections.Count(reject_reason);
        }
    }
    ROS_INFO_THROTTLE(
        5.0,
        "DG start connections: unique=%zu validated=%zu persistent=%zu local=%zu accepted=%zu skipped[not_start_candidate=%zu no_topology_edge=%zu] farthest_candidate=%.2fm farthest_edge=%.2fm reject[unreachable=%zu direction=%zu static_cloud=%zu dynamic_cloud=%zu polygon=%zu terrain=%zu vote=%zu]",
        checked_candidates.size(), validated_candidates,
        persistent_candidates, local_candidates, accepted_connections,
        not_start_candidate, not_topology_connected,
        farthest_candidate, farthest_connection, rejections.unreachable,
        rejections.direction_rejected, rejections.static_cloud_blocked,
        rejections.dynamic_cloud_blocked, rejections.polygon_blocked,
        rejections.terrain_blocked, rejections.vote_pending);
}

bool DynamicGraph::IsValidConnect(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2,
                                  const bool& is_check_contour,
                                  const bool& include_dynamic,
                                  const bool& apply_direction_filter)
{
    const float dist = (node_ptr1->position - node_ptr2->position).norm();
    if (dist < FARUtil::kEpsilon) return true;
    /* check contour connection from node1 to node2 */
    if (is_check_contour) {
        if (this->IsBoundaryConnect(node_ptr1, node_ptr2) || (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, true))) {
            this->RecordContourVote(node_ptr1, node_ptr2);
        } else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
            this->DeleteContourVote(node_ptr1, node_ptr2);
        }
    }
    bool is_connect = false;
    /* check polygon connections */
    const bool is_dynamic_edge =
        node_ptr1->source == GraphNodeSource::DYNAMIC_LOCAL ||
        node_ptr2->source == GraphNodeSource::DYNAMIC_LOCAL;
    const int vote_queue_size = is_dynamic_edge ? 1
        : ((node_ptr1->is_odom || node_ptr2->is_odom)
               ? std::ceil(dg_params_.votes_size / 3.0f)
               : dg_params_.votes_size);
    const bool convex = IsConvexConnect(node_ptr1, node_ptr2);
    // Match the original FAR endpoint rule: an odom-to-corner edge must lie
    // in the obstacle corner's free sector.  Odom itself is a PILLAR endpoint
    // and therefore contributes no artificial surface restriction.
    const bool direct = this->IsInDirectConstraint(node_ptr1, node_ptr2);
    const bool polygon_free =
        ContourGraph::ValidateVisibilityEdgeGeometry(
            node_ptr1, node_ptr2, include_dynamic) ==
        EdgeRejectReason::NONE;
    const bool terrain_free = convex && direct && polygon_free
        ? IsOnTerrainConnect(node_ptr1, node_ptr2, false)
        : false;
    const bool poly_matched = this->IsPolyMatchedForConnect(
        node_ptr1, node_ptr2);
    if (convex && direct && polygon_free && terrain_free) {
        if (poly_matched) {
            RecordPolygonVote(node_ptr1, node_ptr2, vote_queue_size);
        }
    } else {
        DeletePolygonVote(node_ptr1, node_ptr2, vote_queue_size);
    }
    const bool vote_ready = this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2);
    if (vote_ready) {
        if (!apply_direction_filter ||
            !this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) {
            is_connect = true;
        }
    } else if (node_ptr1->is_odom || node_ptr2->is_odom) {
        node_ptr1->edge_votes.erase(node_ptr2->id);
        node_ptr2->edge_votes.erase(node_ptr1->id);
        // clear potential connections
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
    }
    return is_connect;
}

EdgeRejectReason DynamicGraph::ClassifyVisibilityRejection(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2,
    const bool include_dynamic, const bool apply_direction_filter) {
    if (!node_ptr1 || !node_ptr2 || node_ptr1 == node_ptr2 ||
        !IsGraphNodeSearchEligible(*node_ptr1) ||
        !IsGraphNodeSearchEligible(*node_ptr2)) {
        return EdgeRejectReason::UNREACHABLE;
    }
    if (!IsConvexConnect(node_ptr1, node_ptr2) ||
        !this->IsInDirectConstraint(node_ptr1, node_ptr2)) {
        return EdgeRejectReason::DIRECTION_REJECTED;
    }
    const EdgeRejectReason geometry =
        ContourGraph::ValidateVisibilityEdgeGeometry(
            node_ptr1, node_ptr2, include_dynamic);
    if (geometry != EdgeRejectReason::NONE) return geometry;
    if (!IsOnTerrainRoute(node_ptr1->position, node_ptr2->position)) {
        return EdgeRejectReason::TERRAIN_BLOCKED;
    }
    if (!this->IsPolyMatchedForConnect(node_ptr1, node_ptr2) ||
        !this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
        return EdgeRejectReason::VOTE_PENDING;
    }
    if (apply_direction_filter &&
        this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) {
        return EdgeRejectReason::DIRECTION_REJECTED;
    }
    return EdgeRejectReason::VOTE_PENDING;
}

bool DynamicGraph::UpdateGraphEdge(const NavNodePtr& node_ptr1,
                                   const NavNodePtr& node_ptr2,
                                   const bool& is_check_contour) {
    if (!node_ptr1 || !node_ptr2 || node_ptr1 == node_ptr2) return false;
    const auto is_static_obstacle_node = [](const NavNodePtr& node_ptr) {
        return node_ptr &&
               (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
                node_ptr->source == GraphNodeSource::STATIC_GLOBAL);
    };
    const bool is_persistent_static_edge =
        is_static_obstacle_node(node_ptr1) &&
        is_static_obstacle_node(node_ptr2);

    // Static geometry owns the lifetime of a static edge.  A current dynamic
    // obstacle may make that edge inactive for this search snapshot, but must
    // not erase the edge or its accumulated static votes.
    const bool structurally_valid = this->IsValidConnect(
        node_ptr1, node_ptr2, is_check_contour,
        !is_persistent_static_edge);
    return this->ApplyValidatedGraphEdge(node_ptr1, node_ptr2,
                                         structurally_valid);
}

void DynamicGraph::RemoveVisibilityEdge(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2) {
    if (!node_ptr1 || !node_ptr2) return;
    ErasePolyEdge(node_ptr1, node_ptr2);
    const bool has_other_edge_identity =
        FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects) ||
        FARUtil::IsTypeInStack(node_ptr2, node_ptr1->trajectory_connects);
    if (!has_other_edge_identity) EraseEdge(node_ptr1, node_ptr2);
}

bool DynamicGraph::ApplyValidatedGraphEdge(const NavNodePtr& node_ptr1,
                                           const NavNodePtr& node_ptr2,
                                           const bool structurally_valid) {
    if (!node_ptr1 || !node_ptr2 || node_ptr1 == node_ptr2) return false;
    const auto is_static_obstacle_node = [](const NavNodePtr& node_ptr) {
        return node_ptr &&
               (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
                node_ptr->source == GraphNodeSource::STATIC_GLOBAL);
    };
    const bool is_persistent_static_edge =
        is_static_obstacle_node(node_ptr1) &&
        is_static_obstacle_node(node_ptr2);
    if (!structurally_valid) {
        this->RemoveVisibilityEdge(node_ptr1, node_ptr2);
        return false;
    }

    this->AddPolyEdge(node_ptr1, node_ptr2);
    this->AddEdge(node_ptr1, node_ptr2);
    if (!is_persistent_static_edge) return true;

    const auto state_it = node_ptr1->edge_states.find(node_ptr2->id);
    const bool has_route_geometry =
        state_it != node_ptr1->edge_states.end() &&
        state_it->second.validation_mode ==
            EdgeValidationMode::CONTOUR_FOLLOW &&
        state_it->second.has_clearance_geometry;
    const bool dynamic_blocked = has_route_geometry
        ? !ContourGraph::IsRouteConnectFreeDynamicLayer(
              state_it->second.route_start, state_it->second.route_end)
        : !ContourGraph::IsNavNodesConnectFreeDynamicLayer(
              node_ptr1, node_ptr2);
    if (dynamic_blocked) {
        node_ptr1->edge_states[node_ptr2->id].dynamic_blocked = true;
        node_ptr2->edge_states[node_ptr1->id].dynamic_blocked = true;
    } else {
        node_ptr1->edge_states[node_ptr2->id].dynamic_blocked = false;
        node_ptr2->edge_states[node_ptr1->id].dynamic_blocked = false;
    }
    return !dynamic_blocked;
}

bool DynamicGraph::IsOnTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour) {
        if (!node_ptr1->is_active || !node_ptr2->is_active) return true;
        Point3D mid_p = (node_ptr1->position + node_ptr2->position) / 2.0f;
        const Point3D diff_p = node_ptr2->position - node_ptr1->position;
        if (diff_p.norm() > FARUtil::kMatchDist && abs(diff_p.z) / std::hypotf(diff_p.x, diff_p.y) > 1) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false; // slope is too steep > 45 degree
        } 
        if (is_contour && node_ptr1->contour_votes.find(node_ptr2->id) != node_ptr1->contour_votes.end()) { // recorded contour terrain connection
            return true;
        }
        bool is_match;
        float minH, maxH;
        const float avg_h = MapHandler::NearestHeightOfRadius(mid_p, FARUtil::kMatchDist, minH, maxH, is_match);
        if (!is_match) {
            // In semantic-camera mode an unobserved midpoint is UNKNOWN, not
            // evidence of an obstacle or a height discontinuity. Apply the
            // same optimistic unknown-space policy to odom/goal edges and to
            // obstacle-to-obstacle stitch edges. Every edge still has to pass
            // convexity, FAR surface direction, contour intersection and the
            // current static+dynamic raw-cloud corridor checks.
            return true;
        }
        if (is_match && (maxH - minH > FARUtil::kMarginHeight || abs(minH + FARUtil::vehicle_height - mid_p.z) > FARUtil::kTolerZ / 2.0f)) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false;
        }
        if (!is_contour) {
            if (is_match) RecordVaildTerrainConnect(node_ptr1, node_ptr2);
            const auto it = node_ptr1->terrain_votes.find(node_ptr2->id);
            if (it != node_ptr1->terrain_votes.end() && it->second > dg_params_.finalize_thred) {
                return false;
            }
        }
        return true;
    }

bool DynamicGraph::IsOnTerrainRoute(const Point3D& start,
                                    const Point3D& end) {
    const Point3D diff = end - start;
    const float horizontal = std::hypot(diff.x, diff.y);
    if (diff.norm() > FARUtil::kMatchDist &&
        (horizontal < FARUtil::kEpsilon ||
         std::fabs(diff.z) / horizontal > 1.0f)) {
        return false;
    }
    const Point3D midpoint = (start + end) / 2.0f;
    bool matched = false;
    float min_height = 0.0f;
    float max_height = 0.0f;
    MapHandler::NearestHeightOfRadius(midpoint, FARUtil::kMatchDist,
                                      min_height, max_height, matched);
    if (!matched) return true;
    return max_height - min_height <= FARUtil::kMarginHeight &&
           std::fabs(min_height + FARUtil::vehicle_height - midpoint.z) <=
               FARUtil::kTolerZ / 2.0f;
}

bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
    if (FARUtil::IsFreeNavNode(node_ptr) || node_ptr->is_covered) return true;
    NodePtrStack check_odom_list = internav_near_nodes_;
    check_odom_list.push_back(odom_node_ptr_);
    for (const auto& near_optr : check_odom_list) {
        const float cur_dist = (node_ptr->position - near_optr->position).norm();
        if (cur_dist < FARUtil::kMatchDist) return true;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            // TODO: concave nodes will not be marked as covered based on current implementation
            const auto it = near_optr->edge_votes.find(node_ptr->id);
            if (it != near_optr->edge_votes.end() && FARUtil::IsVoteTrue(it->second)) {
                const Point3D diff_p = near_optr->position - node_ptr->position;
                if (FARUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicGraph::IsFrontierNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_contour_match) {
        if (node_ptr->is_block_frontier || node_ptr->is_covered || node_ptr->free_direct != NodeFreeDirect::CONVEX ||
            node_ptr->ctnode->poly_ptr->perimeter < dg_params_.frontier_perimeter_thred) 
        {
            node_ptr->frontier_votes.push_back(0); // non convex frontier or too small
        } else {
            node_ptr->frontier_votes.push_back(1); // convex frontier
        }
    } else if (!FARUtil::IsPointInMarginRange(node_ptr->position)) { // if not in margin range, the node won't be deleted
        node_ptr->frontier_votes.push_back(0); // non convex frontier
    }
    if (node_ptr->frontier_votes.size() > dg_params_.finalize_thred) {
        node_ptr->frontier_votes.pop_front();
    }
    bool is_frontier = FARUtil::IsVoteTrue(node_ptr->frontier_votes);
    if (!node_ptr->is_frontier && is_frontier && node_ptr->frontier_votes.size() == dg_params_.finalize_thred) {
        if (!FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
            is_frontier = false;
        }
    }
    return is_frontier;
}

bool DynamicGraph::IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                              const NavNodePtr& node_ptr_to)
{
    // TODO: check for connection loss
    if (node_ptr_from->is_odom || node_ptr_to->is_odom) return false;
    if (FARUtil::IsTypeInStack(node_ptr_to, node_ptr_from->contour_connects)) { // release for contour connection
        return false;
    }
    // check from to to node connection
    if (this->IsAShorterConnectInDir(node_ptr_from, node_ptr_to)) {
        return true;
    }
    if (this->IsAShorterConnectInDir(node_ptr_to, node_ptr_from)) {
        return true;
    }
    return false;
}

bool DynamicGraph::IsInDirectConstraint(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2) 
{
    // check for odom -> frontier connections
    if ((node_ptr1->is_odom && node_ptr2->is_frontier) || (node_ptr2->is_odom && node_ptr1->is_frontier)) return true;
    // check node1 -> node2
    if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        if (!FARUtil::IsOutReducedDirs(diff_1to2, node_ptr1->surf_dirs)) {
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!FARUtil::IsOutReducedDirs(diff_2to1, node_ptr2->surf_dirs)) {
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsInContourDirConstraint(const NavNodePtr& node_ptr1,
                                            const NavNodePtr& node_ptr2) 
{
    if (FARUtil::IsFreeNavNode(node_ptr1) || FARUtil::IsFreeNavNode(node_ptr2)) return false;
    // check node1 -> node2
    if (node_ptr1->is_finalized && node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_1to2 = node_ptr2->position - node_ptr1->position;
        if (!FARUtil::IsInContourDirPairs(diff_1to2, node_ptr1->surf_dirs)) {
            if (node_ptr1->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr1);
            } 
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->is_finalized && node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_2to1 = node_ptr1->position - node_ptr2->position;
        if (!FARUtil::IsInContourDirPairs(diff_2to1, node_ptr2->surf_dirs)) {
            if (node_ptr2->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr2);
            }
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
    bool is_nav_connect = false;
    bool is_cover_connect = false;
    if (node_ptr_from->is_navpoint && node_ptr_to->is_navpoint) is_nav_connect = true;
    if (node_ptr_from->is_covered && node_ptr_to->is_covered) is_cover_connect = true;
    if (node_ptr_from->connect_nodes.empty()) return false;
    Point3D ref_dir, ref_diff;
    const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
    const Point3D connect_dir = diff_p.normalize();
    const float dist = diff_p.norm();
    for (const auto& cnode : node_ptr_from->connect_nodes) {
        if (is_nav_connect && !cnode->is_navpoint) continue;
        if (is_cover_connect && !cnode->is_covered) continue;
        if (FARUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects)) continue;
        ref_diff = cnode->position - node_ptr_from->position;
        if (cnode->is_odom || ref_diff.norm() < FARUtil::kEpsilon) continue;
        ref_dir = ref_diff.normalize();
        if ((connect_dir * ref_dir) > CONNECT_ANGLE_COS && dist > ref_diff.norm()) {
            return true;
        }
    }
    return false;
}

bool DynamicGraph::UpdateNodePosition(const NavNodePtr& node_ptr,
                                      const Point3D& new_pos) 
{
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        this->InitNodePosition(node_ptr, new_pos);
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    node_ptr->pos_filter_vec.push_back(new_pos);
    if (node_ptr->pos_filter_vec.size() > dg_params_.pool_size) {
        node_ptr->pos_filter_vec.pop_front();
    }
    // calculate mean nav node position using RANSACS
    std::size_t inlier_size = 0;
    Point3D mean_p = FARUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
    if (node_ptr->pos_filter_vec.size() > 1) mean_p.z = node_ptr->position.z; // keep z value with terrain updates
    node_ptr->position = mean_p;
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;
}

void DynamicGraph::InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
    node_ptr->pos_filter_vec.clear();
    node_ptr->position = new_pos;
    node_ptr->pos_filter_vec.push_back(new_pos);
}

bool DynamicGraph::UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs)
{
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    FARUtil::CorrectDirectOrder(node_ptr->surf_dirs, cur_dirs);
    node_ptr->surf_dirs_vec.push_back(cur_dirs);
    if (node_ptr->surf_dirs_vec.size() > dg_params_.pool_size) {
        node_ptr->surf_dirs_vec.pop_front();
    }
    // calculate mean surface corner direction using RANSACS
    std::size_t inlier_size = 0;
    const PointPair mean_dir = FARUtil::RANSACSurfDirs(node_ptr->surf_dirs_vec, dg_params_.filter_dirs_margin, inlier_size);
    if (mean_dir.first == Point3D(0,0,-1) || mean_dir.second == Point3D(0,0,-1)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
    } else {
        node_ptr->surf_dirs = mean_dir;
        this->ReEvaluateConvexity(node_ptr);
    }
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;       
}

void DynamicGraph::ReEvaluateConvexity(const NavNodePtr& node_ptr) {
    if (!node_ptr->is_contour_match || node_ptr->ctnode->poly_ptr->is_pillar) return;
    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(node_ptr->surf_dirs, is_wall);
    if (!is_wall) {
        const Point3D ctnode_p = node_ptr->ctnode->position;
        const Point3D ev_p = ctnode_p + topo_dir * FARUtil::kLeafSize;
        if (FARUtil::IsConvexPoint(node_ptr->ctnode->poly_ptr, ev_p)) {
            node_ptr->free_direct = NodeFreeDirect::CONVEX;
        } else {
            node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        }
    }
}

void DynamicGraph::TopTwoContourConnector(const NavNodePtr& node_ptr) {
    std::vector<int> votesc;
    for (const auto& vote : node_ptr->contour_votes) {
        if (FARUtil::IsVoteTrue(vote.second, false)) {
            votesc.push_back(std::accumulate(vote.second.begin(), vote.second.end(), 0));
        }
    }
    std::sort(votesc.begin(), votesc.end(), std::greater<int>());
    for (const auto& cnode_ptr : node_ptr->potential_contours) {
        if (!cnode_ptr || node_ptr == cnode_ptr) continue;
        const auto it = node_ptr->contour_votes.find(cnode_ptr->id);
        // DEBUG
        if (it == node_ptr->contour_votes.end()) {
            ROS_ERROR("DG: contour potential node matching error");
            continue;
        }
        const int itc = std::accumulate(it->second.begin(), it->second.end(), 0);
        const bool vote_ready = FARUtil::IsVoteTrue(it->second, false);
        const bool current_adjacent =
            node_ptr->is_contour_match && cnode_ptr->is_contour_match &&
            ContourGraph::IsNavNodesConnectFromContour(node_ptr, cnode_ptr);
        const bool persistent_static_pair =
            (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
             node_ptr->source == GraphNodeSource::STATIC_GLOBAL) &&
            (cnode_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
             cnode_ptr->source == GraphNodeSource::STATIC_GLOBAL);
        const bool both_in_reliable_window =
            ContourGraph::IsPointInsideReliableContourWindow(
                node_ptr->position) &&
            ContourGraph::IsPointInsideReliableContourWindow(
                cnode_ptr->position);
        const float contour_observation_tolerance = std::max(
            FARUtil::kLeafSize * 2.0f,
            std::min(FARUtil::kMatchDist, FARUtil::kNavClearDist));
        const bool first_contour_observed =
            node_ptr->is_contour_match ||
            ContourGraph::IsPointObservedOnCurrentStaticContour(
                node_ptr->position, contour_observation_tolerance);
        const bool second_contour_observed =
            cnode_ptr->is_contour_match ||
            ContourGraph::IsPointObservedOnCurrentStaticContour(
                cnode_ptr->position, contour_observation_tolerance);
        const bool current_local_contradiction =
            semantic_update_in_progress_ && persistent_static_pair &&
            both_in_reliable_window && first_contour_observed &&
            second_contour_observed &&
            !current_adjacent;

        if (!current_adjacent) {
            auto state_it = node_ptr->edge_states.find(cnode_ptr->id);
            if (current_local_contradiction &&
                state_it != node_ptr->edge_states.end() &&
                node_ptr->id < cnode_ptr->id) {
                GraphEdgeState& forward = state_it->second;
                GraphEdgeState& reverse =
                    cnode_ptr->edge_states[node_ptr->id];
                ApplyContourTopologyObservation(
                        forward,
                        ContourTopologyObservation::CONTRADICTED,
                        dg_params_.static_remove_frames);
                reverse.topology_blocked = forward.topology_blocked;
                reverse.current_contour_misses =
                    forward.current_contour_misses;
                contour_edge_diagnostics_.push_back({
                    node_ptr->id, cnode_ptr->id, node_ptr->position,
                    cnode_ptr->position,
                    EdgeValidationMode::CONTOUR_FOLLOW,
                    EdgeRejectReason::NOT_CURRENT_ADJACENT});
            }
            // Outside the current verified contour, preserve persistent
            // static topology rather than rebuilding it from robot history.
            continue;
        }

        // A relation verified from the current contour is authoritative for
        // the current local overlay and must not wait behind old vote totals.
        // Historical relations outside the verified local contour still use
        // the original top-two vote rule.
        const bool selected = current_adjacent ||
            (vote_ready && FARUtil::VoteRankInVotes(itc, votesc) < 2);
        if (!selected) {
            if (node_ptr->id < cnode_ptr->id) {
                contour_edge_diagnostics_.push_back({
                    node_ptr->id, cnode_ptr->id, node_ptr->position,
                    cnode_ptr->position, EdgeValidationMode::CONTOUR_FOLLOW,
                    EdgeRejectReason::VOTE_PENDING});
            }
            continue;
        }

        EdgeValidationResult validation =
            ContourGraph::ValidateContourFollowEdge(node_ptr, cnode_ptr);
        if (validation.valid &&
            !IsOnTerrainRoute(validation.route_start,
                              validation.route_end)) {
            validation.valid = false;
            validation.reason = EdgeRejectReason::TERRAIN_BLOCKED;
        }
        if (!validation.valid) {
            // Geometry failure is immediate safety evidence, but not proof
            // that the contour identity itself disappeared. Preserve votes,
            // contour_connects and the last route so a noisy frame cannot
            // churn the graph; simply mask an existing edge until a later
            // snapshot validates it again.
            auto forward_it = node_ptr->edge_states.find(cnode_ptr->id);
            auto reverse_it = cnode_ptr->edge_states.find(node_ptr->id);
            if (forward_it != node_ptr->edge_states.end() &&
                reverse_it != cnode_ptr->edge_states.end()) {
                ApplyContourStaticValidationObservation(
                    forward_it->second, false);
                ApplyContourStaticValidationObservation(
                    reverse_it->second, false);
            }
            if (node_ptr->id < cnode_ptr->id) {
                contour_edge_diagnostics_.push_back({
                    node_ptr->id, cnode_ptr->id, node_ptr->position,
                    cnode_ptr->position, EdgeValidationMode::CONTOUR_FOLLOW,
                    validation.reason});
            }
            continue;
        }

        DynamicGraph::AddContourConnect(node_ptr, cnode_ptr);
        this->AddEdge(node_ptr, cnode_ptr);
        GraphEdgeState& forward = node_ptr->edge_states[cnode_ptr->id];
        GraphEdgeState& reverse = cnode_ptr->edge_states[node_ptr->id];
        ApplyContourTopologyObservation(
            forward, ContourTopologyObservation::CONFIRMED,
            dg_params_.static_remove_frames);
        ApplyContourTopologyObservation(
            reverse, ContourTopologyObservation::CONFIRMED,
            dg_params_.static_remove_frames);
        forward.validation_mode = reverse.validation_mode =
            EdgeValidationMode::CONTOUR_FOLLOW;
        forward.has_clearance_geometry =
            reverse.has_clearance_geometry = true;
        forward.route_start = validation.route_start;
        forward.route_end = validation.route_end;
        reverse.route_start = validation.route_end;
        reverse.route_end = validation.route_start;
        forward.route_cost = reverse.route_cost = validation.route_cost;
        ApplyContourStaticValidationObservation(forward, true);
        ApplyContourStaticValidationObservation(reverse, true);
        forward.current_contour_misses =
            reverse.current_contour_misses = 0;
        forward.dynamic_blocked = reverse.dynamic_blocked =
            validation.dynamic_blocked;
        if (validation.dynamic_blocked && node_ptr->id < cnode_ptr->id) {
            contour_edge_diagnostics_.push_back({
                node_ptr->id, cnode_ptr->id, validation.route_start,
                validation.route_end, EdgeValidationMode::CONTOUR_FOLLOW,
                EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED});
        }
    }
}

void DynamicGraph::RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->contour_votes.end()) != (it2 == node_ptr2->contour_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Contour edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init contour connection votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: contour connection votes are not equal.");
        }
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::RecordPolygonVote(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size, 
                                     const bool& is_reset) 
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->edge_votes.end()) != (it2 == node_ptr2->edge_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Polygon edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        // init polygon edge votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge votes are not equal.");
        }
        if (is_reset) it1->second.clear(), it2->second.clear();
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > queue_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::FillPolygonEdgeConnect(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2,
                                        const int& queue_size)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        std::deque<int> vote_queue1(queue_size, 1);
        std::deque<int> vote_queue2(queue_size, 1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && 
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) 
        {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
        // Add connections
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->poly_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->poly_connects)) 
        {
            node_ptr1->poly_connects.push_back(node_ptr2);
            node_ptr2->poly_connects.push_back(node_ptr1);
        }
    }
}

void DynamicGraph::FillContourConnect(const NavNodePtr& node_ptr1,
                                    const NavNodePtr& node_ptr2,
                                    const int& queue_size)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    std::deque<int> vote_queue1(queue_size, 1);
    std::deque<int> vote_queue2(queue_size, 1);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init polygon edge votes
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && 
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) 
        {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
        // Add contours
        DynamicGraph::AddContourConnect(node_ptr1, node_ptr2);
    }
}

void DynamicGraph::FillTrajConnect(const NavNodePtr& node_ptr1,
                                 const NavNodePtr& node_ptr2)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end()) {
        node_ptr1->trajectory_votes.insert({node_ptr2->id, 0});
        node_ptr2->trajectory_votes.insert({node_ptr1->id, 0});
        // Add connection
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->trajectory_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) 
        {   
            node_ptr1->trajectory_connects.push_back(node_ptr2);
            node_ptr2->trajectory_connects.push_back(node_ptr1);
        }
    }
}

void DynamicGraph::DeletePolygonVote(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size,
                                     const bool& is_reset) 
{
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) return;
    if (is_reset) it1->second.clear(), it2->second.clear();
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > queue_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

/* Delete Contour edge for given two navigation nodes */
void DynamicGraph::DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) return; // no connection (not counter init) in the first place 
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > dg_params_.votes_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

bool DynamicGraph::IsActivateNavNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_active) return true;
    if (FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
        node_ptr->is_active = true;
        return true;
    }
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        const bool is_nearby = (node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kNearDist ? true : false;
        if (is_nearby) {
            node_ptr->is_active = true;
            return true;
        }
        if (FARUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) {
            node_ptr->is_active = true;
            return true;
        }
        bool is_connects_activate = true;
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!cnode_ptr->is_active) {
                is_connects_activate = false;
                break;
            }
        }
        if ((is_connects_activate && !node_ptr->connect_nodes.empty())) {
            node_ptr->is_active = true;
            return true;
        }
    }
    return false;
}

void DynamicGraph::UpdateGlobalNearNodes() {
    /* update nearby navigation nodes stack --> near_nav_nodes_ */
    near_nav_nodes_.clear(), wide_near_nodes_.clear(), extend_match_nodes_.clear();
    margin_near_nodes_.clear(); internav_near_nodes_.clear(), surround_internav_nodes_.clear();
    // Odom is ephemeral, but a validated ODOM_CONNECT to a confirmed global
    // static corner is intentionally allowed to extend beyond the local
    // stitch zone.  Only local-overlay endpoints become invalid merely by
    // leaving their source-specific observation window.  All incident edges
    // are rebuilt from scratch by UpdateOdomConnections() later in this same
    // semantic update.
    const NodePtrStack odom_connects = odom_node_ptr_->connect_nodes;
    for (const auto& node_ptr : odom_connects) {
        if (!node_ptr) continue;
        const float distance =
            (node_ptr->position - odom_node_ptr_->position).norm_flat();
        if (ShouldPruneStartConnectionForRange(
                *node_ptr, distance, FARUtil::kSensorRange,
                dg_params_.static_stitch_radius)) {
            ErasePolyEdge(odom_node_ptr_, node_ptr);
            EraseEdge(odom_node_ptr_, node_ptr);
        }
    }
    NodePtrStack local_graph = globalGraphNodes_;
    local_graph.insert(local_graph.end(), staticCandidateGraphNodes_.begin(),
                       staticCandidateGraphNodes_.end());
    local_graph.insert(local_graph.end(), dynamicLocalGraphNodes_.begin(),
                       dynamicLocalGraphNodes_.end());
    for (const auto& node_ptr : local_graph) {
        node_ptr->is_near_nodes = false;
        node_ptr->is_wide_near  = false;
        if (node_ptr->source == GraphNodeSource::PATH_HISTORY ||
            node_ptr->is_navpoint) continue;
        const float graph_range =
            node_ptr->source == GraphNodeSource::DYNAMIC_LOCAL
                ? FARUtil::kSensorRange
                : dg_params_.static_stitch_radius;
        if ((node_ptr->position - odom_node_ptr_->position).norm_flat() >
            graph_range) continue;
        // The static stitch radius can intentionally be larger than
        // sensor_range so it covers the corners of a square semantic window.
        // Use the source-specific Graph range for both spatial gates; calling
        // FARUtil::IsNodeIn*Range() here would silently crop the square back
        // to the sensor_range inscribed circle.
        const bool in_extend_match_range =
            FARUtil::IsPointInToleratedHeight(
                node_ptr->position, FARUtil::kTolerZ * 1.5f) &&
            (node_ptr->position - FARUtil::odom_pos).norm() < graph_range;
        const bool in_local_graph_range =
            FARUtil::IsPointInToleratedHeight(
                node_ptr->position, FARUtil::kTolerZ) &&
            (node_ptr->position - FARUtil::odom_pos).norm() < graph_range;
        // Keep a known bad height out of the local stitch set, but do not crop
        // an otherwise valid semantic corner merely because its surrounding
        // floor has not yet been observed. Unknown terrain is traversable by
        // policy and final edge collision checks remain mandatory.
        bool terrain_matched = false;
        MapHandler::TerrainHeightOfPoint(node_ptr->position,
                                         terrain_matched, true);
        const bool terrain_neighbor_valid =
            !terrain_matched || MapHandler::IsNavPointOnTerrainNeighbor(
                node_ptr->position, true);
        if (in_extend_match_range &&
            (!node_ptr->is_active || terrain_neighbor_valid)) {
            if (FARUtil::IsOutsideGoal(node_ptr)) continue;
            if (this->IsActivateNavNode(node_ptr) || node_ptr->is_boundary) extend_match_nodes_.push_back(node_ptr);
            if (in_local_graph_range && IsPointOnTerrain(node_ptr->position)) {
                wide_near_nodes_.push_back(node_ptr);
                node_ptr->is_wide_near = true;
                if (node_ptr->is_active || node_ptr->is_boundary) {
                    near_nav_nodes_.push_back(node_ptr);
                    node_ptr->is_near_nodes = true;
                    if (node_ptr->is_navpoint) {
                        node_ptr->position.intensity = node_ptr->fgscore;
                        internav_near_nodes_.push_back(node_ptr);
                        if ((node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kLocalPlanRange / 2.0f) {
                            surround_internav_nodes_.push_back(node_ptr);
                        }
                    }
                }
            } else if (node_ptr->is_active || node_ptr->is_boundary) {
                margin_near_nodes_.push_back(node_ptr);
            }
        }
    }
    for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) { // add additional odom connections to wide near stack
        if (FARUtil::IsOutsideGoal(cnode_ptr)) continue;
        if ((cnode_ptr->position - odom_node_ptr_->position).norm_flat() >
            dg_params_.static_stitch_radius) continue;
        if (!cnode_ptr->is_wide_near) {
            wide_near_nodes_.push_back(cnode_ptr);
            cnode_ptr->is_wide_near = true;
        }
        for (const auto& c2node_ptr : cnode_ptr->connect_nodes) {
            if (!c2node_ptr->is_wide_near && !FARUtil::IsOutsideGoal(c2node_ptr)) {
                wide_near_nodes_.push_back(c2node_ptr);
                c2node_ptr->is_wide_near = true;
            }
        }
    }
    if (!internav_near_nodes_.empty()) { // find the nearest inter_nav node that connect to odom
        std::sort(internav_near_nodes_.begin(), internav_near_nodes_.end(), nodeptr_icomp());
        for (std::size_t i=0; i<internav_near_nodes_.size(); i++) {
            const NavNodePtr temp_internav_ptr = internav_near_nodes_[i];
            if (FARUtil::IsTypeInStack(temp_internav_ptr, odom_node_ptr_->potential_edges) && this->IsInternavInRange(temp_internav_ptr)) {
                if (cur_internav_ptr_ == NULL || temp_internav_ptr == cur_internav_ptr_ || (temp_internav_ptr->position - cur_internav_ptr_->position).norm() < FARUtil::kNearDist ||
                    FARUtil::IsTypeInStack(temp_internav_ptr, cur_internav_ptr_->connect_nodes)) 
                {   
                    this->UpdateCurInterNavNode(temp_internav_ptr);  
                } else {
                    is_bridge_internav_ = true;
                }
                break;
            }
        }
    }
}

bool DynamicGraph::ReEvaluateCorner(const NavNodePtr node_ptr) {
    if (node_ptr->is_boundary) return true;
    if (node_ptr->is_navpoint) {
        if (FARUtil::IsTypeInStack(node_ptr, surround_internav_nodes_) && this->IsNodeInTerrainOccupy(node_ptr)) {
            return false;
        }
        return true;
    }
    const bool is_near_new = FARUtil::IsPointNearNewPoints(node_ptr->position, false);
    if (is_near_new) { // if nearby env changes;
        this->ResetNodeFilters(node_ptr);
        if (!node_ptr->is_contour_match) this->ResetNodeConnectVotes(node_ptr);
    }
    if (!node_ptr->is_contour_match) {
        if (FARUtil::IsPointInMarginRange(node_ptr->position) || is_near_new) return false;
        return true;
    }
    if (node_ptr->is_finalized) return true;

    bool is_pos_cov  = false;
    bool is_dirs_cov = false;
    if (node_ptr->is_contour_match) {
        is_pos_cov  = this->UpdateNodePosition(node_ptr, node_ptr->ctnode->position);
        is_dirs_cov = this->UpdateNodeSurfDirs(node_ptr, node_ptr->ctnode->surf_dirs);
        if (FARUtil::IsDebug) ROS_ERROR_COND(node_ptr->free_direct == NodeFreeDirect::UNKNOW, "DG: node free space is unknown.");
    }
    if (is_pos_cov && is_dirs_cov) node_ptr->is_finalized = true;

    return true;
}

bool DynamicGraph::ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2) {
    PointStack terrain_path;
    if (terrain_planner_.PlanPathFromNodeToNode(node_ptr1, node_ptr2, terrain_path)) {
        return true;
    }
    return false;
}
