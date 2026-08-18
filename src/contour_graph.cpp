/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/contour_graph.h"
#include "far_planner/intersection.h"
#include "far_planner/terminal_visibility_policy.h"

PointCloudPtr ContourGraph::local_collision_cloud_(new PointCloud());
PointKdTreePtr ContourGraph::local_collision_kdtree_(
    new pcl::KdTreeFLANN<PCLPoint>());
PointCloudPtr ContourGraph::local_static_collision_cloud_(new PointCloud());
PointKdTreePtr ContourGraph::local_static_collision_kdtree_(
    new pcl::KdTreeFLANN<PCLPoint>());
PointCloudPtr ContourGraph::local_dynamic_collision_cloud_(new PointCloud());
PointKdTreePtr ContourGraph::local_dynamic_collision_kdtree_(
    new pcl::KdTreeFLANN<PCLPoint>());
float ContourGraph::contour_projection_min_ = 0.15f;
float ContourGraph::contour_projection_step_ = 0.075f;
float ContourGraph::contour_projection_max_ = 0.60f;
float ContourGraph::contour_boundary_guard_ = 0.40f;

/***************************************************************************************/

void ContourGraph::Init(const ContourGraphParams& params) {
    ctgraph_params_ = params;
    contour_projection_min_ = std::max(0.0f, params.contour_projection_min);
    contour_projection_step_ = std::max(
        FARUtil::kEpsilon, params.contour_projection_step);
    contour_projection_max_ = std::max(
        contour_projection_min_, params.contour_projection_max);
    contour_boundary_guard_ = std::max(
        FARUtil::kLeafSize, params.contour_boundary_guard);
    ContourGraph::contour_graph_.clear();
    ContourGraph::contour_polygons_.clear();
    ALIGN_ANGLE_COS = cos(M_PI - FARUtil::kAcceptAlign / 2.0f);
    is_robot_inside_poly_ = false;
    ContourGraph::global_contour_set_.clear();
    ContourGraph::boundary_contour_set_.clear();
}

void ContourGraph::UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                                      const std::vector<std::vector<Point3D>>& filtered_contours) {
    this->UpdateContourGraph(odom_node_ptr, filtered_contours,
                             std::vector<std::vector<Point3D>>());
}

void ContourGraph::UpdateContourGraph(
    const NavNodePtr& odom_node_ptr,
    const std::vector<std::vector<Point3D>>& static_contours,
    const std::vector<std::vector<Point3D>>& dynamic_contours) {
    odom_node_ptr_ = odom_node_ptr;
    this->ClearContourGraph();
    const auto add_polygons = [this](
        const std::vector<std::vector<Point3D>>& contours,
        const GraphNodeSource source) {
      for (const auto& poly : contours) {
        PolygonPtr new_poly_ptr = NULL;
        this->CreatePolygon(poly, new_poly_ptr, source);
        this->AddPolyToContourPolygon(new_poly_ptr);
      }
    };
    add_polygons(static_contours, GraphNodeSource::STATIC_CANDIDATE);
    add_polygons(dynamic_contours, GraphNodeSource::DYNAMIC_LOCAL);
    ContourGraph::UpdateOdomFreePosition(odom_node_ptr_, FARUtil::free_odom_p);
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p);
        CTNodePtr new_ctnode_ptr = NULL;
        if (poly_ptr->is_pillar) {
            Point3D mean_p = FARUtil::AveragePoints(poly_ptr->vertices);
            this->CreateCTNode(mean_p, new_ctnode_ptr, poly_ptr, true);
            new_ctnode_ptr->is_boundary_clipped =
                poly_ptr->is_boundary_clipped;
            this->AddCTNodeToGraph(new_ctnode_ptr);
        } else {
            CTNodeStack ctnode_stack;
            ctnode_stack.clear();
            const int N = poly_ptr->vertices.size();
            for (std::size_t idx=0; idx<N; idx++) {
                this->CreateCTNode(poly_ptr->vertices[idx], new_ctnode_ptr, poly_ptr, false);
                new_ctnode_ptr->is_boundary_clipped =
                    !IsPointInsideReliableContourWindow(
                        poly_ptr->vertices[idx]);
                ctnode_stack.push_back(new_ctnode_ptr);
            }
            // add connections to contour nodes
            for (int idx=0; idx<N; idx++) {
                int ref_idx = FARUtil::Mod(idx-1, N);
                ctnode_stack[idx]->front = ctnode_stack[ref_idx];
                ref_idx = FARUtil::Mod(idx+1, N);
                ctnode_stack[idx]->back = ctnode_stack[ref_idx];
                this->AddCTNodeToGraph(ctnode_stack[idx]);
            }
            // add first ctnode of each polygon to poly ctnodes stack
            if (!ctnode_stack.empty()) ContourGraph::polys_ctnodes_.push_back(ctnode_stack.front());
        }
    }
    this->AnalysisSurfAngleAndConvexity(ContourGraph::contour_graph_);      
}

void ContourGraph::SetLocalCollisionCloud(
    const PointCloudPtr& collision_cloud) {
    SetLocalCollisionCloud(collision_cloud, PointCloudPtr(new PointCloud()));
}

void ContourGraph::SetLocalCollisionCloud(
    const PointCloudPtr& static_cloud, const PointCloudPtr& dynamic_cloud) {
    const auto rebuild = [](const PointCloudPtr& input,
                            PointCloudPtr& output,
                            PointKdTreePtr& tree) {
        if (!output) output.reset(new PointCloud());
        output->clear();
        if (input) *output = *input;
        tree.reset(new pcl::KdTreeFLANN<PCLPoint>());
        tree->setSortedResults(false);
        if (!output->empty()) tree->setInputCloud(output);
    };
    rebuild(static_cloud, local_static_collision_cloud_,
            local_static_collision_kdtree_);
    rebuild(dynamic_cloud, local_dynamic_collision_cloud_,
            local_dynamic_collision_kdtree_);

    if (!local_collision_cloud_) local_collision_cloud_.reset(new PointCloud());
    local_collision_cloud_->clear();
    *local_collision_cloud_ += *local_static_collision_cloud_;
    *local_collision_cloud_ += *local_dynamic_collision_cloud_;
    local_collision_kdtree_.reset(new pcl::KdTreeFLANN<PCLPoint>());
    local_collision_kdtree_->setSortedResults(false);
    if (!local_collision_cloud_->empty()) {
        local_collision_kdtree_->setInputCloud(local_collision_cloud_);
    }
}

/* Match current contour with global navigation nodes */
// 已经拿到关键分支了：这套代码不是做“整轮轮廓刚性配准”，而是“局部匹配 + 未匹配保留 + 内外部可达性判断”。我现在把这三层逻辑串起来，直接对应到代码里的集合和条件。
// 是的，这段代码不是在做“局部轮廓和全局轮廓强行重合”的刚性配准，而是做三层处理：
// 第一层是“尽量匹配，匹配不上就别硬配”。在 MatchContourWithNavGraph 里，当前帧每个 CTNode 先去 near_nodes 里找最近的 NavNode，再过一次 IsCTMatchLineFreePolygon() 的可连通性检查。只有“距离合适 + 连线在自由多边形里”才算匹配成功；如果差别太大，或者中间有障碍，直接不匹配。最后没匹配上的轮廓点会进 new_convex_vertices，后面再作为新顶点候选进入图更新。
// 第二层是“局部不一致不会被硬吞掉，而是显式分流”。在 IsPointsConnectFreePolygon 里，局部检查和全局检查是分开的：
// 局部检查时，会检查当前局部 contour_polygons_，还会看 unmatched_contour_ 和 inactive_contour_，这些都能阻止一条边被认为可通行。
// 全局检查时，才去看 global_contour_。
// 所以如果当前局部轮廓和历史全局轮廓有明显差异，代码不会把它们“拉齐”，而是把不一致保留下来，变成匹配失败、未匹配轮廓、或 inactive 轮廓。
// 第三层是“区分轮廓内部还是外部，而且这个判断是显式存在的”。在 UpdateContourGraph 里，每个多边形都会计算 poly_ptr->is_robot_inside = PointInsideAPoly(...)。后面 UpdateOdomFreePosition 还会根据机器人是否在多边形内部，去找一个“外移后的 free 位置”。另外在可连通性检查里，is_robot_inside 会参与判断一条边是否跨越了“机器人所在侧”和“非机器人所在侧”的轮廓。也就是说，这里确实区分了轮廓内外，但它不是把两个轮廓系统做整体配准，而是用“当前机器人处于多边形内/外”和“边是否穿过轮廓”来做可达性约束。
void ContourGraph::MatchContourWithNavGraph(
    const NodePtrStack& global_nodes, const NodePtrStack& near_nodes,
    CTNodeStack& new_convex_vertices, const float static_duplicate_radius) {
    for (const auto& node_ptr : global_nodes) {
        node_ptr->is_contour_match = false;
        node_ptr->ctnode = NULL;
    }
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) {
        ctnode_ptr->is_global_match = false;
        ctnode_ptr->nav_node_id = 0;
    }

    // Build all plausible pairs first and assign them globally in increasing
    // score order.  The old CT-by-CT nearest loop allowed a later contour
    // vertex to steal a NavNode from an earlier one without rematching the
    // displaced vertex, so identities changed with findContours() iteration
    // order.  This deterministic one-to-one assignment makes every accepted
    // pair compete in the same frame.
    struct MatchCandidate {
        CTNodePtr contour_node;
        NavNodePtr nav_node;
        float score;
        float distance;
    };
    std::vector<MatchCandidate> match_candidates;
    const float direction_threshold = 0.5f;
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) {
        if (!ctnode_ptr ||
            ctnode_ptr->free_direct == NodeFreeDirect::UNKNOW) {
            continue;
        }
        const bool static_contour =
            ctnode_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
            ctnode_ptr->source == GraphNodeSource::STATIC_GLOBAL;
        const bool dynamic_contour =
            ctnode_ptr->source == GraphNodeSource::DYNAMIC_LOCAL;
        for (const auto& node_ptr : near_nodes) {
            if (!node_ptr || node_ptr->is_odom || node_ptr->is_navpoint ||
                FARUtil::IsOutsideGoal(node_ptr) ||
                !IsInMatchHeight(ctnode_ptr, node_ptr)) {
                continue;
            }
            const bool static_node =
                node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
                node_ptr->source == GraphNodeSource::STATIC_GLOBAL;
            const bool dynamic_node =
                node_ptr->source == GraphNodeSource::DYNAMIC_LOCAL;
            if ((static_contour && !static_node) ||
                (dynamic_contour && !dynamic_node)) {
                continue;
            }
            if (!IsContourEndpointLifetimeMatchCompatible(
                    static_contour, ctnode_ptr->is_boundary_clipped,
                    *node_ptr)) {
                continue;
            }
            const bool contour_pillar =
                ctnode_ptr->free_direct == NodeFreeDirect::PILLAR;
            const bool node_pillar =
                node_ptr->free_direct == NodeFreeDirect::PILLAR;
            if (contour_pillar != node_pillar) continue;

            float direction_score = 0.0f;
            if (dynamic_contour && dynamic_node) {
                direction_score = 1.0f;
            } else if (!contour_pillar &&
                       node_ptr->free_direct != NodeFreeDirect::UNKNOW &&
                       ctnode_ptr->free_direct == node_ptr->free_direct) {
                const Point3D nav_direction =
                    FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
                const Point3D contour_direction =
                    FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs);
                direction_score =
                    (nav_direction * contour_direction - direction_threshold) /
                    (1.0f - direction_threshold);
            } else if (contour_pillar && node_pillar) {
                direction_score = 0.5f;
            }
            // Preserve FAR's source/type and surface-direction identity, but
            // do not let a noisy semantic contour direction create another
            // vertex at practically the same corner. A weak/misaligned
            // direction gets only a tight grid-scale positional fallback;
            // it can never merge two distinct nearby door-frame corners.
            if (!dynamic_contour && !contour_pillar &&
                node_ptr->free_direct != ctnode_ptr->free_direct) {
                continue;
            }
            const float tight_position_radius = std::max(
                FARUtil::kLeafSize * 2.0f,
                FARUtil::robot_dim * 0.5f);
            const float match_radius = direction_score > 0.0f
                ? FARUtil::kMatchDist * std::max(0.5f, direction_score)
                : tight_position_radius;
            const float distance =
                (node_ptr->position - ctnode_ptr->position).norm_flat();
            if (distance >= match_radius ||
                !IsCTMatchLineFreePolygon(ctnode_ptr, node_ptr, false)) {
                continue;
            }
            // Position is primary; direction agreement acts as a small
            // regularizer. Stable node id and contour coordinates below
            // resolve exact voxel-grid ties.
            const float score = distance /
                std::max(match_radius, FARUtil::kEpsilon) +
                (1.0f - std::max(0.0f, direction_score)) * 0.10f;
            match_candidates.push_back(
                {ctnode_ptr, node_ptr, score, distance});
        }
    }
    std::sort(match_candidates.begin(), match_candidates.end(),
              [](const MatchCandidate& first, const MatchCandidate& second) {
        if (std::fabs(first.score - second.score) > FARUtil::kEpsilon) {
            return first.score < second.score;
        }
        if (first.nav_node->id != second.nav_node->id) {
            return first.nav_node->id < second.nav_node->id;
        }
        if (std::fabs(first.contour_node->position.x -
                      second.contour_node->position.x) > FARUtil::kEpsilon) {
            return first.contour_node->position.x <
                   second.contour_node->position.x;
        }
        return first.contour_node->position.y <
               second.contour_node->position.y;
    });
    std::unordered_set<std::size_t> assigned_nav_ids;
    std::unordered_set<const CTNode*> assigned_contour_nodes;
    for (const auto& candidate : match_candidates) {
        if (assigned_nav_ids.count(candidate.nav_node->id) ||
            assigned_contour_nodes.count(candidate.contour_node.get())) {
            continue;
        }
        this->MatchCTNodeWithNavNode(candidate.contour_node,
                                     candidate.nav_node);
        assigned_nav_ids.insert(candidate.nav_node->id);
        assigned_contour_nodes.insert(candidate.contour_node.get());
    }
    this->EnclosePolygonsCheck();
    new_convex_vertices.clear();
    std::size_t rejected_concave = 0;
    std::size_t suppressed_duplicates = 0;
    const float duplicate_radius = std::max(0.0f, static_duplicate_radius);
    const float duplicate_direction_cos = std::cos(
        std::max(FARUtil::kAngleNoise * 2.0f,
                 static_cast<float>(15.0 * M_PI / 180.0)));
    const auto is_static_contour = [](const CTNodePtr& node) {
        return node &&
            (node->source == GraphNodeSource::STATIC_CANDIDATE ||
             node->source == GraphNodeSource::STATIC_GLOBAL);
    };
    const auto same_static_corner = [duplicate_radius,
                                     duplicate_direction_cos](
        const CTNodePtr& current, const Point3D& other_position,
        const PointPair& other_dirs, const NodeFreeDirect other_free_direct) {
        if (!current || duplicate_radius <= 0.0f ||
            current->is_boundary_clipped ||
            current->free_direct != other_free_direct ||
            (current->position - other_position).norm_flat() >
                duplicate_radius ||
            std::fabs(current->position.z - other_position.z) >
                FARUtil::kTolerZ) {
            return false;
        }
        if (current->free_direct == NodeFreeDirect::PILLAR) return true;
        const Point3D current_direction =
            FARUtil::SurfTopoDirect(current->surf_dirs);
        const Point3D other_direction =
            FARUtil::SurfTopoDirect(other_dirs);
        return current_direction * other_direction >=
               duplicate_direction_cos;
    };
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) { // Get new vertices
        if (!ctnode_ptr->is_global_match &&
            ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
            if (is_static_contour(ctnode_ptr) &&
                ctnode_ptr->free_direct == NodeFreeDirect::CONCAVE) {
                ++rejected_concave;
                continue;
            }
            if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) { // check wall contour
                const float dot_value = ctnode_ptr->surf_dirs.first * ctnode_ptr->surf_dirs.second;
                if (dot_value < ALIGN_ANGLE_COS) continue; // wall detected
            }
            bool duplicate = false;
            if (is_static_contour(ctnode_ptr)) {
                // A second current corner is suppressed only when an already
                // matched/accepted static routing vertex is extremely close,
                // has the same corner class and nearly the same free-space
                // direction. Distinct door-frame corners therefore survive.
                for (const auto& node_ptr : near_nodes) {
                    if (!node_ptr || !node_ptr->is_contour_match ||
                        (node_ptr->source !=
                             GraphNodeSource::STATIC_CANDIDATE &&
                         node_ptr->source != GraphNodeSource::STATIC_GLOBAL)) {
                        continue;
                    }
                    if (same_static_corner(
                            ctnode_ptr, node_ptr->position,
                            node_ptr->surf_dirs, node_ptr->free_direct) &&
                        IsCTMatchLineFreePolygon(
                            ctnode_ptr, node_ptr, false)) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    for (const auto& accepted : new_convex_vertices) {
                        if (!is_static_contour(accepted)) continue;
                        if (same_static_corner(
                                ctnode_ptr, accepted->position,
                                accepted->surf_dirs,
                                accepted->free_direct)) {
                            duplicate = true;
                            break;
                        }
                    }
                }
            }
            if (duplicate) {
                ++suppressed_duplicates;
                continue;
            }
            new_convex_vertices.push_back(ctnode_ptr);
        }
    }
    ROS_INFO_THROTTLE(
        5.0,
        "CG static routing filter: rejected_concave=%zu suppressed_duplicates=%zu",
        rejected_concave, suppressed_duplicates);
}

bool ContourGraph::IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position);
    ConnectPair cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, FARUtil::kProjectDist, is_global_check);
    if (node_ptr1->is_odom) {
        cedge.start_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y);
    } else if (node_ptr2->is_odom) {
        cedge.end_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y);
    }
    ConnectPair bd_cedge = cedge;
    const HeightPair h_pair(node_ptr1->position, node_ptr2->position);
    if (!node_ptr1->is_boundary) bd_cedge.start_p = cv::Point2f(node_ptr1->position.x, node_ptr1->position.y);
    if (!node_ptr2->is_boundary) bd_cedge.end_p = cv::Point2f(node_ptr2->position.x, node_ptr2->position.y);
    const PolygonPtr endpoint_poly1 =
        node_ptr1->ctnode ? node_ptr1->ctnode->poly_ptr : PolygonPtr();
    const PolygonPtr endpoint_poly2 =
        node_ptr2->ctnode ? node_ptr2->ctnode->poly_ptr : PolygonPtr();
    return ContourGraph::IsPointsConnectFreePolygonForLayer(
        cedge, bd_cedge, h_pair, is_global_check, CollisionLayer::COMBINED,
        endpoint_poly1, endpoint_poly2);
}

bool ContourGraph::IsNavNodesConnectFreeStaticPolygon(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const bool is_global_check =
        ContourGraph::IsNeedGlobalCheck(node_ptr1->position,
                                        node_ptr2->position);
    ConnectPair cedge = ContourGraph::ReprojectEdge(
        node_ptr1, node_ptr2, FARUtil::kProjectDist, is_global_check);
    if (node_ptr1->is_odom) {
        cedge.start_p = cv::Point2f(FARUtil::free_odom_p.x,
                                    FARUtil::free_odom_p.y);
    } else if (node_ptr2->is_odom) {
        cedge.end_p = cv::Point2f(FARUtil::free_odom_p.x,
                                  FARUtil::free_odom_p.y);
    }
    ConnectPair bd_cedge = cedge;
    const HeightPair h_pair(node_ptr1->position, node_ptr2->position);
    if (!node_ptr1->is_boundary) {
        bd_cedge.start_p = cv::Point2f(node_ptr1->position.x,
                                       node_ptr1->position.y);
    }
    if (!node_ptr2->is_boundary) {
        bd_cedge.end_p = cv::Point2f(node_ptr2->position.x,
                                     node_ptr2->position.y);
    }
    const PolygonPtr endpoint_poly1 =
        node_ptr1->ctnode ? node_ptr1->ctnode->poly_ptr : PolygonPtr();
    const PolygonPtr endpoint_poly2 =
        node_ptr2->ctnode ? node_ptr2->ctnode->poly_ptr : PolygonPtr();
    return ContourGraph::IsPointsConnectFreePolygonForLayer(
        cedge, bd_cedge, h_pair, is_global_check,
        CollisionLayer::STATIC_ONLY, endpoint_poly1, endpoint_poly2);
}

bool ContourGraph::IsNavNodesConnectFreeDynamicLayer(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    // Dynamic blocking tests the actual static edge. Endpoint margins in the
    // raw-cloud check prevent the obstacle vertices themselves from falsely
    // blocking an otherwise valid edge.
    const ConnectPair edge(node_ptr1->position, node_ptr2->position);
    const HeightPair height(node_ptr1->position, node_ptr2->position);
    return ContourGraph::IsPointsConnectFreePolygonForLayer(
        edge, edge, height, false, CollisionLayer::DYNAMIC_ONLY);
}

EdgeRejectReason ContourGraph::ValidateVisibilityEdgeGeometry(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2,
    const bool include_dynamic) {
    if (node_ptr1 && node_ptr2 &&
        (node_ptr1->is_odom || node_ptr1->is_goal ||
         node_ptr2->is_odom || node_ptr2->is_goal)) {
        return ValidateVisibilityEdgeWithRoute(
                   node_ptr1, node_ptr2, include_dynamic).reason;
    }
    if (!node_ptr1 || !node_ptr2) return EdgeRejectReason::UNREACHABLE;
    const bool is_global_check = IsNeedGlobalCheck(node_ptr1->position,
                                                   node_ptr2->position);
    ConnectPair edge = ReprojectEdge(node_ptr1, node_ptr2,
                                     FARUtil::kProjectDist,
                                     is_global_check);
    if (node_ptr1->is_odom) {
        edge.start_p = cv::Point2f(FARUtil::free_odom_p.x,
                                   FARUtil::free_odom_p.y);
    } else if (node_ptr2->is_odom) {
        edge.end_p = cv::Point2f(FARUtil::free_odom_p.x,
                                 FARUtil::free_odom_p.y);
    }
    ConnectPair boundary_edge = edge;
    if (!node_ptr1->is_boundary) {
        boundary_edge.start_p = cv::Point2f(node_ptr1->position.x,
                                            node_ptr1->position.y);
    }
    if (!node_ptr2->is_boundary) {
        boundary_edge.end_p = cv::Point2f(node_ptr2->position.x,
                                          node_ptr2->position.y);
    }
    const HeightPair height(node_ptr1->position, node_ptr2->position);
    const PolygonPtr endpoint_poly1 =
        node_ptr1->ctnode ? node_ptr1->ctnode->poly_ptr : PolygonPtr();
    const PolygonPtr endpoint_poly2 =
        node_ptr2->ctnode ? node_ptr2->ctnode->poly_ptr : PolygonPtr();
    if (!IsEdgeCollisionFreeInCloud(
            edge, height, local_static_collision_cloud_,
            local_static_collision_kdtree_)) {
        return EdgeRejectReason::STATIC_CLOUD_BLOCKED;
    }
    if (include_dynamic && !IsEdgeCollisionFreeInCloud(
            edge, height, local_dynamic_collision_cloud_,
            local_dynamic_collision_kdtree_)) {
        return EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
    }
    if (!IsPointsConnectFreePolygonForLayer(
            edge, boundary_edge, height, is_global_check,
            CollisionLayer::STATIC_ONLY, endpoint_poly1, endpoint_poly2,
            false)) {
        return EdgeRejectReason::POLYGON_BLOCKED;
    }
    if (!include_dynamic) return EdgeRejectReason::NONE;
    if (!IsPointsConnectFreePolygonForLayer(
            edge, boundary_edge, height, is_global_check,
            CollisionLayer::DYNAMIC_ONLY, endpoint_poly1, endpoint_poly2,
            false)) {
        return EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
    }
    return EdgeRejectReason::NONE;
}

EdgeValidationResult ContourGraph::ValidateVisibilityEdgeWithRoute(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2,
    const bool include_dynamic) {
    if (!node_ptr1 || !node_ptr2) {
        EdgeValidationResult invalid;
        invalid.reason = EdgeRejectReason::UNREACHABLE;
        return invalid;
    }
    const bool first_is_terminal = node_ptr1->is_odom || node_ptr1->is_goal;
    const bool second_is_terminal = node_ptr2->is_odom || node_ptr2->is_goal;
    if (first_is_terminal != second_is_terminal) {
        return first_is_terminal
            ? ValidateTerminalVisibilityEdgeWithRoute(
                  node_ptr2, node_ptr1, false, include_dynamic)
            : ValidateTerminalVisibilityEdgeWithRoute(
                  node_ptr1, node_ptr2, true, include_dynamic);
    }

    EdgeValidationResult result;
    result.reason = ValidateVisibilityEdgeGeometry(
        node_ptr1, node_ptr2, include_dynamic);
    result.valid = result.reason == EdgeRejectReason::NONE;
    result.dynamic_blocked =
        result.reason == EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
    if (!node_ptr1 || !node_ptr2) return result;
    const bool is_global_check =
        IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position);
    ConnectPair edge = ReprojectEdge(node_ptr1, node_ptr2,
                                     FARUtil::kProjectDist,
                                     is_global_check);
    if (node_ptr1->is_odom) {
        edge.start_p = cv::Point2f(FARUtil::free_odom_p.x,
                                   FARUtil::free_odom_p.y);
    } else if (node_ptr2->is_odom) {
        edge.end_p = cv::Point2f(FARUtil::free_odom_p.x,
                                 FARUtil::free_odom_p.y);
    }
    result.route_start = Point3D(edge.start_p.x, edge.start_p.y,
                                 node_ptr1->position.z);
    result.route_end = Point3D(edge.end_p.x, edge.end_p.y,
                               node_ptr2->position.z);
    result.route_cost =
        (node_ptr1->position - result.route_start).norm() +
        (result.route_start - result.route_end).norm() +
        (result.route_end - node_ptr2->position).norm();
    result.projection_distance = FARUtil::kProjectDist;
    return result;
}

EdgeValidationResult ContourGraph::ValidateTerminalVisibilityEdgeWithRoute(
    const NavNodePtr& obstacle_node, const NavNodePtr& terminal_node,
    const bool obstacle_is_start, const bool include_dynamic) {
    EdgeValidationResult invalid;
    invalid.reason = EdgeRejectReason::UNREACHABLE;
    if (!obstacle_node || !terminal_node || obstacle_node == terminal_node) {
        return invalid;
    }

    const float node_distance =
        (obstacle_node->position - terminal_node->position).norm_flat();
    if (node_distance < FARUtil::kEpsilon) {
        invalid.valid = true;
        invalid.reason = EdgeRejectReason::NONE;
        invalid.route_start = obstacle_is_start
            ? obstacle_node->position : terminal_node->position;
        invalid.route_end = obstacle_is_start
            ? terminal_node->position : obstacle_node->position;
        return invalid;
    }

    const bool is_global_check = IsNeedGlobalCheck(
        obstacle_node->position, terminal_node->position);
    const PolygonPtr endpoint_poly = obstacle_node->ctnode
        ? obstacle_node->ctnode->poly_ptr : PolygonPtr();
    const cv::Point2f project_direction = NodeProjectDir(obstacle_node);
    const bool can_project =
        std::hypot(project_direction.x, project_direction.y) >
        FARUtil::kEpsilon;
    const HeightPair height(obstacle_node->position,
                            terminal_node->position);
    const float terminal_projection_max = std::max(
        contour_projection_max_,
        FARUtil::kNavClearDist + FARUtil::kLeafSize);

    const TerminalProjectionSearchResult search =
        FindNearestSafeTerminalProjection(
            contour_projection_min_, terminal_projection_max,
            contour_projection_step_, can_project,
            [&](const float requested_projection) {
                EdgeValidationResult attempt;
                attempt.reason = EdgeRejectReason::OFFSET_FAILED;

                const float projection = can_project
                    ? std::min(node_distance * 0.4f,
                               requested_projection)
                    : 0.0f;
                Point3D projected_corner = obstacle_node->position;
                projected_corner.x += project_direction.x * projection;
                projected_corner.y += project_direction.y * projection;

                const Point3D route_start = obstacle_is_start
                    ? projected_corner : terminal_node->position;
                const Point3D route_end = obstacle_is_start
                    ? terminal_node->position : projected_corner;
                const ConnectPair route(route_start, route_end);
                ConnectPair boundary_route = route;
                if (!obstacle_node->is_boundary) {
                    const cv::Point2f raw_corner(obstacle_node->position.x,
                                                 obstacle_node->position.y);
                    if (obstacle_is_start) {
                        boundary_route.start_p = raw_corner;
                    } else {
                        boundary_route.end_p = raw_corner;
                    }
                }

                // Once a corner has been projected, the stored route is a
                // robot-centre trajectory.  Check it without endpoint
                // exclusion so increasing the projection can genuinely move
                // the route outside the obstacle clearance band.  Pillars
                // have no reliable projection direction and retain the
                // legacy endpoint exclusion.
                const float endpoint_exclusion = can_project ? 0.0f : -1.0f;
                if (!IsEdgeCollisionFreeInCloud(
                        route, height, local_static_collision_cloud_,
                        local_static_collision_kdtree_,
                        endpoint_exclusion)) {
                    attempt.reason = EdgeRejectReason::STATIC_CLOUD_BLOCKED;
                    attempt.projection_distance = projection;
                    return attempt;
                }
                if (include_dynamic && !IsEdgeCollisionFreeInCloud(
                        route, height, local_dynamic_collision_cloud_,
                        local_dynamic_collision_kdtree_,
                        endpoint_exclusion)) {
                    attempt.reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
                    attempt.dynamic_blocked = true;
                    attempt.projection_distance = projection;
                    return attempt;
                }
                if (is_global_check &&
                    !IsRouteClearOfGlobalContours(
                        route, height, FARUtil::kNavClearDist)) {
                    attempt.reason = EdgeRejectReason::POLYGON_BLOCKED;
                    attempt.projection_distance = projection;
                    return attempt;
                }
                if (!IsPointsConnectFreePolygonForLayer(
                        route, boundary_route, height, is_global_check,
                        CollisionLayer::STATIC_ONLY, endpoint_poly,
                        PolygonPtr(), false)) {
                    attempt.reason = EdgeRejectReason::POLYGON_BLOCKED;
                    attempt.projection_distance = projection;
                    return attempt;
                }
                if (include_dynamic &&
                    !IsPointsConnectFreePolygonForLayer(
                        route, boundary_route, height, is_global_check,
                        CollisionLayer::DYNAMIC_ONLY, endpoint_poly,
                        PolygonPtr(), false)) {
                    attempt.reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
                    attempt.dynamic_blocked = true;
                    attempt.projection_distance = projection;
                    return attempt;
                }

                attempt.valid = true;
                attempt.reason = EdgeRejectReason::NONE;
                attempt.route_start = route_start;
                attempt.route_end = route_end;
                attempt.projection_distance = projection;
                attempt.route_cost =
                    (obstacle_node->position - projected_corner).norm() +
                    (route_start - route_end).norm();
                return attempt;
            });
    return search.validation;
}

bool ContourGraph::IsRouteClearOfGlobalContours(
    const ConnectPair& route, const HeightPair& height,
    const float clearance) {
    const float required_clearance = std::max(0.0f, clearance);
    const Point3D route_start(route.start_p.x, route.start_p.y, 0.0f);
    const Point3D route_end(route.end_p.x, route.end_p.y, 0.0f);
    const PointPair route_line(route_start, route_end);
    for (const PointPair& contour : global_contour_) {
        if (!IsEdgeOverlapInHeight(
                height, HeightPair(contour.first, contour.second))) {
            continue;
        }
        if (IsEdgeCollideSegment(contour, route)) return false;
        const float distance = std::min(
            std::min(FARUtil::DistanceToLineSeg2D(
                         route_start, contour),
                     FARUtil::DistanceToLineSeg2D(route_end, contour)),
            std::min(FARUtil::DistanceToLineSeg2D(
                         contour.first, route_line),
                     FARUtil::DistanceToLineSeg2D(
                         contour.second, route_line)));
        if (distance < required_clearance - FARUtil::kEpsilon) {
            return false;
        }
    }
    return true;
}

EdgeRejectReason ContourGraph::ValidateGoalEdgeGeometry(
    const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    if (!node_ptr || !goal_ptr) return EdgeRejectReason::UNREACHABLE;
    if (FARUtil::IsMultiLayer) {
        if (!FARUtil::IsAtSameLayer(node_ptr, goal_ptr) &&
            !node_ptr->is_frontier) {
            return EdgeRejectReason::DIRECTION_REJECTED;
        }
    }
    return ValidateTerminalVisibilityEdgeWithRoute(
               node_ptr, goal_ptr, true, true).reason;
}

EdgeValidationResult ContourGraph::ValidateGoalEdgeWithRoute(
    const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    if (!node_ptr || !goal_ptr) {
        EdgeValidationResult invalid;
        invalid.reason = EdgeRejectReason::UNREACHABLE;
        return invalid;
    }
    if (FARUtil::IsMultiLayer &&
        !FARUtil::IsAtSameLayer(node_ptr, goal_ptr) &&
        !node_ptr->is_frontier) {
        EdgeValidationResult invalid;
        invalid.reason = EdgeRejectReason::DIRECTION_REJECTED;
        return invalid;
    }
    return ValidateTerminalVisibilityEdgeWithRoute(
        node_ptr, goal_ptr, true, true);
}

EdgeValidationResult ContourGraph::ValidateDirectOdomGoalEdgeWithRoute(
    const NavNodePtr& odom_ptr, const NavNodePtr& goal_ptr,
    const bool include_dynamic) {
    EdgeValidationResult result;
    result.reason = EdgeRejectReason::UNREACHABLE;
    if (!odom_ptr || !goal_ptr || !odom_ptr->is_odom || !goal_ptr->is_goal) {
        return result;
    }

    // Both endpoints describe the robot centre in free space.  In particular,
    // odom must never be passed through the obstacle-corner projection path:
    // its legacy endpoint exclusion can hide a nearby wall on a short direct
    // edge.  A zero exclusion samples the complete segment, including both
    // endpoint neighbourhoods.
    const Point3D route_start = odom_ptr->position;
    const Point3D route_end = goal_ptr->position;
    const ConnectPair route(route_start, route_end);
    const HeightPair height(route_start, route_end);
    constexpr float kNoEndpointExclusion = 0.0f;

    if (!IsEdgeCollisionFreeInCloud(
            route, height, local_static_collision_cloud_,
            local_static_collision_kdtree_, kNoEndpointExclusion)) {
        result.reason = EdgeRejectReason::STATIC_CLOUD_BLOCKED;
        return result;
    }
    if (include_dynamic && !IsEdgeCollisionFreeInCloud(
            route, height, local_dynamic_collision_cloud_,
            local_dynamic_collision_kdtree_, kNoEndpointExclusion)) {
        result.reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
        result.dynamic_blocked = true;
        return result;
    }

    const bool is_global_check = IsNeedGlobalCheck(route_start, route_end);
    if (is_global_check &&
        !IsRouteClearOfGlobalContours(
            route, height, FARUtil::kNavClearDist)) {
        result.reason = EdgeRejectReason::POLYGON_BLOCKED;
        return result;
    }
    if (!IsPointsConnectFreePolygonForLayer(
            route, route, height, is_global_check,
            CollisionLayer::STATIC_ONLY, PolygonPtr(), PolygonPtr(), false)) {
        result.reason = EdgeRejectReason::POLYGON_BLOCKED;
        return result;
    }
    if (include_dynamic && !IsPointsConnectFreePolygonForLayer(
            route, route, height, is_global_check,
            CollisionLayer::DYNAMIC_ONLY, PolygonPtr(), PolygonPtr(), false)) {
        result.reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
        result.dynamic_blocked = true;
        return result;
    }

    result.valid = true;
    result.reason = EdgeRejectReason::NONE;
    result.route_start = route_start;
    result.route_end = route_end;
    result.route_cost = (route_end - route_start).norm();
    result.projection_distance = 0.0f;
    return result;
}

bool ContourGraph::IsRouteConnectFreeDynamicLayer(
    const Point3D& route_start, const Point3D& route_end) {
    const ConnectPair route(route_start, route_end);
    const HeightPair height(route_start, route_end);
    if (!IsEdgeCollisionFreeInCloud(
            route, height, local_dynamic_collision_cloud_,
            local_dynamic_collision_kdtree_, 0.0f)) {
        return false;
    }
    return IsPointsConnectFreePolygonForLayer(
        route, route, height, false, CollisionLayer::DYNAMIC_ONLY,
        PolygonPtr(), PolygonPtr(), false);
}

bool ContourGraph::IsRouteConnectFreeStaticLayer(
    const Point3D& route_start, const Point3D& route_end) {
    const ConnectPair route(route_start, route_end);
    const HeightPair height(route_start, route_end);
    if (!IsEdgeCollisionFreeInCloud(
            route, height, local_static_collision_cloud_,
            local_static_collision_kdtree_, 0.0f)) {
        return false;
    }
    return IsPointsConnectFreePolygonForLayer(
        route, route, height, false, CollisionLayer::STATIC_ONLY,
        PolygonPtr(), PolygonPtr(), false);
}

bool ContourGraph::IsPointInsideReliableContourWindow(
    const Point3D& point) {
    const float half_extent = std::max(
        0.0f, FARUtil::kSensorRange - contour_boundary_guard_);
    return std::abs(point.x - FARUtil::odom_pos.x) <= half_extent &&
           std::abs(point.y - FARUtil::odom_pos.y) <= half_extent;
}

bool ContourGraph::DoesSegmentIntersectReliableContourWindow(
    const Point3D& start, const Point3D& end) {
    const float half_extent = std::max(
        0.0f, FARUtil::kSensorRange - contour_boundary_guard_);
    const float min_x = FARUtil::odom_pos.x - half_extent;
    const float max_x = FARUtil::odom_pos.x + half_extent;
    const float min_y = FARUtil::odom_pos.y - half_extent;
    const float max_y = FARUtil::odom_pos.y + half_extent;
    float lower = 0.0f;
    float upper = 1.0f;
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const auto clip_axis = [&lower, &upper](
        const float origin, const float direction,
        const float minimum, const float maximum) {
        if (std::abs(direction) <= FARUtil::kEpsilon) {
            return origin >= minimum && origin <= maximum;
        }
        float first = (minimum - origin) / direction;
        float second = (maximum - origin) / direction;
        if (first > second) std::swap(first, second);
        lower = std::max(lower, first);
        upper = std::min(upper, second);
        return lower <= upper;
    };
    return clip_axis(start.x, dx, min_x, max_x) &&
           clip_axis(start.y, dy, min_y, max_y);
}

bool ContourGraph::IsPointObservedOnCurrentStaticContour(
    const Point3D& point, const float tolerance) {
    const float distance_tolerance = std::max(FARUtil::kEpsilon, tolerance);
    for (const auto& polygon : contour_polygons_) {
        if (!polygon || polygon->source == GraphNodeSource::DYNAMIC_LOCAL ||
            polygon->vertices.empty()) {
            continue;
        }
        if (polygon->is_pillar) {
            for (const auto& vertex : polygon->vertices) {
                if ((vertex - point).norm_flat() <= distance_tolerance) {
                    return true;
                }
            }
            continue;
        }
        for (std::size_t index = 0; index < polygon->vertices.size(); ++index) {
            const Point3D& first = polygon->vertices[index];
            const Point3D& second = polygon->vertices[
                (index + 1) % polygon->vertices.size()];
            // Do not use a segment incident to a cropped raster vertex as
            // proof that an old topology endpoint was observed.  In
            // particular this excludes findContours()' artificial closing
            // segment across the query-window boundary.
            if (!IsPointInsideReliableContourWindow(first) ||
                !IsPointInsideReliableContourWindow(second)) {
                continue;
            }
            if (FARUtil::DistanceToLineSeg2D(
                    point, PointPair(first, second)) <= distance_tolerance) {
                return true;
            }
        }
    }
    return false;
}

bool ContourGraph::IsPointConfirmedOnCurrentStaticSegmentInterior(
    const Point3D& point, const float tolerance, const float endpoint_guard,
    PolygonPtr* matched_polygon) {
    if (matched_polygon) *matched_polygon = PolygonPtr();
    const float distance_tolerance = std::max(FARUtil::kEpsilon, tolerance);
    const float endpoint_clearance = std::max(
        distance_tolerance, endpoint_guard);
    float best_distance = FARUtil::kINF;
    PolygonPtr best_polygon;

    for (const auto& polygon : contour_polygons_) {
        if (!polygon || polygon->source == GraphNodeSource::DYNAMIC_LOCAL ||
            polygon->is_pillar || polygon->is_boundary_clipped ||
            polygon->vertices.size() < 2) {
            continue;
        }
        for (std::size_t index = 0; index < polygon->vertices.size(); ++index) {
            const Point3D& first = polygon->vertices[index];
            const Point3D& second = polygon->vertices[
                (index + 1) % polygon->vertices.size()];
            if (!IsPointInsideReliableContourWindow(first) ||
                !IsPointInsideReliableContourWindow(second)) {
                continue;
            }
            const float dx = second.x - first.x;
            const float dy = second.y - first.y;
            const float length_sq = dx * dx + dy * dy;
            const float segment_length = std::sqrt(length_sq);
            if (segment_length <= endpoint_clearance * 2.0f ||
                length_sq <= FARUtil::kEpsilon) {
                continue;
            }
            const float projection = std::max(
                0.0f, std::min(1.0f,
                    ((point.x - first.x) * dx +
                     (point.y - first.y) * dy) / length_sq));
            const float along = projection * segment_length;
            if (along <= endpoint_clearance ||
                segment_length - along <= endpoint_clearance) {
                continue;
            }
            const Point3D projected(first.x + projection * dx,
                                    first.y + projection * dy, point.z);
            const float distance = (projected - point).norm_flat();
            if (distance <= distance_tolerance && distance < best_distance) {
                best_distance = distance;
                best_polygon = polygon;
            }
        }
    }
    if (!best_polygon) return false;

    // A detected current corner close to the historical vertex is evidence
    // for identity jitter, not evidence that the historical corner vanished.
    const float corner_guard = std::max(endpoint_clearance,
                                        FARUtil::kMatchDist * 0.5f);
    for (const auto& contour_node : contour_graph_) {
        if (!contour_node || contour_node->poly_ptr != best_polygon ||
            contour_node->is_boundary_clipped ||
            contour_node->free_direct == NodeFreeDirect::UNKNOW) {
            continue;
        }
        if ((contour_node->position - point).norm_flat() <= corner_guard) {
            return false;
        }
    }
    if (matched_polygon) *matched_polygon = best_polygon;
    return true;
}

EdgeValidationResult ContourGraph::ValidateContourFollowEdge(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    EdgeValidationResult result;
    result.reason = EdgeRejectReason::NOT_CURRENT_ADJACENT;
    if (!node_ptr1 || !node_ptr2 || node_ptr1 == node_ptr2 ||
        !node_ptr1->is_contour_match || !node_ptr2->is_contour_match ||
        !node_ptr1->ctnode || !node_ptr2->ctnode ||
        node_ptr1->ctnode->poly_ptr != node_ptr2->ctnode->poly_ptr ||
        node_ptr1->ctnode->source != node_ptr2->ctnode->source) {
        return result;
    }

    if (!ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2)) {
        return result;
    }

    const CTNodePtr ct1 = node_ptr1->ctnode;
    const CTNodePtr ct2 = node_ptr2->ctnode;
    const PolygonPtr endpoint_poly = ct1->poly_ptr;
    const bool static_structure =
        ct1->source != GraphNodeSource::DYNAMIC_LOCAL;
    EdgeRejectReason last_reason = EdgeRejectReason::OFFSET_FAILED;

    for (float projection = contour_projection_min_;
         projection <= contour_projection_max_ + FARUtil::kEpsilon;
         projection += contour_projection_step_) {
        Point3D route_start = ct1->position;
        Point3D route_end = ct2->position;
        const cv::Point2f projected_start = ProjectNode(ct1, projection);
        const cv::Point2f projected_end = ProjectNode(ct2, projection);
        route_start.x = projected_start.x;
        route_start.y = projected_start.y;
        route_end.x = projected_end.x;
        route_end.y = projected_end.y;
        const ConnectPair route(route_start, route_end);
        const HeightPair route_height(route_start, route_end);

        // ProjectNode() selects the CT vertex's free-space direction. Verify
        // that the complete candidate segment still lies on the robot's side
        // of its owning polygon before testing unrelated obstacles.
        const Point3D route_center(
            (route_start.x + route_end.x) * 0.5f,
            (route_start.y + route_end.y) * 0.5f,
            (route_start.z + route_end.z) * 0.5f);
        // A boundary-clipped OpenCV polygon contains an artificial closing
        // segment at the raster edge. Do not treat that synthetic cap as the
        // endpoint obstacle itself. Persistent static voxels below remain
        // authoritative, as do every other static/dynamic polygon.
        if (endpoint_poly && !endpoint_poly->is_pillar &&
            !endpoint_poly->is_boundary_clipped &&
            (endpoint_poly->is_robot_inside !=
                 FARUtil::PointInsideAPoly(endpoint_poly->vertices,
                                           route_center) ||
             IsEdgeCollidePoly(endpoint_poly->vertices, route))) {
            last_reason = EdgeRejectReason::SELF_POLYGON_BLOCKED;
            continue;
        }

        if (!IsEdgeCollisionFreeInCloud(
                route, route_height, local_static_collision_cloud_,
                local_static_collision_kdtree_, 0.0f)) {
            last_reason = EdgeRejectReason::STATIC_CLOUD_BLOCKED;
            continue;
        }
        if (!IsPointsConnectFreePolygonForLayer(
                route, route, route_height, false,
                CollisionLayer::STATIC_ONLY, endpoint_poly, endpoint_poly,
                false)) {
            last_reason = EdgeRejectReason::OTHER_STATIC_BLOCKED;
            continue;
        }

        // A dynamic contour is structural in this snapshot, so its own
        // free-side route must clear the complete current dynamic layer. A
        // persistent static contour is instead retained and dynamically
        // masked below without changing its static geometry.
        if (!static_structure) {
            if (!IsEdgeCollisionFreeInCloud(
                    route, route_height, local_dynamic_collision_cloud_,
                    local_dynamic_collision_kdtree_, 0.0f)) {
                last_reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
                continue;
            }
            if (!IsPointsConnectFreePolygonForLayer(
                    route, route, route_height, false,
                    CollisionLayer::DYNAMIC_ONLY, endpoint_poly,
                    endpoint_poly, false)) {
                last_reason = EdgeRejectReason::POLYGON_BLOCKED;
                continue;
            }
        }

        result.valid = true;
        result.reason = EdgeRejectReason::NONE;
        result.route_start = route_start;
        result.route_end = route_end;
        result.projection_distance = projection;
        result.route_cost =
            (node_ptr1->position - route_start).norm() +
            (route_start - route_end).norm() +
            (route_end - node_ptr2->position).norm();

        if (static_structure) {
            result.dynamic_blocked =
                !IsRouteConnectFreeDynamicLayer(route_start, route_end);
            if (result.dynamic_blocked) {
                result.reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
            }
        }
        return result;
    }

    result.reason = last_reason;
    return result;
}

bool ContourGraph::IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2) {
    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(p1, p2);
    const ConnectPair ori_cedge(p1, p2);
    const ConnectPair cedge = ori_cedge;
    const HeightPair h_pair(p1, p2);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, ori_cedge, h_pair, is_global_check);
}

bool ContourGraph::IsEdgeCollideBoundary(const Point3D& p1, const Point3D& p2) {
    if (ContourGraph::boundary_contour_.empty()) return false;
    const ConnectPair edge = ConnectPair(p1, p2);
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (ContourGraph::IsEdgeCollideSegment(contour, edge)) {return true;}
    }
    return false;
}

bool ContourGraph::IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    return ValidateGoalEdgeGeometry(node_ptr, goal_ptr) ==
        EdgeRejectReason::NONE;
}


bool ContourGraph::IsCTMatchLineFreePolygon(const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check) {
    if ((matched_ctnode->position - matched_navnode->position).norm() < FARUtil::kNavClearDist) return true;
    const HeightPair h_pair(matched_ctnode->position, matched_navnode->position);
    const ConnectPair bd_cedge = ConnectPair(matched_ctnode->position, matched_navnode->position);
    const ConnectPair cedge = ContourGraph::ReprojectEdge(matched_ctnode, matched_navnode, FARUtil::kProjectDist);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
}

bool ContourGraph::IsPillarConnectBlocked(const PolygonPtr& poly_ptr,
                                          const ConnectPair& edge,
                                          const HeightPair& edge_height) {
    if (!poly_ptr || poly_ptr->vertices.empty()) return false;

    float min_height = FARUtil::kINF;
    float max_height = -FARUtil::kINF;
    for (const auto& vertex : poly_ptr->vertices) {
        min_height = std::min(min_height, vertex.z);
        max_height = std::max(max_height, vertex.z);
    }
    if (!ContourGraph::IsEdgeOverlapInHeight(
            edge_height, HeightPair(min_height, max_height))) {
        return false;
    }

    const PointPair edge_line(
        Point3D(edge.start_p.x, edge.start_p.y, 0.0f),
        Point3D(edge.end_p.x, edge.end_p.y, 0.0f));
    const Point3D center = FARUtil::AveragePoints(poly_ptr->vertices);
    if (FARUtil::DistanceToLineSeg2D(center, edge_line) <=
        FARUtil::kNavClearDist) {
        return true;
    }
    for (const auto& vertex : poly_ptr->vertices) {
        if (FARUtil::DistanceToLineSeg2D(vertex, edge_line) <=
            FARUtil::kNavClearDist) {
            return true;
        }
    }
    return false;
}

bool ContourGraph::IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                              const ConnectPair& bd_cedge,
                                              const HeightPair h_pair,
                                              const bool& is_global_check)
{
    return ContourGraph::IsPointsConnectFreePolygonForLayer(
        cedge, bd_cedge, h_pair, is_global_check,
        CollisionLayer::COMBINED);
}

bool ContourGraph::IsPointsConnectFreePolygonForLayer(
    const ConnectPair& cedge, const ConnectPair& bd_cedge,
    const HeightPair h_pair, const bool& is_global_check,
    const CollisionLayer layer, const PolygonPtr& endpoint_poly1,
    const PolygonPtr& endpoint_poly2, const bool check_raw_cloud) {
    // Edge checks use only the latest already-cropped local layers.  The
    // complete OctoMap is queried only by the node lifecycle evidence path,
    // never once per candidate edge.
    if (check_raw_cloud && layer == CollisionLayer::COMBINED) {
        if (!ContourGraph::IsEdgeCollisionFreeInLocalCloud(cedge, h_pair)) {
            return false;
        }
    } else if (check_raw_cloud && layer == CollisionLayer::STATIC_ONLY) {
        if (!ContourGraph::IsEdgeCollisionFreeInCloud(
                cedge, h_pair, local_static_collision_cloud_,
                local_static_collision_kdtree_)) return false;
    } else if (check_raw_cloud && !ContourGraph::IsEdgeCollisionFreeInCloud(
                   cedge, h_pair, local_dynamic_collision_cloud_,
                   local_dynamic_collision_kdtree_)) {
        return false;
    }

    const bool include_static = layer != CollisionLayer::DYNAMIC_ONLY;
    const bool include_dynamic = layer != CollisionLayer::STATIC_ONLY;
    // check for boundaries edges
    if (include_static) {
        for (const auto& contour : ContourGraph::boundary_contour_) {
            if (!ContourGraph::IsEdgeOverlapInHeight(
                    h_pair, HeightPair(contour.first, contour.second))) continue;
            if (ContourGraph::IsEdgeCollideSegment(contour, bd_cedge)) {
                return false;
            }
        }
    }
    const auto includes_polygon = [include_static, include_dynamic](
        const PolygonPtr& poly_ptr) {
        if (!poly_ptr) return false;
        if (poly_ptr->source == GraphNodeSource::DYNAMIC_LOCAL) {
            return include_dynamic;
        }
        return include_static;
    };
    if (!is_global_check) {
        // check for local range polygons
        const Point3D center_p = Point3D((cedge.start_p.x + cedge.end_p.x) / 2.0f,
                                         (cedge.start_p.y + cedge.end_p.y) / 2.0f,
                                         0.0f);
        for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
            if (!includes_polygon(poly_ptr)) continue;
            // A boundary-clipped contour is physically open even though
            // OpenCV represents it as a closed polygon.  Its raw semantic
            // voxels remain authoritative for collision; ignore the
            // artificial polygon interior/closing segment.
            if (poly_ptr->is_boundary_clipped) continue;
            const bool is_endpoint_polygon =
                poly_ptr == endpoint_poly1 || poly_ptr == endpoint_poly2;
            if (poly_ptr->is_pillar) {
                // A pillar used as an edge endpoint is a routing landmark, not
                // an obstacle in the interior of its own incident edge.
                if (is_endpoint_polygon) continue;
                if (ContourGraph::IsPillarConnectBlocked(
                        poly_ptr, cedge, h_pair)) return false;
                continue;
            }
            if ((poly_ptr->is_robot_inside != FARUtil::PointInsideAPoly(poly_ptr->vertices, center_p)) || 
                ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) 
            {
                return false;
            }
        }
        // check for unmatched local contours
        if (include_static) {
            for (const auto& contour : ContourGraph::unmatched_contour_) {
                if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                    return false;
                }
            }
            // check for any inactive local contours
            for (const auto& contour : ContourGraph::inactive_contour_) {
                if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                    return false;
                }
            }
        }
    } else {
        if (include_static) {
            for (const auto& contour : ContourGraph::global_contour_) {
                if (!ContourGraph::IsEdgeOverlapInHeight(
                        h_pair, HeightPair(contour.first, contour.second))) continue;
                if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                    return false;
                }
            }
        }
        for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
            if (!includes_polygon(poly_ptr)) continue;
            if (poly_ptr->is_boundary_clipped) continue;
            const bool is_endpoint_polygon =
                poly_ptr == endpoint_poly1 || poly_ptr == endpoint_poly2;
            if (poly_ptr->is_pillar) {
                if (is_endpoint_polygon) continue;
                if (ContourGraph::IsPillarConnectBlocked(
                        poly_ptr, cedge, h_pair)) return false;
                continue;
            }
            if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
                return false;
            }
        }
    }
    return true;
}

bool ContourGraph::IsEdgeCollisionFreeInLocalCloud(
    const ConnectPair& edge, const HeightPair& edge_height) {
    return ContourGraph::IsEdgeCollisionFreeInCloud(
        edge, edge_height, local_collision_cloud_, local_collision_kdtree_);
}

bool ContourGraph::IsEdgeCollisionFreeInCloud(
    const ConnectPair& edge, const HeightPair& edge_height,
    const PointCloudPtr& cloud, const PointKdTreePtr& kdtree,
    const float endpoint_exclusion) {
    if (!cloud || cloud->empty() || !kdtree || !kdtree->getInputCloud()) {
        return true;
    }
    const float dx = edge.end_p.x - edge.start_p.x;
    const float dy = edge.end_p.y - edge.start_p.y;
    const float length = std::hypot(dx, dy);
    if (length < FARUtil::kEpsilon) return true;

    const float step = std::max(FARUtil::kLeafSize * 0.75f, 0.05f);
    const float requested_radius = std::max(FARUtil::kLeafSize * 0.75f,
                                            FARUtil::kNavClearDist);
    // The samples approximate a continuous swept segment.  Inflate each
    // sample sphere by half a step in quadrature so their union guarantees
    // the requested perpendicular clearance even midway between samples.
    const float radius = std::hypot(requested_radius, step * 0.5f);
    // The search ball, not only its centre, must stay outside the endpoints.
    // Otherwise points belonging to the target contour are reported as an
    // obstacle of the edge that intentionally terminates at that contour.
    const float endpoint_margin = endpoint_exclusion >= 0.0f
        ? std::min(endpoint_exclusion, length * 0.45f)
        : std::min(length * 0.45f,
                   radius + std::max(FARUtil::kLeafSize, step));
    const float mid_z = (edge_height.minH + edge_height.maxH) * 0.5f;
    for (float distance = endpoint_margin; distance <= length - endpoint_margin;
         distance += step) {
        const float ratio = distance / length;
        PCLPoint sample;
        sample.x = edge.start_p.x + dx * ratio;
        sample.y = edge.start_p.y + dy * ratio;
        sample.z = mid_z;
        sample.intensity = 0.0f;
        std::vector<int> indices;
        std::vector<float> squared_distances;
        if (kdtree->radiusSearch(
                sample, radius, indices, squared_distances, 1) > 0) {
            return false;
        }
    }
    // A fixed step does not normally land exactly on the far end.  For the
    // zero-exclusion routes used by odom, goal and contour-follow edges that
    // omission could leave the final fraction of an otherwise blocked edge
    // unchecked.  Sample the far checked endpoint explicitly (repeating an
    // exact sample is harmless).
    const float far_distance = length - endpoint_margin;
    if (far_distance >= endpoint_margin) {
        const float ratio = far_distance / length;
        PCLPoint sample;
        sample.x = edge.start_p.x + dx * ratio;
        sample.y = edge.start_p.y + dy * ratio;
        sample.z = mid_z;
        sample.intensity = 0.0f;
        std::vector<int> indices;
        std::vector<float> squared_distances;
        if (kdtree->radiusSearch(
                sample, radius, indices, squared_distances, 1) > 0) {
            return false;
        }
    }
    return true;
}

bool ContourGraph::IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_odom || node_ptr2->is_odom) return false;
    const CTNodePtr ctnode1 = node_ptr1->ctnode;
    const CTNodePtr ctnode2 = node_ptr2->ctnode;
    if (ctnode1 == NULL || ctnode2 == NULL || ctnode1 == ctnode2) return false;
    return ContourGraph::IsCTNodesConnectFromContour(ctnode1, ctnode2);
}

bool ContourGraph::IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    // Preserve FAR's incremental exploration behaviour: the endpoint created
    // where the current raster cuts a wall may temporarily close the OpenCV
    // contour and provide a route around the currently visible wall end. The
    // corresponding NavNode is explicitly transient, and every generated
    // route still has to pass the full persistent-static and current-dynamic
    // cloud corridor checks below.
    // check for boundary collision
    const ConnectPair cedge = ConnectPair(ctnode1->position, ctnode2->position);
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
            return false;
        }
    }
    // forward search
    CTNodePtr next_ctnode = ctnode1->front; 
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) 
        {
            break;
        } else {
            next_ctnode = next_ctnode->front;
        }
    }
    // backward search
    next_ctnode = ctnode1->back;
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) 
        {
            break;
        } else {
            next_ctnode = next_ctnode->back;
        }
    }
    return false;
}

CTNodePtr ContourGraph::FirstMatchedCTNode(const CTNodePtr& ctnode_ptr) {
    if (ctnode_ptr->is_global_match) return ctnode_ptr;
    CTNodePtr cur_ctnode_ptr = ctnode_ptr->front;
    while (cur_ctnode_ptr != ctnode_ptr) {
        if (cur_ctnode_ptr->is_global_match) return cur_ctnode_ptr;
        cur_ctnode_ptr = cur_ctnode_ptr->front;
    }
    return NULL;
}

NavNodePtr ContourGraph::MatchOutrangeNodeWithCTNode(const NavNodePtr& out_node_ptr, const NodePtrStack& near_nodes) {
    if (near_nodes.empty()) return NULL;
    float min_dist = FARUtil::kINF;
    NavNodePtr min_matched_node = NULL;
    for (const auto& node_ptr : near_nodes) {
        if (!node_ptr->is_contour_match) continue;
        CTNodePtr matched_ctnode = NULL;
        if (IsContourLineMatch(node_ptr, out_node_ptr, matched_ctnode)) {
            const float dist = FARUtil::VerticalDistToLine2D(node_ptr->position, matched_ctnode->position, out_node_ptr->position);
            if (dist < min_dist) {
                min_dist = dist;
                min_matched_node = node_ptr;
            }
        }
    }
    if (min_dist < FARUtil::kNavClearDist) {
        return min_matched_node;
    }
    return NULL;
}

bool ContourGraph::IsContourLineMatch(const NavNodePtr& inNode_ptr, const NavNodePtr& outNode_ptr, CTNodePtr& matched_ctnode) {
    const CTNodePtr ctnode_ptr = inNode_ptr->ctnode;
    matched_ctnode = NULL;
    if (ctnode_ptr == NULL || ctnode_ptr->poly_ptr->is_pillar) return false;
    // check forward
    const PointPair line1(inNode_ptr->position, outNode_ptr->position);
    CTNodePtr next_ctnode = ctnode_ptr->front;
    CTNodePtr prev_ctnode = ctnode_ptr;
    while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
           FARUtil::IsInCylinder(ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) 
    {
        if (!FARUtil::IsPointInMarginRange(next_ctnode->position) &&
            (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) 
        {
            const PointPair line2(ctnode_ptr->position, next_ctnode->position);
            if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                    matched_ctnode = next_ctnode;
                    return true;
                }
            }
        }
        prev_ctnode = next_ctnode;
        next_ctnode = next_ctnode->front;
    }
    // check backward
    next_ctnode = ctnode_ptr->back;
    prev_ctnode = ctnode_ptr;
    while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
           FARUtil::IsInCylinder(ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) 
    {
        if (!FARUtil::IsPointInMarginRange(next_ctnode->position) && 
            (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) 
        {
            const PointPair line2(ctnode_ptr->position, next_ctnode->position);
            if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                    matched_ctnode = next_ctnode;
                    return true;
                }
            }
        }
        prev_ctnode = next_ctnode;
        next_ctnode = next_ctnode->back;
    }
    return false;
}

bool ContourGraph::IsCTNodesConnectWithinOrder(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2, CTNodePtr& block_vertex) {
    block_vertex = NULL;
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    CTNodePtr next_ctnode = ctnode1->front; // forward search
    while (next_ctnode != NULL && next_ctnode != ctnode2) {
        if (!FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) {
            block_vertex = next_ctnode;
            return false;
        }
        next_ctnode = next_ctnode->front;
    }
    return true;
}

void ContourGraph::EnclosePolygonsCheck() {
    for (const auto& ctnode_ptr : ContourGraph::polys_ctnodes_) { // loop each polygon
        if (ctnode_ptr->poly_ptr->is_pillar) continue;
        const CTNodePtr start_ctnode_ptr = FirstMatchedCTNode(ctnode_ptr);
        if (start_ctnode_ptr == NULL) continue;
        CTNodePtr pre_ctnode_ptr = start_ctnode_ptr;
        CTNodePtr cur_ctnode_ptr = start_ctnode_ptr->front;
        while (cur_ctnode_ptr != start_ctnode_ptr) {
            if (!cur_ctnode_ptr->is_global_match) {
                cur_ctnode_ptr = cur_ctnode_ptr->front;
                continue;
            }
            CTNodePtr block_vertex = NULL;
            if (!IsCTNodesConnectWithinOrder(pre_ctnode_ptr, cur_ctnode_ptr, block_vertex) && block_vertex != NULL) {
                if (block_vertex->is_ground_associate && FARUtil::IsPointInMarginRange(block_vertex->position)) {
                    block_vertex->is_contour_necessary = true;
                }
            }
            pre_ctnode_ptr = cur_ctnode_ptr;
            cur_ctnode_ptr = cur_ctnode_ptr->front;
        }
    }
}

void ContourGraph::CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar) {
    ctnode_ptr = std::make_shared<CTNode>();
    ctnode_ptr->position = pos;
    ctnode_ptr->front = NULL;
    ctnode_ptr->back  = NULL;
    ctnode_ptr->is_global_match = false;
    ctnode_ptr->is_contour_necessary = false;
    ctnode_ptr->is_ground_associate = false;
    ctnode_ptr->nav_node_id = 0;
    ctnode_ptr->poly_ptr = poly_ptr;
    ctnode_ptr->source = poly_ptr ? poly_ptr->source : GraphNodeSource::UNKNOWN;
    ctnode_ptr->free_direct = is_pillar ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
    ctnode_ptr->connect_nodes.clear();
}

void ContourGraph::CreatePolygon(const PointStack& poly_points,
                                 PolygonPtr& poly_ptr,
                                 const GraphNodeSource source) {
    poly_ptr = std::make_shared<Polygon>();
    poly_ptr->N = poly_points.size();
    poly_ptr->vertices = poly_points;
    poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_points, odom_node_ptr_->position);
    float perimeter = 0.0f;
    poly_ptr->is_pillar = this->IsAPillarPolygon(poly_points, perimeter);
    poly_ptr->is_boundary_clipped = false;
    for (const auto& point : poly_points) {
        if (!IsPointInsideReliableContourWindow(point)) {
            poly_ptr->is_boundary_clipped = true;
            break;
        }
    }
    poly_ptr->perimeter = perimeter;
    poly_ptr->source = source;
}

NavNodePtr ContourGraph::NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes) {
    float nearest_dist = FARUtil::kINF;
    NavNodePtr nearest_node = NULL;
    float min_edist = FARUtil::kINF;
    const float dir_thred = 0.5f; //cos(pi/3);
    for (const auto& node_ptr : near_nodes) {
        if (node_ptr->is_odom || node_ptr->is_navpoint || FARUtil::IsOutsideGoal(node_ptr) || !IsInMatchHeight(ctnode_ptr, node_ptr)) continue;
        const bool static_contour =
            ctnode_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
            ctnode_ptr->source == GraphNodeSource::STATIC_GLOBAL;
        const bool static_node =
            node_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
            node_ptr->source == GraphNodeSource::STATIC_GLOBAL;
        const bool dynamic_contour =
            ctnode_ptr->source == GraphNodeSource::DYNAMIC_LOCAL;
        const bool dynamic_node =
            node_ptr->source == GraphNodeSource::DYNAMIC_LOCAL;
        if ((static_contour && !static_node) ||
            (dynamic_contour && !dynamic_node)) continue;
        // no match with pillar to non-pillar local vertices
        if ((node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) ||
            (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::PILLAR)) 
        {
            continue;
        }
        float dist_thred = FARUtil::kMatchDist;
        float dir_score = 0.0f;
        if (dynamic_contour && dynamic_node) {
            // Current dynamic corners may move and change slightly with voxel
            // quantisation. Source/type checks above plus nearest one-to-one
            // assignment make the full match radius safe and stable.
            dir_score = 1.0f;
        } else if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::UNKNOW && node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            if (ctnode_ptr->free_direct == node_ptr->free_direct) {
                const Point3D topo_dir1 = FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
                const Point3D topo_dir2 = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs);
                dir_score = (topo_dir1 * topo_dir2 - dir_thred) / (1.0f - dir_thred);
            }
        } else if (node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct == NodeFreeDirect::PILLAR) {
            dir_score = 0.5f;
        }
        dist_thred *= dir_score;
        const float edist = (node_ptr->position - ctnode_ptr->position).norm_flat();
        if (edist < dist_thred && edist < min_edist) {
            nearest_node = node_ptr;
            min_edist = edist;
        }
    }
    if (nearest_node != NULL && nearest_node->is_contour_match) {
        const float pre_dist = (nearest_node->position - nearest_node->ctnode->position).norm_flat();
        if (min_edist < pre_dist) {
            // reset matching for previous ctnode
            RemoveMatchWithNavNode(nearest_node);
        } else {
            nearest_node = NULL;
        }
    }
    return nearest_node;
}

void ContourGraph::AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph) {
    for (const auto& ctnode_ptr : contour_graph) {
        if (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR || ctnode_ptr->poly_ptr->is_pillar) {
            ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
            ctnode_ptr->poly_ptr->is_pillar = true;
            ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        } else {
            CTNodePtr next_ctnode;
            // front direction
            next_ctnode = ctnode_ptr->front;
            Point3D start_p = ctnode_ptr->position;
            Point3D end_p = next_ctnode->position;
            float edist = (end_p - ctnode_ptr->position).norm_flat();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < FARUtil::kNavClearDist) {
                next_ctnode = next_ctnode->front;
                start_p = end_p;
                end_p = next_ctnode->position;
                edist = (end_p - ctnode_ptr->position).norm_flat();
            }
            if (edist < FARUtil::kNavClearDist) { // This Node should be a pillar.
                ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
                ctnode_ptr->poly_ptr->is_pillar = true;
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                continue;
            } else {
                ctnode_ptr->surf_dirs.first = FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
            }
            // back direction
            next_ctnode = ctnode_ptr->back;
            start_p = ctnode_ptr->position;
            end_p   = next_ctnode->position;
            edist = (end_p - ctnode_ptr->position).norm_flat();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < FARUtil::kNavClearDist) {
                next_ctnode = next_ctnode->back;
                start_p = end_p;
                end_p = next_ctnode->position;
                edist = (end_p - ctnode_ptr->position).norm_flat();
            }
            if (edist < FARUtil::kNavClearDist) { // This Node should be a pillar.
                ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)}; // TODO!
                ctnode_ptr->poly_ptr->is_pillar = true;
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                continue;
            } else {
                ctnode_ptr->surf_dirs.second = FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
            }
        }
        // analysis convexity (except pillar)
        this->AnalysisConvexityOfCTNode(ctnode_ptr);
    }
}

bool ContourGraph::IsAPillarPolygon(const PointStack& vertex_points, float& perimeter) {
    perimeter = 0.0f;
    if (vertex_points.size() < 3) return true;
    Point3D prev_p(vertex_points[0]);
    for (std::size_t i=1; i<vertex_points.size(); i++) {
        const Point3D cur_p(vertex_points[i]);
        const float dist = std::hypotf(cur_p.x - prev_p.x, cur_p.y - prev_p.y);
        perimeter += dist;
        prev_p = cur_p;
    }
    return perimeter > ctgraph_params_.kPillarPerimeter ? false : true;
}

bool ContourGraph::IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge) {
    const cv::Point2f start_p(line.first.x, line.first.y);
    const cv::Point2f end_p(line.second.x, line.second.y);
    if (POLYOPS::doIntersect(start_p, end_p, edge.start_p, edge.end_p)) {
        return true;
    }
    return false;
}

bool ContourGraph::IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge) {
    const int N = poly.size();
    if (N < 3) cout<<"Poly vertex size less than 3."<<endl;
    for (int i=0; i<N; i++) {
        const PointPair line(poly[i], poly[FARUtil::Mod(i+1, N)]);
        if (ContourGraph::IsEdgeCollideSegment(line, edge)) {
            return true;
        }
    }
    return false;
}

void ContourGraph::AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr) {
    if (ctnode_ptr->surf_dirs.first  == Point3D(0,0,-1) || ctnode_ptr->surf_dirs.second == Point3D(0,0,-1) || ctnode_ptr->poly_ptr->is_pillar) {
        ctnode_ptr->surf_dirs.first = Point3D(0,0,-1), ctnode_ptr->surf_dirs.second == Point3D(0,0,-1);
        ctnode_ptr->poly_ptr->is_pillar = true;
        ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        return;
    }
    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs, is_wall);
    if (is_wall) {
        ctnode_ptr->free_direct = NodeFreeDirect::UNKNOW;
        return;
    }
    const Point3D ev_p = ctnode_ptr->position + topo_dir * FARUtil::kLeafSize;
    if (FARUtil::IsConvexPoint(ctnode_ptr->poly_ptr, ev_p)) {
        ctnode_ptr->free_direct = NodeFreeDirect::CONVEX;
    } else {
        ctnode_ptr->free_direct = NodeFreeDirect::CONCAVE;
    }
}

bool ContourGraph::ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius) {
    PolygonPtr inside_poly_ptr = NULL;
    bool is_inside_poly = false;
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        if (poly_ptr->is_pillar) continue;
        if (FARUtil::PointInsideAPoly(poly_ptr->vertices, point) && !FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p)) {
            inside_poly_ptr = poly_ptr;
            is_inside_poly = true;
            break;
        }
    }
    if (is_inside_poly) {
        float near_dist = FARUtil::kINF;
        Point3D reproject_p = point;
        Point3D free_dir(0,0,-1);
        const int N = inside_poly_ptr->vertices.size();
        for (int idx=0; idx<N; idx++) {
            const Point3D vertex = inside_poly_ptr->vertices[idx];
            const float temp_dist = (vertex - point).norm_flat();
            if (temp_dist < near_dist) {
                const Point3D dir1 = (inside_poly_ptr->vertices[FARUtil::Mod(idx-1, N)] - vertex).normalize_flat();
                const Point3D dir2 = (inside_poly_ptr->vertices[FARUtil::Mod(idx+1, N)] - vertex).normalize_flat();
                const Point3D dir = (dir1 + dir2).normalize_flat();
                if (FARUtil::PointInsideAPoly(inside_poly_ptr->vertices, vertex + dir * FARUtil::kLeafSize)) { // convex 
                    reproject_p = vertex;
                    near_dist = temp_dist;
                    free_dir = dir;
                }
            }
        }
        const float origin_z = point.z;
        point = reproject_p - free_dir * free_radius;
        point.z = origin_z;
    }
    return is_inside_poly;
}

void ContourGraph::AddContourToSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    NavEdge edge(node_ptr1, node_ptr2);
    // force to form pair id1 < id2
    if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);

    ContourGraph::global_contour_set_.insert(edge);
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        ContourGraph::boundary_contour_set_.insert(edge);
    }
}

void ContourGraph::DeleteContourFromSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    NavEdge edge(node_ptr1, node_ptr2);
    // force to form pair id1 < id2
    if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);

    ContourGraph::global_contour_set_.erase(edge);
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        ContourGraph::boundary_contour_set_.erase(edge);
    }
}

void ContourGraph::ExtractGlobalContours() {
    ContourGraph::global_contour_.clear();
    ContourGraph::inactive_contour_.clear();
    ContourGraph::unmatched_contour_.clear();
    ContourGraph::boundary_contour_.clear();
    ContourGraph::local_boundary_.clear();
    for (const auto& edge : ContourGraph::global_contour_set_) {
        ContourGraph::global_contour_.push_back({edge.first->position, edge.second->position});
        if (IsEdgeInLocalRange(edge.first, edge.second)) {
            if (!this->IsActiveEdge(edge.first, edge.second)) {
                ContourGraph::inactive_contour_.push_back({edge.first->position, edge.second->position});
            } else if (!edge.first->is_near_nodes || !edge.second->is_near_nodes) {
                PointPair unmatched_pair = std::make_pair(edge.first->position, edge.second->position);
                if (edge.first->is_contour_match) {
                    unmatched_pair.first = edge.first->ctnode->position;
                } else if (edge.second->is_contour_match) {
                    unmatched_pair.second = edge.second->ctnode->position;
                } 
                ContourGraph::unmatched_contour_.push_back(unmatched_pair);
            }
        }
    }
    for (const auto& edge : ContourGraph::boundary_contour_set_) {
        ContourGraph::boundary_contour_.push_back({edge.first->position, edge.second->position});
        if (IsEdgeInLocalRange(edge.first, edge.second)) {
            ContourGraph::local_boundary_.push_back({edge.first->position, edge.second->position});
            bool is_new_invalid = false;
            if (!IsValidBoundary(edge.first, edge.second, is_new_invalid) && is_new_invalid) {
                edge.first->invalid_boundary.insert(edge.second->id);
                edge.second->invalid_boundary.insert(edge.first->id);
            }
        }
    }
}

bool ContourGraph::IsValidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, bool& is_new) {
    is_new = true;
    if (node_ptr1->invalid_boundary.find(node_ptr2->id) != node_ptr1->invalid_boundary.end()) { // already invalid
        is_new = false;
        return false;
    }
    // check against local polygon
    const ConnectPair cedge = ConnectPair(node_ptr1->position, node_ptr2->position);
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        if (poly_ptr->is_pillar) continue;
        if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
            return false;
        }
    }
    return true;
}

void ContourGraph::UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p) {
    Point3D free_p = odom_ptr->position;
    bool is_free_p = true;
    PointStack free_sample_points;
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        if (!poly_ptr->is_pillar && poly_ptr->is_robot_inside) {
            is_free_p = false;
            FARUtil::CreatePointsAroundCenter(free_p, FARUtil::kNavClearDist, FARUtil::kLeafSize, free_sample_points);
            break;
        }
    }
    if (is_free_p) is_robot_inside_poly_ = false;
    global_free_p = free_p;
    if (!is_free_p && !is_robot_inside_poly_) {
        bool is_free_pos_found = false;
        for (const auto& p : free_sample_points) {
            bool is_sample_free = true;
            for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
                if (!poly_ptr->is_pillar && FARUtil::PointInsideAPoly(poly_ptr->vertices, p)) {
                    is_sample_free = false;
                    break;
                }
            }
            if (is_sample_free) {
                global_free_p = p;
                is_free_pos_found = true;
                break;
            }
        }
        if (!is_free_pos_found) is_robot_inside_poly_ = true;
    }
}

ConnectPair ContourGraph::ReprojectEdge(const CTNodePtr& ctnode_ptr1, const NavNodePtr& node_ptr2, const float& dist) {
    ConnectPair edgeOut;
    const float ndist = (ctnode_ptr1->position - node_ptr2->position).norm_flat();
    const float ref_dist = std::min(ndist*0.4f, dist);

    edgeOut.start_p = ProjectNode(ctnode_ptr1, ref_dist); // node 1
    edgeOut.end_p   = ProjectNode(node_ptr2, ref_dist);   // node 2

    return edgeOut;
}

ConnectPair ContourGraph::ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist, const bool& is_global_check) {
    ConnectPair edgeOut;
    const float ndist = (node_ptr1->position - node_ptr2->position).norm_flat();
    const float ref_dist = std::min(ndist*0.4f, dist);
    // node 1
    if (!is_global_check && node_ptr1->is_contour_match && node_ptr1->ctnode->free_direct == node_ptr1->free_direct) { // node 1
        const auto ctnode1 = node_ptr1->ctnode;
        edgeOut.start_p = ProjectNode(ctnode1, ref_dist); // node 1
    } else {
        edgeOut.start_p = ProjectNode(node_ptr1, ref_dist); // node 1
    }
    // node 2
    if (!is_global_check && node_ptr2->is_contour_match && node_ptr2->ctnode->free_direct == node_ptr2->free_direct) { // node 2
        const auto ctnode2 = node_ptr2->ctnode;
        edgeOut.end_p = ProjectNode(ctnode2, ref_dist); // node 1
    } else {
        edgeOut.end_p = ProjectNode(node_ptr2, ref_dist);
    }
    return edgeOut;
}

void ContourGraph::ResetCurrentContour() {
    this->ClearContourGraph();
    // clear contour sets
    ContourGraph::global_contour_set_.clear();
    ContourGraph::boundary_contour_set_.clear();

    odom_node_ptr_ = NULL;
    is_robot_inside_poly_ = false;
}   
