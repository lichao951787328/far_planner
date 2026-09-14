/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/contour_graph.h"
#include "far_planner/intersection.h"
#include "far_planner/terminal_visibility_policy.h"

#include <cstdint>
#include <limits>

PointCloudPtr ContourGraph::local_collision_cloud_(new PointCloud());
PointKdTreePtr ContourGraph::local_collision_kdtree_(
    new pcl::KdTreeFLANN<PCLPoint>());
PointCloudPtr ContourGraph::local_static_collision_cloud_(new PointCloud());
PointKdTreePtr ContourGraph::local_static_collision_kdtree_(
    new pcl::KdTreeFLANN<PCLPoint>());
PointCloudPtr ContourGraph::local_dynamic_collision_cloud_(new PointCloud());
PointKdTreePtr ContourGraph::local_dynamic_collision_kdtree_(
    new pcl::KdTreeFLANN<PCLPoint>());
ContourGraph::LocalCollisionGrid2D ContourGraph::local_static_collision_grid_;
ContourGraph::LocalCollisionGrid2D ContourGraph::local_dynamic_collision_grid_;
ContourGraph::LocalCollisionGrid2D ContourGraph::local_collision_grid_;
float ContourGraph::contour_projection_min_ = 0.15f;
float ContourGraph::contour_projection_step_ = 0.075f;
float ContourGraph::contour_projection_max_ = 0.60f;
float ContourGraph::contour_boundary_guard_ = 0.40f;
LocalObservationWindow2D ContourGraph::local_observation_window_;

/***************************************************************************************/

void ContourGraph::Init(const ContourGraphParams& params) {
    ctgraph_params_ = params;
    constexpr float kStaticIdentityHardCap = 0.60f;
    ctgraph_params_.static_match_tight_radius = std::min(
        kStaticIdentityHardCap,
        std::max(0.0f, params.static_match_tight_radius));
    ctgraph_params_.static_match_max_radius = std::min(
        kStaticIdentityHardCap,
        std::max(ctgraph_params_.static_match_tight_radius,
                 params.static_match_max_radius));
    ctgraph_params_.dynamic_match_max_radius = std::max(
        0.0f, params.dynamic_match_max_radius);
    contour_projection_min_ = std::max(0.0f, params.contour_projection_min);
    contour_projection_step_ = std::max(
        FARUtil::kEpsilon, params.contour_projection_step);
    contour_projection_max_ = std::max(
        contour_projection_min_, params.contour_projection_max);
    contour_boundary_guard_ = std::max(
        FARUtil::kLeafSize, params.contour_boundary_guard);
    local_observation_window_.enabled =
        params.use_local_observation_window;
    local_observation_window_.min_x = params.local_window_min_x;
    local_observation_window_.max_x = params.local_window_max_x;
    local_observation_window_.min_y = params.local_window_min_y;
    local_observation_window_.max_y = params.local_window_max_y;
    local_observation_window_.guard = contour_boundary_guard_;
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
    this->UpdateContourGraph(
        odom_node_ptr, static_contours, dynamic_contours,
        std::vector<PointStack>(), std::vector<PointStack>(),
        std::vector<std::vector<std::size_t>>(),
        std::vector<std::vector<std::size_t>>());
}

void ContourGraph::UpdateContourGraph(
    const NavNodePtr& odom_node_ptr,
    const std::vector<std::vector<Point3D>>& static_contours,
    const std::vector<std::vector<Point3D>>& dynamic_contours,
    const std::vector<std::vector<Point3D>>& static_dense_contours,
    const std::vector<std::vector<Point3D>>& dynamic_dense_contours,
    const std::vector<std::vector<std::size_t>>&
        static_simplified_dense_indices,
    const std::vector<std::vector<std::size_t>>&
        dynamic_simplified_dense_indices) {
    odom_node_ptr_ = odom_node_ptr;
    this->ClearContourGraph();
    const auto add_polygons = [this](
        const std::vector<std::vector<Point3D>>& contours,
        const std::vector<std::vector<Point3D>>& dense_contours,
        const std::vector<std::vector<std::size_t>>& correspondences,
        const GraphNodeSource source) {
      for (std::size_t index = 0; index < contours.size(); ++index) {
        const PointStack& poly = contours[index];
        const PointStack dense = index < dense_contours.size()
            ? dense_contours[index] : PointStack();
        const std::vector<std::size_t> correspondence =
            index < correspondences.size()
                ? correspondences[index]
                : std::vector<std::size_t>();
        PolygonPtr new_poly_ptr = NULL;
        this->CreatePolygon(poly, new_poly_ptr, source, dense,
                            correspondence);
        this->AddPolyToContourPolygon(new_poly_ptr);
      }
    };
    add_polygons(static_contours, static_dense_contours,
                 static_simplified_dense_indices,
                 GraphNodeSource::STATIC_CANDIDATE);
    add_polygons(dynamic_contours, dynamic_dense_contours,
                 dynamic_simplified_dense_indices,
                 GraphNodeSource::DYNAMIC_LOCAL);
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
                new_ctnode_ptr->contour_index = idx;
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
    for (const auto& node_ptr : ContourGraph::contour_graph_) {
        UpdateFreeSpaceDirection(node_ptr);
    }
}

void ContourGraph::SetLocalCollisionCloud(
    const PointCloudPtr& collision_cloud) {
    SetLocalCollisionCloud(collision_cloud, PointCloudPtr(new PointCloud()));
}

void ContourGraph::SetLocalCollisionCloud(
    const PointCloudPtr& static_cloud, const PointCloudPtr& dynamic_cloud) {
    // A cloud update starts a new snapshot.  FAR supplies the matching grids
    // after contour extraction; direct unit-test/legacy callers intentionally
    // remain on the raw-cloud fallback.
    local_static_collision_grid_.Clear();
    local_dynamic_collision_grid_.Clear();
    local_collision_grid_.Clear();
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

void ContourGraph::SetLocalCollisionGrids(
    const cv::Mat& static_occupied, const cv::Mat& dynamic_occupied,
    const Point3D& raster_center, const float resolution) {
    const auto assign = [&raster_center, resolution](
        const cv::Mat& input, LocalCollisionGrid2D& output) {
        output.Clear();
        if (input.empty() || input.type() != CV_8UC1 ||
            resolution <= FARUtil::kEpsilon) return;
        output.occupied = input.clone();
        output.center = raster_center;
        output.resolution = resolution;
        output.center_row = static_cast<float>(input.rows / 2);
        output.center_col = static_cast<float>(input.cols / 2);
    };
    assign(static_occupied, local_static_collision_grid_);
    assign(dynamic_occupied, local_dynamic_collision_grid_);

    local_collision_grid_.Clear();
    if (local_static_collision_grid_.IsValid()) {
        local_collision_grid_ = local_static_collision_grid_;
        local_collision_grid_.occupied =
            local_static_collision_grid_.occupied.clone();
        if (local_dynamic_collision_grid_.IsValid() &&
            local_dynamic_collision_grid_.occupied.size() ==
                local_collision_grid_.occupied.size()) {
            cv::bitwise_or(local_collision_grid_.occupied,
                           local_dynamic_collision_grid_.occupied,
                           local_collision_grid_.occupied);
        }
    } else if (local_dynamic_collision_grid_.IsValid()) {
        local_collision_grid_ = local_dynamic_collision_grid_;
        local_collision_grid_.occupied =
            local_dynamic_collision_grid_.occupied.clone();
    }
}

ContourGraph::GridCellState ContourGraph::GridStateAtPoint(
    const Point3D& point, const LocalCollisionGrid2D& grid) {
    if (!grid.IsValid()) return GridCellState::OUTSIDE;
    const int row = static_cast<int>(std::floor(
        grid.center_row + (point.x - grid.center.x) /
            grid.resolution + 0.5f));
    const int col = static_cast<int>(std::floor(
        grid.center_col + (point.y - grid.center.y) /
            grid.resolution + 0.5f));
    if (row < 0 || row >= grid.occupied.rows ||
        col < 0 || col >= grid.occupied.cols) {
        return GridCellState::OUTSIDE;
    }
    return grid.occupied.at<std::uint8_t>(row, col) == 0
        ? GridCellState::FREE : GridCellState::OCCUPIED;
}

void ContourGraph::UpdateFreeSpaceDirection(const CTNodePtr& node_ptr) {
    if (!node_ptr) return;
    node_ptr->free_space_dir = Point3D(0.0f, 0.0f, 0.0f);
    node_ptr->is_free_space_dir_reliable = false;
    if (node_ptr->free_direct != NodeFreeDirect::CONVEX &&
        node_ptr->free_direct != NodeFreeDirect::CONCAVE) {
        return;
    }

    Point3D bisector = FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
    if (bisector.norm_flat() <= FARUtil::kEpsilon) return;
    Point3D candidate = node_ptr->free_direct == NodeFreeDirect::CONVEX
        ? -bisector : bisector;
    candidate = candidate.normalize_flat();
    node_ptr->free_space_dir = candidate;

    const LocalCollisionGrid2D& grid =
        node_ptr->source == GraphNodeSource::DYNAMIC_LOCAL
            ? local_dynamic_collision_grid_ : local_static_collision_grid_;
    if (!grid.IsValid()) return;

    if (node_ptr->is_boundary_clipped) {
        // One neighbour of a cropped endpoint may be OpenCV's synthetic
        // closing cap.  Never let that segment participate in the corner
        // bisector used for motion.  Derive a local normal solely from an
        // observed contour tangent, then let the configuration-space grid
        // and the current-window gate select its free side.
        CTNodePtr physical_neighbor;
        const auto consider_neighbor = [&node_ptr, &physical_neighbor](
            const CTNodePtr& neighbor) {
            if (!neighbor || neighbor == node_ptr ||
                IsArtificialBoundaryClosingSegment(
                    node_ptr->position, neighbor->position)) {
                return;
            }
            if (!physical_neighbor ||
                (neighbor->position - node_ptr->position).norm_flat() >
                    (physical_neighbor->position -
                     node_ptr->position).norm_flat()) {
                physical_neighbor = neighbor;
            }
        };
        consider_neighbor(node_ptr->front);
        consider_neighbor(node_ptr->back);
        if (!physical_neighbor) return;

        Point3D tangent =
            (node_ptr->position - physical_neighbor->position)
                .normalize_flat();
        if (tangent.norm_flat() <= FARUtil::kEpsilon) return;
        const Point3D normals[2] = {
            Point3D(-tangent.y, tangent.x, 0.0f),
            Point3D(tangent.y, -tangent.x, 0.0f)};
        struct DirectionSupport {
            int free = 0;
            int occupied = 0;
            int inside = 0;
            int reliable_inside = 0;
        } support[2];
        const float base_sample = std::max(
            FARUtil::kNavClearDist + grid.resolution * 2.0f,
            grid.resolution * 3.0f);
        const float sample_step = std::max(grid.resolution * 2.0f, 0.05f);
        for (int direction_index = 0; direction_index < 2;
             ++direction_index) {
            for (int sample_index = 0; sample_index < 3; ++sample_index) {
                const float distance =
                    base_sample + sample_step * sample_index;
                const Point3D sample = node_ptr->position +
                    normals[direction_index] * distance;
                if (!IsPointInsideCurrentObservationWindow(sample)) continue;
                ++support[direction_index].inside;
                if (IsPointInsideReliableContourWindow(sample)) {
                    ++support[direction_index].reliable_inside;
                }
                const GridCellState state = GridStateAtPoint(sample, grid);
                if (state == GridCellState::FREE) {
                    ++support[direction_index].free;
                } else if (state == GridCellState::OCCUPIED) {
                    ++support[direction_index].occupied;
                }
            }
        }
        const auto score = [&support](const int index) {
            return support[index].free * 4 +
                   support[index].reliable_inside -
                   support[index].occupied * 5;
        };
        const int best = score(1) > score(0) ? 1 : 0;
        const int other = 1 - best;
        if (support[best].inside >= 2 && support[best].free >= 2 &&
            support[best].occupied == 0 && score(best) > score(other)) {
            node_ptr->free_space_dir = normals[best];
            node_ptr->is_free_space_dir_reliable = true;
        }
        return;
    }

    int forward_free = 0;
    int forward_occupied = 0;
    int reverse_free = 0;
    int reverse_occupied = 0;
    const float base_sample = std::max(
        FARUtil::kLeafSize * 0.75f, grid.resolution * 2.0f);
    for (int sample_index = 1; sample_index <= 3; ++sample_index) {
        const float distance = base_sample * sample_index;
        const Point3D forward = node_ptr->position + candidate * distance;
        const Point3D reverse = node_ptr->position - candidate * distance;
        // W_guard/outside is not positive free-space evidence.
        if (IsPointInsideReliableContourWindow(forward)) {
            const GridCellState state = GridStateAtPoint(forward, grid);
            if (state == GridCellState::FREE) ++forward_free;
            else if (state == GridCellState::OCCUPIED) ++forward_occupied;
        }
        if (IsPointInsideReliableContourWindow(reverse)) {
            const GridCellState state = GridStateAtPoint(reverse, grid);
            if (state == GridCellState::FREE) ++reverse_free;
            else if (state == GridCellState::OCCUPIED) ++reverse_occupied;
        }
    }

    const bool forward_supported =
        forward_free >= 2 && reverse_occupied >= 2;
    const bool reverse_supported =
        reverse_free >= 2 && forward_occupied >= 2;
    if (forward_supported && !reverse_supported) {
        node_ptr->is_free_space_dir_reliable = true;
    } else if (reverse_supported && !forward_supported) {
        node_ptr->free_space_dir = -candidate;
        node_ptr->is_free_space_dir_reliable = true;
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
    match_debug_records_.clear();
    duplicate_debug_records_.clear();
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
        std::size_t debug_record_index = 0;
    };
    std::vector<MatchCandidate> match_candidates;
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

            if (!dynamic_contour && !contour_pillar &&
                node_ptr->free_direct != ctnode_ptr->free_direct) {
                continue;
            }

            const bool directions_reliable =
                static_contour && !contour_pillar &&
                node_ptr->is_free_space_dir_reliable &&
                ctnode_ptr->is_free_space_dir_reliable;
            float free_direction_cosine = 0.0f;
            if (directions_reliable) {
                free_direction_cosine = std::max(
                    -1.0f, std::min(1.0f,
                        node_ptr->free_space_dir.norm_flat_dot(
                            ctnode_ptr->free_space_dir)));
            }
            const float match_radius = static_contour
                ? StaticCornerMatchRadius(
                    ctgraph_params_.static_match_tight_radius,
                    ctgraph_params_.static_match_max_radius,
                    directions_reliable, free_direction_cosine)
                : ctgraph_params_.dynamic_match_max_radius;
            const float distance =
                (node_ptr->position - ctnode_ptr->position).norm_flat();
            if (static_contour &&
                !AreStaticCornerFreeDirectionsCompatible(
                    directions_reliable, free_direction_cosine)) {
                if (debug_visualization_enabled_ &&
                    distance < ctgraph_params_.static_match_max_radius) {
                    ContourMatchDebugRecord record;
                    record.contour_position = ctnode_ptr->position;
                    record.graph_position = node_ptr->position;
                    record.graph_node_id = node_ptr->id;
                    record.distance = distance;
                    record.direction_angle_deg = std::acos(
                        free_direction_cosine) * 180.0f /
                        static_cast<float>(M_PI);
                    record.match_radius = match_radius;
                    record.score = distance;
                    record.free_direction_reliable = true;
                    record.outcome =
                        ContourMatchDebugOutcome::DIRECTION_REJECTED;
                    match_debug_records_.push_back(record);
                }
                continue;
            }
            if (distance >= match_radius) {
                continue;
            }
            const bool line_free =
                IsCTMatchLineFreePolygon(ctnode_ptr, node_ptr, false);
            float direction_angle_deg = 0.0f;
            if (directions_reliable) {
                direction_angle_deg = std::acos(free_direction_cosine) * 180.0f /
                    static_cast<float>(M_PI);
            }
            // Direction controls admission only.  Assignment order uses real
            // metric distance, so a larger permitted radius never makes a
            // farther candidate look artificially closer.
            const float score = distance;
            std::size_t debug_record_index = 0;
            if (debug_visualization_enabled_) {
                ContourMatchDebugRecord record;
                record.contour_position = ctnode_ptr->position;
                record.graph_position = node_ptr->position;
                record.graph_node_id = node_ptr->id;
                record.distance = distance;
                record.direction_angle_deg = direction_angle_deg;
                record.match_radius = match_radius;
                record.score = score;
                record.free_direction_reliable = directions_reliable;
                record.outcome = line_free
                    ? ContourMatchDebugOutcome::CANDIDATE
                    : ContourMatchDebugOutcome::LINE_BLOCKED;
                match_debug_records_.push_back(record);
                debug_record_index = match_debug_records_.size() - 1;
            }
            if (!line_free) continue;
            match_candidates.push_back(
                {ctnode_ptr, node_ptr, score, distance,
                 debug_record_index});
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
            if (debug_visualization_enabled_) {
                match_debug_records_[candidate.debug_record_index].outcome =
                    ContourMatchDebugOutcome::ONE_TO_ONE_LOST;
            }
            continue;
        }
        this->MatchCTNodeWithNavNode(candidate.contour_node,
                                     candidate.nav_node);
        assigned_nav_ids.insert(candidate.nav_node->id);
        assigned_contour_nodes.insert(candidate.contour_node.get());
        if (debug_visualization_enabled_) {
            match_debug_records_[candidate.debug_record_index].outcome =
                ContourMatchDebugOutcome::ACCEPTED;
        }
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
                ctnode_ptr->free_direct == NodeFreeDirect::CONCAVE &&
                !ctnode_ptr->is_contour_necessary) {
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
                        if (debug_visualization_enabled_) {
                            ContourDuplicateDebugRecord record;
                            record.suppressed_position = ctnode_ptr->position;
                            record.keeper_position = node_ptr->position;
                            record.keeper_node_id = node_ptr->id;
                            record.distance = (ctnode_ptr->position -
                                               node_ptr->position).norm_flat();
                            record.duplicate_radius = duplicate_radius;
                            record.keeper_is_historical = true;
                            duplicate_debug_records_.push_back(record);
                        }
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
                            if (debug_visualization_enabled_) {
                                ContourDuplicateDebugRecord record;
                                record.suppressed_position =
                                    ctnode_ptr->position;
                                record.keeper_position = accepted->position;
                                record.distance = (ctnode_ptr->position -
                                                   accepted->position)
                                                      .norm_flat();
                                record.duplicate_radius = duplicate_radius;
                                record.keeper_is_historical = false;
                                duplicate_debug_records_.push_back(record);
                            }
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

void ContourGraph::RecordHistoricalDuplicate(
    const NavNodePtr& obsolete, const NavNodePtr& keeper,
    const float duplicate_radius) {
    if (!debug_visualization_enabled_ || !obsolete || !keeper) return;
    ContourDuplicateDebugRecord record;
    record.suppressed_position = obsolete->position;
    record.keeper_position = keeper->position;
    record.suppressed_node_id = obsolete->id;
    record.keeper_node_id = keeper->id;
    record.distance =
        (obsolete->position - keeper->position).norm_flat();
    record.duplicate_radius = duplicate_radius;
    record.keeper_is_historical = true;
    record.is_history_consolidation = true;
    duplicate_debug_records_.push_back(record);
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
    const bool first_projected = std::hypot(
        edge.start_p.x - node_ptr1->position.x,
        edge.start_p.y - node_ptr1->position.y) > FARUtil::kEpsilon;
    const bool second_projected = std::hypot(
        edge.end_p.x - node_ptr2->position.x,
        edge.end_p.y - node_ptr2->position.y) > FARUtil::kEpsilon;

    // A projected endpoint is part of the robot-centre trajectory, not an
    // obstacle anchor. It must itself be free. The legacy endpoint exclusion
    // otherwise lets an unmatched historical corner such as N20 keep active
    // visibility edges even after its fixed 0.15 m projection has moved
    // inside the latest configuration-space obstacle.
    const Point3D projected_first(edge.start_p.x, edge.start_p.y,
                                  node_ptr1->position.z);
    const Point3D projected_second(edge.end_p.x, edge.end_p.y,
                                   node_ptr2->position.z);
    if ((first_projected &&
         !IsPointCollisionFreeStaticLayer(projected_first)) ||
        (second_projected &&
         !IsPointCollisionFreeStaticLayer(projected_second))) {
        return EdgeRejectReason::STATIC_CLOUD_BLOCKED;
    }
    if (include_dynamic &&
        ((first_projected &&
          !IsPointCollisionFreeDynamicLayer(projected_first)) ||
         (second_projected &&
          !IsPointCollisionFreeDynamicLayer(projected_second)))) {
        return EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
    }
    // When both obstacle corners have a real projection, validate the whole
    // stored route without an endpoint blind zone. Pillars/unknown directions
    // retain the small legacy exclusion because they cannot provide a
    // robot-centre endpoint distinct from the obstacle anchor.
    const float endpoint_exclusion =
        first_projected && second_projected ? 0.0f : -1.0f;
    if (!IsEdgeCollisionFreeInCloud(
            edge, height, local_static_collision_cloud_,
            local_static_collision_kdtree_, endpoint_exclusion)) {
        return EdgeRejectReason::STATIC_CLOUD_BLOCKED;
    }
    if (include_dynamic && !IsEdgeCollisionFreeInCloud(
            edge, height, local_dynamic_collision_cloud_,
            local_dynamic_collision_kdtree_, endpoint_exclusion)) {
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
    result.route_points = {result.route_start, result.route_end};
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
        invalid.route_points = {invalid.route_start, invalid.route_end};
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
                        route, height,
                        local_static_collision_grid_.IsValid()
                            ? 0.0f : FARUtil::kNavClearDist)) {
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
                attempt.route_points = {route_start, route_end};
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
            route, height,
            local_static_collision_grid_.IsValid()
                ? 0.0f : FARUtil::kNavClearDist)) {
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
    result.route_points = {route_start, route_end};
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

bool ContourGraph::IsRouteConnectFreeDynamicLayer(
    const std::vector<Point3D>& route_points) {
    if (route_points.size() < 2) return false;
    for (std::size_t index = 1; index < route_points.size(); ++index) {
        if (!IsRouteConnectFreeDynamicLayer(route_points[index - 1],
                                            route_points[index])) {
            return false;
        }
    }
    return true;
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

bool ContourGraph::IsRouteConnectFreeStaticLayer(
    const std::vector<Point3D>& route_points) {
    if (route_points.size() < 2) return false;
    for (std::size_t index = 1; index < route_points.size(); ++index) {
        if (!IsRouteConnectFreeStaticLayer(route_points[index - 1],
                                           route_points[index])) {
            return false;
        }
    }
    return true;
}

bool ContourGraph::IsPointCollisionFreeStaticLayer(const Point3D& point) {
    return IsPointCollisionFreeInCloud(
        point, local_static_collision_cloud_, local_static_collision_kdtree_);
}

bool ContourGraph::IsPointCollisionFreeDynamicLayer(const Point3D& point) {
    return IsPointCollisionFreeInCloud(
        point, local_dynamic_collision_cloud_,
        local_dynamic_collision_kdtree_);
}

bool ContourGraph::IsPointInsideReliableContourWindow(
    const Point3D& point) {
    if (local_observation_window_.enabled) {
        return local_observation_window_.Contains(point);
    }
    const float half_extent = std::max(
        0.0f, FARUtil::kSensorRange - contour_boundary_guard_);
    return std::abs(point.x - FARUtil::odom_pos.x) <= half_extent &&
           std::abs(point.y - FARUtil::odom_pos.y) <= half_extent;
}

bool ContourGraph::DoesSegmentIntersectReliableContourWindow(
    const Point3D& start, const Point3D& end) {
    if (local_observation_window_.enabled) {
        return local_observation_window_.SegmentIntersects(start, end);
    }
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

bool ContourGraph::IsSegmentFullyInsideReliableContourWindow(
    const Point3D& start, const Point3D& end) {
    if (local_observation_window_.enabled) {
        return local_observation_window_.SegmentFullyContained(start, end);
    }
    return IsPointInsideReliableContourWindow(start) &&
           IsPointInsideReliableContourWindow(end);
}

bool ContourGraph::IsPointInsideCurrentObservationWindow(
    const Point3D& point) {
    if (local_observation_window_.enabled) {
        // Contour raster dilation/interpolation can move a boundary vertex a
        // few cells beyond the exact voxel box. The contour is rasterized
        // around the graph-update pose while the local footprint is stamped
        // at the source-cloud pose, so retain one additional contour cell for
        // quantization and sub-frame pose skew. Keep this enlarged halo
        // current-only; it is never used to confirm persistent geometry.
        const float current_clip_tolerance =
            contour_boundary_guard_ + FARUtil::kLeafSize;
        return local_observation_window_.ContainsFull(
            point, current_clip_tolerance);
    }
    return std::abs(point.x - FARUtil::odom_pos.x) <= FARUtil::kSensorRange &&
           std::abs(point.y - FARUtil::odom_pos.y) <= FARUtil::kSensorRange;
}

bool ContourGraph::IsSegmentFullyInsideCurrentObservationWindow(
    const Point3D& start, const Point3D& end) {
    if (local_observation_window_.enabled) {
        const float current_clip_tolerance =
            contour_boundary_guard_ + FARUtil::kLeafSize;
        return local_observation_window_.SegmentFullyContainedFull(
            start, end, current_clip_tolerance);
    }
    return IsPointInsideCurrentObservationWindow(start) &&
           IsPointInsideCurrentObservationWindow(end);
}

void ContourGraph::SetLocalObservationPose(
    const Point3D& origin, const Point3D& forward) {
    local_observation_window_.origin = origin;
    local_observation_window_.forward = forward;
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
            polygon->is_pillar || polygon->vertices.size() < 2) {
            continue;
        }
        for (std::size_t index = 0; index < polygon->vertices.size(); ++index) {
            const Point3D& first = polygon->vertices[index];
            const Point3D& second = polygon->vertices[
                (index + 1) % polygon->vertices.size()];
            // A clipped polygon is not wholly unreliable.  Only its
            // synthetic window-closing cap is unobserved; a physical contour
            // side from an interior corner to a CLIP endpoint still provides
            // reliable evidence around an old vertex well inside W_inner.
            if (polygon->is_boundary_clipped &&
                IsArtificialBoundaryClosingSegment(first, second)) {
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
            if (distance > distance_tolerance || distance >= best_distance ||
                !IsPointInsideReliableContourWindow(projected)) continue;

            // Require a complete local neighbourhood on both sides of the
            // projection to lie in W_inner.  This makes reliability local to
            // the historical vertex instead of rejecting an entire polygon
            // merely because a remote endpoint is clipped.
            const float inverse_length = 1.0f / segment_length;
            const Point3D tangent(dx * inverse_length,
                                  dy * inverse_length, 0.0f);
            // Endpoint clearance answers whether this is truly the segment
            // interior; it is intentionally larger than the amount of local
            // contour needed to prove that the projection itself was
            // observed. Two contour cells are enough for the latter and keep
            // a valid interior section next to W_guard from being discarded.
            const float reliable_span = std::max(
                FARUtil::kLeafSize * 2.0f,
                std::min(distance_tolerance, contour_boundary_guard_));
            const Point3D before = projected - tangent * reliable_span;
            const Point3D after = projected + tangent * reliable_span;
            if (!IsPointInsideReliableContourWindow(before) ||
                !IsPointInsideReliableContourWindow(after)) {
                continue;
            }
            best_distance = distance;
            best_polygon = polygon;
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

    const bool observed_contour =
        ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2);
    const bool clip_attempt = !observed_contour &&
        ContourGraph::IsNavNodesConnectFromClipAttempt(node_ptr1, node_ptr2);
    if (!observed_contour && !clip_attempt) {
        return result;
    }

    const CTNodePtr ct1 = node_ptr1->ctnode;
    const CTNodePtr ct2 = node_ptr2->ctnode;
    // Both CT nodes belong to the contour graph reconstructed from this
    // observation.  A boundary-clipped vertex is therefore already proven
    // to be part of the current snapshot.  Rechecking its quantized world
    // coordinate against the moving local-window boundary is redundant and
    // can reject every incident contour edge after millimetre-scale pose
    // motion (the rasterized vertex commonly lies exactly on the halo).
    // CLIP persistence is handled by the snapshot-local node lifecycle; do
    // not turn that lifecycle rule into a current-frame topology gate here.
    CTNodeStack contour_chain;
    bool crossed_artificial_cap = false;
    if (!GetContourChain(ct1, ct2, contour_chain, clip_attempt,
                         &crossed_artificial_cap) ||
        (clip_attempt && !crossed_artificial_cap)) {
        return result;
    }

    // A FAR contour edge is a reduced topological relation, not a sampled
    // robot-centre trajectory.  Runtime waypoint projection and the local
    // planner provide free-space clearance when this relation is selected.
    result.valid = true;
    result.reason = EdgeRejectReason::NONE;
    result.mode = clip_attempt ? EdgeValidationMode::CLIP_ATTEMPT
                               : EdgeValidationMode::CONTOUR_FOLLOW;
    result.route_start = node_ptr1->position;
    result.route_end = node_ptr2->position;
    result.route_points = {node_ptr1->position, node_ptr2->position};
    result.route_cost =
        (node_ptr2->position - node_ptr1->position).norm();
    result.projection_distance = 0.0f;
    result.dynamic_blocked = false;
    return result;
}

bool ContourGraph::ValidateProjectedContourRoute(
    const std::vector<Point3D>& route_points,
    const PolygonPtr& endpoint_polygon, const bool static_structure,
    EdgeRejectReason& reason) {
    if (route_points.size() < 2) {
        reason = EdgeRejectReason::OFFSET_FAILED;
        return false;
    }
    for (std::size_t index = 1; index < route_points.size(); ++index) {
        const Point3D& segment_start = route_points[index - 1];
        const Point3D& segment_end = route_points[index];
        const ConnectPair route(segment_start, segment_end);
        const HeightPair route_height(segment_start, segment_end);
        const Point3D route_center =
            (segment_start + segment_end) / 2.0f;

        if (!local_static_collision_grid_.IsValid() && endpoint_polygon &&
            !endpoint_polygon->is_pillar &&
            !endpoint_polygon->is_boundary_clipped &&
            (endpoint_polygon->is_robot_inside !=
                 FARUtil::PointInsideAPoly(endpoint_polygon->vertices,
                                           route_center) ||
             IsEdgeCollidePoly(endpoint_polygon->vertices, route))) {
            reason = EdgeRejectReason::SELF_POLYGON_BLOCKED;
            return false;
        }
        if (!IsEdgeCollisionFreeInCloud(
                route, route_height, local_static_collision_cloud_,
                local_static_collision_kdtree_, 0.0f)) {
            reason = EdgeRejectReason::STATIC_CLOUD_BLOCKED;
            return false;
        }
        if (!local_static_collision_grid_.IsValid() &&
            !IsPointsConnectFreePolygonForLayer(
                route, route, route_height, false,
                CollisionLayer::STATIC_ONLY, endpoint_polygon,
                endpoint_polygon, false)) {
            reason = EdgeRejectReason::OTHER_STATIC_BLOCKED;
            return false;
        }
        if (!static_structure &&
            (!IsEdgeCollisionFreeInCloud(
                 route, route_height, local_dynamic_collision_cloud_,
                 local_dynamic_collision_kdtree_, 0.0f) ||
             (!local_dynamic_collision_grid_.IsValid() &&
              !IsPointsConnectFreePolygonForLayer(
                  route, route, route_height, false,
                  CollisionLayer::DYNAMIC_ONLY, endpoint_polygon,
                  endpoint_polygon, false)))) {
            reason = EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
            return false;
        }
    }
    reason = EdgeRejectReason::NONE;
    return true;
}

bool ContourGraph::BuildConvexHullContourRoute(
    const CTNodePtr& first, const CTNodePtr& second,
    const PointStack& dense_chain, const PolygonPtr& polygon,
    std::vector<Point3D>& route_points, float& projection_distance,
    EdgeRejectReason& reason) {
    route_points.clear();
    projection_distance = 0.0f;
    if (!first || !second || !polygon || polygon->is_pillar ||
        polygon->is_boundary_clipped || dense_chain.size() < 2) {
        return false;
    }
    const PointStack& boundary = polygon->dense_vertices.size() >= 3
        ? polygon->dense_vertices : polygon->vertices;
    if (boundary.size() < 3) return false;

    std::vector<cv::Point2f> input;
    input.reserve(boundary.size());
    for (const Point3D& point : boundary) {
        input.emplace_back(point.x, point.y);
    }
    std::vector<cv::Point2f> hull_cv;
    cv::convexHull(input, hull_cv, false, true);
    if (hull_cv.size() < 3) return false;

    PointStack hull;
    hull.reserve(hull_cv.size());
    const float route_height = (first->position.z + second->position.z) * 0.5f;
    for (const cv::Point2f& point : hull_cv) {
        hull.emplace_back(point.x, point.y, route_height);
    }
    const auto nearest_hull_index = [&hull](const Point3D& point) {
        std::size_t best_index = 0;
        float best_distance = FARUtil::kINF;
        for (std::size_t index = 0; index < hull.size(); ++index) {
            const float distance =
                (hull[index] - point).norm_flat();
            if (distance < best_distance) {
                best_distance = distance;
                best_index = index;
            }
        }
        return best_index;
    };
    const std::size_t first_index = nearest_hull_index(first->position);
    const std::size_t second_index = nearest_hull_index(second->position);
    if (first_index == second_index) return false;

    float signed_twice_area = 0.0f;
    for (std::size_t index = 0; index < hull.size(); ++index) {
        const Point3D& current = hull[index];
        const Point3D& next = hull[(index + 1) % hull.size()];
        signed_twice_area += current.x * next.y - next.x * current.y;
    }
    const bool counter_clockwise = signed_twice_area > 0.0f;
    const auto outward_normal = [counter_clockwise](
        const Point3D& from, const Point3D& to) {
        const Point3D edge = (to - from).normalize_flat();
        return counter_clockwise
            ? Point3D(edge.y, -edge.x, 0.0f)
            : Point3D(-edge.y, edge.x, 0.0f);
    };

    const Point3D dense_midpoint = dense_chain[dense_chain.size() / 2];
    const auto distance_to_arc = [&hull, &dense_midpoint](
        const std::vector<std::size_t>& arc) {
        float distance = FARUtil::kINF;
        for (std::size_t index = 1; index < arc.size(); ++index) {
            distance = std::min(
                distance, FARUtil::DistanceToLineSeg2D(
                    dense_midpoint,
                    PointPair(hull[arc[index - 1]], hull[arc[index]])));
        }
        return distance;
    };
    const auto make_arc = [&hull, first_index, second_index](
        const int step) {
        std::vector<std::size_t> arc;
        std::size_t index = first_index;
        for (std::size_t count = 0; count <= hull.size(); ++count) {
            arc.push_back(index);
            if (index == second_index) break;
            index = step > 0
                ? (index + 1) % hull.size()
                : (index + hull.size() - 1) % hull.size();
        }
        return arc;
    };
    std::vector<std::size_t> forward_arc = make_arc(1);
    std::vector<std::size_t> backward_arc = make_arc(-1);
    const std::vector<std::size_t>& selected_arc =
        distance_to_arc(forward_arc) <= distance_to_arc(backward_arc)
            ? forward_arc : backward_arc;

    for (float projection = contour_projection_min_;
         projection <= contour_projection_max_ + FARUtil::kEpsilon;
         projection += contour_projection_step_) {
        PointStack offset_hull(hull.size());
        bool offset_valid = true;
        for (std::size_t index = 0; index < hull.size(); ++index) {
            const std::size_t previous =
                (index + hull.size() - 1) % hull.size();
            const std::size_t next = (index + 1) % hull.size();
            const Point3D previous_normal =
                outward_normal(hull[previous], hull[index]);
            const Point3D next_normal =
                outward_normal(hull[index], hull[next]);
            Point3D bisector = previous_normal + next_normal;
            if (bisector.norm_flat() <= FARUtil::kEpsilon) {
                offset_valid = false;
                break;
            }
            bisector = bisector.normalize_flat();
            const float denominator = std::min(
                bisector.norm_flat_dot(previous_normal),
                bisector.norm_flat_dot(next_normal));
            if (denominator <= 0.1f) {
                offset_valid = false;
                break;
            }
            const float miter = std::min(
                projection / denominator, projection * 3.0f);
            offset_hull[index] = hull[index] + bisector * miter;
        }
        if (!offset_valid) continue;

        PointStack candidate;
        candidate.reserve(selected_arc.size() + 2);
        const cv::Point2f first_projected = ProjectNode(first, projection);
        const cv::Point2f second_projected = ProjectNode(second, projection);
        candidate.emplace_back(first_projected.x, first_projected.y,
                               first->position.z);
        for (const std::size_t index : selected_arc) {
            if ((candidate.back() - offset_hull[index]).norm_flat() >
                FARUtil::kEpsilon) {
                candidate.push_back(offset_hull[index]);
            }
        }
        const Point3D projected_second(second_projected.x,
                                       second_projected.y,
                                       second->position.z);
        if ((candidate.back() - projected_second).norm_flat() >
            FARUtil::kEpsilon) {
            candidate.push_back(projected_second);
        }
        bool inside_window = true;
        for (const Point3D& point : candidate) {
            if (!IsPointInsideCurrentObservationWindow(point)) {
                inside_window = false;
                break;
            }
        }
        if (!inside_window || !IsRouteConnectFreeStaticLayer(candidate)) {
            reason = EdgeRejectReason::STATIC_CLOUD_BLOCKED;
            continue;
        }
        EdgeRejectReason validation_reason = EdgeRejectReason::NONE;
        if (!ValidateProjectedContourRoute(
                candidate, polygon, true, validation_reason)) {
            reason = validation_reason;
            continue;
        }
        route_points = std::move(candidate);
        projection_distance = projection;
        reason = EdgeRejectReason::NONE;
        return true;
    }
    return false;
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

    if (local_static_collision_grid_.IsValid()) {
        // The current polygon was extracted from an already inflated
        // configuration-space obstacle.  Applying kNavClearDist again here
        // would double the robot radius.  Only actual intersection/interior
        // is blocking; the raster supercover performs the authoritative
        // current-layer check.
        const Point3D midpoint(
            (edge.start_p.x + edge.end_p.x) * 0.5f,
            (edge.start_p.y + edge.end_p.y) * 0.5f, 0.0f);
        return IsEdgeCollidePoly(poly_ptr->vertices, edge) ||
               FARUtil::PointInsideAPoly(poly_ptr->vertices, midpoint);
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

bool ContourGraph::IsArtificialBoundaryClosingSegment(
    const Point3D& first, const Point3D& second) {
    if (IsPointInsideReliableContourWindow(first) ||
        IsPointInsideReliableContourWindow(second)) {
        return false;
    }

    const float tolerance = std::max(0.05f, FARUtil::kLeafSize * 1.5f);
    const auto near_same_side = [tolerance](
        const float first_value, const float second_value,
        const float boundary) {
        return std::abs(first_value - boundary) <= tolerance &&
               std::abs(second_value - boundary) <= tolerance;
    };

    if (local_observation_window_.enabled &&
        local_observation_window_.IsValid()) {
        const Point3D local_first = local_observation_window_.ToLocal(first);
        const Point3D local_second = local_observation_window_.ToLocal(second);
        const float halo = local_observation_window_.guard;
        return near_same_side(local_first.x, local_second.x,
                              local_observation_window_.min_x - halo) ||
               near_same_side(local_first.x, local_second.x,
                              local_observation_window_.min_x) ||
               near_same_side(local_first.x, local_second.x,
                              local_observation_window_.max_x) ||
               near_same_side(local_first.x, local_second.x,
                              local_observation_window_.max_x + halo) ||
               near_same_side(local_first.y, local_second.y,
                              local_observation_window_.min_y - halo) ||
               near_same_side(local_first.y, local_second.y,
                              local_observation_window_.min_y) ||
               near_same_side(local_first.y, local_second.y,
                              local_observation_window_.max_y) ||
               near_same_side(local_first.y, local_second.y,
                              local_observation_window_.max_y + halo);
    }

    const float range = FARUtil::kSensorRange;
    return near_same_side(first.x, second.x, FARUtil::odom_pos.x - range) ||
           near_same_side(first.x, second.x, FARUtil::odom_pos.x + range) ||
           near_same_side(first.y, second.y, FARUtil::odom_pos.y - range) ||
           near_same_side(first.y, second.y, FARUtil::odom_pos.y + range);
}

bool ContourGraph::IsBoundaryClippedPolygonBlocked(
    const PolygonPtr& polygon, const ConnectPair& edge) {
    if (!polygon || polygon->vertices.size() < 2) return false;
    for (std::size_t index = 0; index < polygon->vertices.size(); ++index) {
        const Point3D& first = polygon->vertices[index];
        const Point3D& second =
            polygon->vertices[(index + 1) % polygon->vertices.size()];
        if (IsArtificialBoundaryClosingSegment(first, second)) continue;
        if (IsEdgeCollideSegment(PointPair(first, second), edge)) return true;
    }
    return false;
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
            // A boundary-clipped contour is physically open only along its
            // synthetic window cap. Its observed sides remain obstacles.
            if (poly_ptr->is_boundary_clipped) {
                if (IsBoundaryClippedPolygonBlocked(poly_ptr, cedge)) {
                    return false;
                }
                continue;
            }
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
            if (poly_ptr->is_boundary_clipped) {
                if (IsBoundaryClippedPolygonBlocked(poly_ptr, cedge)) {
                    return false;
                }
                continue;
            }
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

const ContourGraph::LocalCollisionGrid2D*
ContourGraph::CollisionGridForCloud(const PointCloudPtr& cloud) {
    if (!cloud) return nullptr;
    if (cloud.get() == local_static_collision_cloud_.get() &&
        local_static_collision_grid_.IsValid()) {
        return &local_static_collision_grid_;
    }
    if (cloud.get() == local_dynamic_collision_cloud_.get() &&
        local_dynamic_collision_grid_.IsValid()) {
        return &local_dynamic_collision_grid_;
    }
    if (cloud.get() == local_collision_cloud_.get() &&
        local_collision_grid_.IsValid()) {
        return &local_collision_grid_;
    }
    return nullptr;
}

bool ContourGraph::IsEdgeCollisionFreeInGrid(
    const ConnectPair& edge, const LocalCollisionGrid2D& grid,
    const float endpoint_exclusion) {
    if (!grid.IsValid()) return true;

    const float world_dx = edge.end_p.x - edge.start_p.x;
    const float world_dy = edge.end_p.y - edge.start_p.y;
    const float length = std::hypot(world_dx, world_dy);
    if (length < FARUtil::kEpsilon) return true;

    // Explicit non-negative exclusions retain their previous meaning.  The
    // legacy negative value is used only when an obstacle endpoint cannot be
    // projected (notably a one-node pillar); exclude no more than two refined
    // cells because clearance is already encoded in the configuration grid.
    const float endpoint_margin = endpoint_exclusion >= 0.0f
        ? std::min(endpoint_exclusion, length * 0.45f)
        : std::min(length * 0.45f, grid.resolution * 2.0f);
    if (length < endpoint_margin * 2.0f + FARUtil::kEpsilon) return true;

    const float first_ratio = endpoint_margin / length;
    const float last_ratio = (length - endpoint_margin) / length;
    const float start_x = edge.start_p.x + world_dx * first_ratio;
    const float start_y = edge.start_p.y + world_dy * first_ratio;
    const float end_x = edge.start_p.x + world_dx * last_ratio;
    const float end_y = edge.start_p.y + world_dy * last_ratio;

    // World x maps to image row and world y maps to image column.  Adding
    // 0.5 converts pixel-centre coordinates to cell-boundary coordinates so
    // floor() selects the containing cell for Amanatides-Woo traversal.
    const double grid_start_row = grid.center_row +
        (start_x - grid.center.x) / grid.resolution + 0.5;
    const double grid_start_col = grid.center_col +
        (start_y - grid.center.y) / grid.resolution + 0.5;
    const double grid_end_row = grid.center_row +
        (end_x - grid.center.x) / grid.resolution + 0.5;
    const double grid_end_col = grid.center_col +
        (end_y - grid.center.y) / grid.resolution + 0.5;

    int row = static_cast<int>(std::floor(grid_start_row));
    int col = static_cast<int>(std::floor(grid_start_col));
    const int end_row = static_cast<int>(std::floor(grid_end_row));
    const int end_col = static_cast<int>(std::floor(grid_end_col));
    const auto occupied = [&grid](const int query_row,
                                  const int query_col) {
        // The configuration grid is authoritative only inside its raster.
        // Historical contours/window policy handle the unobserved exterior.
        if (query_row < 0 || query_row >= grid.occupied.rows ||
            query_col < 0 || query_col >= grid.occupied.cols) {
            return false;
        }
        return grid.occupied.at<std::uint8_t>(query_row, query_col) != 0;
    };
    if (occupied(row, col)) return false;

    const double delta_row = grid_end_row - grid_start_row;
    const double delta_col = grid_end_col - grid_start_col;
    const int step_row = delta_row > 0.0 ? 1 : (delta_row < 0.0 ? -1 : 0);
    const int step_col = delta_col > 0.0 ? 1 : (delta_col < 0.0 ? -1 : 0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double t_delta_row = step_row == 0
        ? infinity : 1.0 / std::abs(delta_row);
    const double t_delta_col = step_col == 0
        ? infinity : 1.0 / std::abs(delta_col);
    double t_max_row = step_row == 0 ? infinity :
        ((step_row > 0 ? std::floor(grid_start_row) + 1.0
                       : std::floor(grid_start_row)) - grid_start_row) /
        delta_row;
    double t_max_col = step_col == 0 ? infinity :
        ((step_col > 0 ? std::floor(grid_start_col) + 1.0
                       : std::floor(grid_start_col)) - grid_start_col) /
        delta_col;

    // A line passing exactly through a cell corner belongs to all touched
    // cells in a supercover.  Checking both orthogonal neighbours prevents
    // diagonal corner cutting between two occupied configuration cells.
    constexpr double kTraversalEpsilon = 1e-10;
    const int max_iterations = grid.occupied.rows + grid.occupied.cols + 8;
    int iterations = 0;
    while ((row != end_row || col != end_col) &&
           iterations++ < max_iterations) {
        if (t_max_row + kTraversalEpsilon < t_max_col) {
            row += step_row;
            t_max_row += t_delta_row;
            if (occupied(row, col)) return false;
        } else if (t_max_col + kTraversalEpsilon < t_max_row) {
            col += step_col;
            t_max_col += t_delta_col;
            if (occupied(row, col)) return false;
        } else {
            if (occupied(row + step_row, col) ||
                occupied(row, col + step_col)) {
                return false;
            }
            row += step_row;
            col += step_col;
            t_max_row += t_delta_row;
            t_max_col += t_delta_col;
            if (occupied(row, col)) return false;
        }
    }
    return true;
}

bool ContourGraph::IsEdgeCollisionFreeInCloud(
    const ConnectPair& edge, const HeightPair& edge_height,
    const PointCloudPtr& cloud, const PointKdTreePtr& kdtree,
    const float endpoint_exclusion) {
    if (const LocalCollisionGrid2D* grid = CollisionGridForCloud(cloud)) {
        return IsEdgeCollisionFreeInGrid(edge, *grid, endpoint_exclusion);
    }
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

bool ContourGraph::IsPointCollisionFreeInCloud(
    const Point3D& point, const PointCloudPtr& cloud,
    const PointKdTreePtr& kdtree) {
    if (const LocalCollisionGrid2D* grid = CollisionGridForCloud(cloud)) {
        const int row = static_cast<int>(std::floor(
            grid->center_row + (point.x - grid->center.x) /
                grid->resolution + 0.5f));
        const int col = static_cast<int>(std::floor(
            grid->center_col + (point.y - grid->center.y) /
                grid->resolution + 0.5f));
        if (row < 0 || row >= grid->occupied.rows ||
            col < 0 || col >= grid->occupied.cols) {
            return true;
        }
        return grid->occupied.at<std::uint8_t>(row, col) == 0;
    }
    if (!cloud || cloud->empty() || !kdtree || !kdtree->getInputCloud()) {
        return true;
    }
    PCLPoint sample;
    sample.x = point.x;
    sample.y = point.y;
    sample.z = point.z;
    sample.intensity = 0.0f;
    const float radius = std::max(FARUtil::kLeafSize * 0.75f,
                                  FARUtil::kNavClearDist);
    std::vector<int> indices;
    std::vector<float> squared_distances;
    return kdtree->radiusSearch(
               sample, radius, indices, squared_distances, 1) == 0;
}

bool ContourGraph::IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_odom || node_ptr2->is_odom) return false;
    const CTNodePtr ctnode1 = node_ptr1->ctnode;
    const CTNodePtr ctnode2 = node_ptr2->ctnode;
    if (ctnode1 == NULL || ctnode2 == NULL || ctnode1 == ctnode2) return false;
    return ContourGraph::IsCTNodesConnectFromContour(ctnode1, ctnode2);
}

bool ContourGraph::IsNavNodesConnectFromClipAttempt(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (!node_ptr1 || !node_ptr2 || node_ptr1->is_odom || node_ptr2->is_odom ||
        !node_ptr1->ctnode || !node_ptr2->ctnode) {
        return false;
    }
    return IsCTNodesConnectFromClipAttempt(node_ptr1->ctnode,
                                           node_ptr2->ctnode);
}

bool ContourGraph::IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
    CTNodeStack chain;
    return GetContourChain(ctnode1, ctnode2, chain);
}

bool ContourGraph::IsCTNodesConnectFromClipAttempt(
    const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
    if (!ctnode1 || !ctnode2 || !ctnode1->is_boundary_clipped ||
        !ctnode2->is_boundary_clipped) {
        return false;
    }
    CTNodeStack chain;
    bool crossed_artificial_cap = false;
    return GetContourChain(ctnode1, ctnode2, chain, true,
                           &crossed_artificial_cap) &&
           crossed_artificial_cap;
}

bool ContourGraph::GetContourChain(const CTNodePtr& ctnode1,
                                   const CTNodePtr& ctnode2,
                                   CTNodeStack& chain,
                                   const bool allow_artificial_cap,
                                   bool* crossed_artificial_cap) {
    chain.clear();
    if (crossed_artificial_cap) *crossed_artificial_cap = false;
    if (!ctnode1 || !ctnode2 || ctnode1 == ctnode2 ||
        ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    // check for boundary collision
    const ConnectPair cedge = ConnectPair(ctnode1->position, ctnode2->position);
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
            return false;
        }
    }

    // Match upstream FAR: walk each contour direction until the target is
    // reached. A different matched CT vertex owns the next reduced interval;
    // an unmatched intermediate vertex may be skipped only while it remains
    // inside kNearDist of the endpoint chord. Deep bends are handled by
    // EnclosePolygonsCheck(), which promotes the blocking vertex as a
    // contour-necessary node instead of encoding a dense execution route.
    const auto trace = [&ctnode1, &ctnode2, allow_artificial_cap](
        const bool use_front, CTNodeStack& candidate,
        bool& candidate_crossed_cap) {
        candidate.clear();
        candidate_crossed_cap = false;
        candidate.push_back(ctnode1);
        CTNodePtr previous = ctnode1;
        CTNodePtr current = use_front ? ctnode1->front : ctnode1->back;
        while (current && current != ctnode1) {
            // findContours closes every cropped obstacle into a polygon by
            // drawing a segment along the raster/window boundary. That cap
            // is not an observed obstacle surface and must never become a
            // contour-follow topology edge. The CLIP vertex itself remains a
            // valid snapshot-local endpoint through its physical neighbour.
            if (ctnode1->poly_ptr &&
                ctnode1->poly_ptr->is_boundary_clipped &&
                IsArtificialBoundaryClosingSegment(
                    previous->position, current->position)) {
                if (!allow_artificial_cap) {
                    candidate.clear();
                    return false;
                }
                candidate_crossed_cap = true;
            }
            if (current == ctnode2) {
                candidate.push_back(current);
                return true;
            }
            if (current->is_global_match ||
                !FARUtil::IsInCylinder(
                    ctnode1->position, ctnode2->position,
                    current->position, FARUtil::kNearDist, true)) {
                candidate.clear();
                return false;
            }
            candidate.push_back(current);
            previous = current;
            current = use_front ? current->front : current->back;
        }
        candidate.clear();
        return false;
    };
    CTNodeStack forward_chain;
    CTNodeStack backward_chain;
    bool forward_crossed_cap = false;
    bool backward_crossed_cap = false;
    const bool has_forward = trace(
        true, forward_chain, forward_crossed_cap);
    const bool has_backward = trace(
        false, backward_chain, backward_crossed_cap);
    if (!has_forward && !has_backward) return false;
    // FAR tests front first and accepts it immediately. Preserve that stable
    // contour-order tie break rather than selecting a route by arc length.
    if (has_forward && has_backward) {
        chain = forward_chain;
        if (crossed_artificial_cap) {
            *crossed_artificial_cap = forward_crossed_cap;
        }
    } else {
        chain = has_forward ? forward_chain : backward_chain;
        if (crossed_artificial_cap) {
            *crossed_artificial_cap = has_forward
                ? forward_crossed_cap : backward_crossed_cap;
        }
    }
    return chain.size() >= 2;
}

bool ContourGraph::GetDenseContourChain(
    const CTNodePtr& ctnode1, const CTNodePtr& ctnode2,
    const CTNodeStack& sparse_chain, PointStack& dense_chain) {
    dense_chain.clear();
    if (!ctnode1 || !ctnode2 || sparse_chain.size() < 2 ||
        sparse_chain.front() != ctnode1 || sparse_chain.back() != ctnode2 ||
        ctnode1->poly_ptr != ctnode2->poly_ptr) {
        return false;
    }
    const PolygonPtr polygon = ctnode1->poly_ptr;
    if (!polygon || polygon->dense_vertices.size() < 2 ||
        polygon->simplified_dense_indices.size() !=
            polygon->vertices.size() ||
        ctnode1->contour_index >=
            polygon->simplified_dense_indices.size() ||
        ctnode2->contour_index >=
            polygon->simplified_dense_indices.size()) {
        for (const CTNodePtr& node : sparse_chain) {
            if (node) dense_chain.push_back(node->position);
        }
        return dense_chain.size() >= 2;
    }

    const bool use_front = sparse_chain[1] == ctnode1->front;
    const bool use_back = sparse_chain[1] == ctnode1->back;
    if (!use_front && !use_back) return false;
    const std::size_t count = polygon->dense_vertices.size();
    std::size_t current = polygon->simplified_dense_indices[
        ctnode1->contour_index] % count;
    const std::size_t target = polygon->simplified_dense_indices[
        ctnode2->contour_index] % count;
    dense_chain.push_back(polygon->dense_vertices[current]);
    for (std::size_t steps = 0; current != target && steps < count; ++steps) {
        current = use_front
            ? (current + count - 1) % count
            : (current + 1) % count;
        dense_chain.push_back(polygon->dense_vertices[current]);
    }
    if (current != target || dense_chain.size() < 2) {
        dense_chain.clear();
        for (const CTNodePtr& node : sparse_chain) {
            if (node) dense_chain.push_back(node->position);
        }
    }
    return dense_chain.size() >= 2;
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
    const auto is_static = [](const CTNodePtr& node) {
        return node &&
            (node->source == GraphNodeSource::STATIC_CANDIDATE ||
             node->source == GraphNodeSource::STATIC_GLOBAL);
    };
    const auto is_routing_anchor = [this](const CTNodePtr& node) {
        if (!node || node->free_direct == NodeFreeDirect::UNKNOW ||
            node->free_direct == NodeFreeDirect::CONCAVE) {
            return false;
        }
        if (node->is_global_match ||
            node->free_direct == NodeFreeDirect::PILLAR) {
            return true;
        }
        return node->surf_dirs.first * node->surf_dirs.second >=
               ALIGN_ANGLE_COS;
    };
    const auto can_retain_as_necessary = [](const CTNodePtr& node) {
        if (!node || node->free_direct != NodeFreeDirect::CONCAVE) {
            return false;
        }
        // Missing terrain support is UNKNOWN in the local-only semantic
        // pipeline, not evidence that the contour bend is invalid.  The
        // graph-node admission stage still rejects a known height conflict.
        return UsesLocalObservationWindow()
            ? IsPointInsideCurrentObservationWindow(node->position)
            : FARUtil::IsPointInMarginRange(node->position);
    };

    std::size_t necessary_count = 0;
    for (const auto& polygon_start : ContourGraph::polys_ctnodes_) {
        if (!polygon_start || !polygon_start->poly_ptr ||
            polygon_start->poly_ptr->is_pillar ||
            !is_static(polygon_start)) {
            continue;
        }

        CTNodeStack ordered;
        CTNodePtr current = polygon_start;
        do {
            ordered.push_back(current);
            current = current->front;
        } while (current && current != polygon_start &&
                 ordered.size() <= polygon_start->poly_ptr->N);
        if (!current || ordered.size() < 3) continue;

        std::vector<std::size_t> anchors;
        for (std::size_t index = 0; index < ordered.size(); ++index) {
            if (is_routing_anchor(ordered[index])) anchors.push_back(index);
        }
        if (anchors.size() < 2) continue;

        // Reduce every physical contour interval independently.  If an
        // endpoint chord hides a bend farther than kNearDist, retain the
        // farthest eligible CONCAVE vertex and recurse on both halves.  This
        // is the contour equivalent of RDP: only the vertices required to
        // express the bend enter the graph, while ordinary concave samples
        // remain collision geometry only.
        const auto mark_interval = [&ordered, &can_retain_as_necessary,
                                    &necessary_count](
            const std::vector<std::size_t>& path) {
            std::function<void(std::size_t, std::size_t)> split;
            split = [&ordered, &path, &can_retain_as_necessary,
                     &necessary_count, &split](
                const std::size_t begin, const std::size_t end) {
                if (end <= begin + 1) return;
                const PointPair chord(ordered[path[begin]]->position,
                                      ordered[path[end]]->position);
                float max_distance = FARUtil::kNearDist;
                std::size_t split_index = end;
                for (std::size_t index = begin + 1; index < end; ++index) {
                    const CTNodePtr& candidate = ordered[path[index]];
                    if (!can_retain_as_necessary(candidate)) continue;
                    const float distance = FARUtil::DistanceToLineSeg2D(
                        candidate->position, chord);
                    if (distance > max_distance) {
                        max_distance = distance;
                        split_index = index;
                    }
                }
                if (split_index == end) return;
                CTNodePtr& necessary = ordered[path[split_index]];
                if (!necessary->is_contour_necessary) {
                    necessary->is_contour_necessary = true;
                    ++necessary_count;
                }
                split(begin, split_index);
                split(split_index, end);
            };
            split(0, path.size() - 1);
        };

        for (std::size_t anchor = 0; anchor < anchors.size(); ++anchor) {
            const std::size_t begin = anchors[anchor];
            const std::size_t end = anchors[(anchor + 1) % anchors.size()];
            std::vector<std::size_t> path;
            path.push_back(begin);
            std::size_t index = begin;
            bool artificial_cap = false;
            while (index != end && path.size() <= ordered.size()) {
                const std::size_t next = (index + 1) % ordered.size();
                if (IsArtificialBoundaryClosingSegment(
                        ordered[index]->position, ordered[next]->position)) {
                    artificial_cap = true;
                }
                path.push_back(next);
                index = next;
            }
            if (!artificial_cap && path.size() >= 3) {
                mark_interval(path);
            }
        }
    }
    ROS_INFO_THROTTLE(
        5.0, "CG contour reduction retained necessary_concave=%zu",
        necessary_count);
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
                                 const GraphNodeSource source,
                                 const PointStack& dense_points,
                                 const std::vector<std::size_t>&
                                     simplified_dense_indices) {
    poly_ptr = std::make_shared<Polygon>();
    poly_ptr->N = poly_points.size();
    poly_ptr->vertices = poly_points;
    poly_ptr->dense_vertices = dense_points.empty()
        ? poly_points : dense_points;
    poly_ptr->simplified_dense_indices = simplified_dense_indices;
    if (poly_ptr->simplified_dense_indices.size() != poly_points.size()) {
        poly_ptr->simplified_dense_indices.clear();
        poly_ptr->simplified_dense_indices.reserve(poly_points.size());
        for (const Point3D& vertex : poly_points) {
            std::size_t best_index = 0;
            float best_distance = FARUtil::kINF;
            for (std::size_t index = 0;
                 index < poly_ptr->dense_vertices.size(); ++index) {
                const float distance =
                    (vertex - poly_ptr->dense_vertices[index]).norm_flat();
                if (distance < best_distance) {
                    best_distance = distance;
                    best_index = index;
                }
            }
            poly_ptr->simplified_dense_indices.push_back(best_index);
        }
    }
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
