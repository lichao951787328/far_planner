/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/map_handler.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <octomap/OcTree.h>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

using octomap::ColorOcTreeNode;
using octomap::point3d;

namespace {

inline uint32_t MakeRgbKey(uint8_t r, uint8_t g, uint8_t b) {
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

using SemanticOctree = octomap::SemanticOcTree<octomap::SemanticsLogOdds>;
using SemanticOcTreeNode = octomap::SemanticOcTreeNode<octomap::SemanticsLogOdds>;

template <typename GroupContainer>
inline bool MatchRgbKey(const GroupContainer& groups, uint32_t rgb_key) {
    for (const auto& group : groups) {
        if (group.rgb_key == rgb_key) return true;
    }
    return false;
}

// Three signed 21-bit grid coordinates. At 0.2 m resolution this remains
// unique for roughly +/-209 km, far beyond the intended mapping workspace.
inline uint64_t PersistentStaticKey(const PCLPoint& point,
                                    const float resolution) {
    const double inverse = 1.0 / std::max(1e-3f, resolution);
    const int64_t ix = static_cast<int64_t>(std::floor(point.x * inverse));
    const int64_t iy = static_cast<int64_t>(std::floor(point.y * inverse));
    const int64_t iz = static_cast<int64_t>(std::floor(point.z * inverse));
    constexpr uint64_t mask = (1ULL << 21) - 1ULL;
    return ((static_cast<uint64_t>(ix) & mask) << 42) |
           ((static_cast<uint64_t>(iy) & mask) << 21) |
           (static_cast<uint64_t>(iz) & mask);
}

inline PCLPoint PersistentStaticCellCenter(const PCLPoint& point,
                                           const float resolution) {
    const float safe_resolution = std::max(1e-3f, resolution);
    PCLPoint center = point;
    center.x = (std::floor(point.x / safe_resolution) + 0.5f) * safe_resolution;
    center.y = (std::floor(point.y / safe_resolution) + 0.5f) * safe_resolution;
    center.z = (std::floor(point.z / safe_resolution) + 0.5f) * safe_resolution;
    center.intensity = 0.0f;
    return center;
}

struct QueryBox {
    point3d min;
    point3d max;
};

inline bool BoxIntersectsVoxel(const point3d& center,
                               const double size,
                               const QueryBox& box) {
    const double half = size * 0.5;
    return center.x() + half >= box.min.x() && center.x() - half <= box.max.x() &&
           center.y() + half >= box.min.y() && center.y() - half <= box.max.y() &&
           center.z() + half >= box.min.z() && center.z() - half <= box.max.z();
}

inline bool PointInsideBox(const point3d& point, const QueryBox& box) {
    return point.x() >= box.min.x() && point.x() <= box.max.x() &&
           point.y() >= box.min.y() && point.y() <= box.max.y() &&
           point.z() >= box.min.z() && point.z() <= box.max.z();
}

inline bool PointInsideBox(const PCLPoint& point, const QueryBox& box) {
    return point.x >= box.min.x() && point.x <= box.max.x() &&
           point.y >= box.min.y() && point.y <= box.max.y() &&
           point.z >= box.min.z() && point.z <= box.max.z();
}

void AppendExpandedVoxelCenters(const point3d& node_center,
                                const double node_size,
                                const double resolution,
                                const QueryBox& box,
                                const PointCloudPtr& cloud_out) {
    if (!BoxIntersectsVoxel(node_center, node_size, box)) return;
    if (node_size <= resolution + 1e-6) {
        if (!PointInsideBox(node_center, box)) return;
        PCLPoint point;
        point.x = node_center.x();
        point.y = node_center.y();
        point.z = node_center.z();
        point.intensity = 0.0f;
        cloud_out->points.push_back(point);
        return;
    }

    const double child_size = node_size * 0.5;
    const double offset = child_size * 0.5;
    for (int i = 0; i < 8; ++i) {
        point3d child_center = node_center;
        child_center.x() += (i & 1) ? offset : -offset;
        child_center.y() += (i & 2) ? offset : -offset;
        child_center.z() += (i & 4) ? offset : -offset;
        AppendExpandedVoxelCenters(child_center, child_size, resolution,
                                   box, cloud_out);
    }
}

inline void FinalizeCloud(const PointCloudPtr& cloud) {
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
}

inline void AppendCloud(const PointCloudPtr& source,
                        const PointCloudPtr& destination) {
    if (!source || !destination || source->empty()) return;
    destination->points.insert(destination->points.end(),
                               source->points.begin(), source->points.end());
}

template <typename TreeType, typename ColorGetter>
void ExtractClassifiedCloudInBox(
    const TreeType& tree,
    const QueryBox& box,
    const std::vector<SemanticClassGroup>& groups,
    const ColorGetter& color_getter,
    const PointCloudPtr& cloud_out) {
    cloud_out->clear();
    const double resolution = tree.getResolution();
    for (auto it = tree.begin_leafs_bbx(box.min, box.max),
              end = tree.end_leafs_bbx(); it != end; ++it) {
        if (!tree.isNodeOccupied(*it)) continue;
        const ColorOcTreeNode::Color color = color_getter(it);
        if (!MatchRgbKey(groups, MakeRgbKey(color.r, color.g, color.b))) continue;
        AppendExpandedVoxelCenters(it.getCoordinate(), it.getSize(), resolution,
                                   box, cloud_out);
    }
    FinalizeCloud(cloud_out);
}

template <typename TreeType>
void BuildDenseVoxelMap(const TreeType& tree,
                        const PointCloudPtr& cloud,
                        std::unordered_map<uint64_t, PCLPoint>& voxel_map) {
    voxel_map.clear();
    for (const auto& point : cloud->points) {
        octomap::OcTreeKey key;
        if (!tree.coordToKeyChecked(point.x, point.y, point.z, key)) continue;
        const uint64_t packed = (static_cast<uint64_t>(key[0]) << 32) |
                                (static_cast<uint64_t>(key[1]) << 16) |
                                static_cast<uint64_t>(key[2]);
        voxel_map.emplace(packed, point);
    }
}

}  // namespace

std::shared_ptr<octomap::OcTree> MapHandler::local_terrain_support_octree_;
float MapHandler::terrain_search_radius_ = 0.8f;
float MapHandler::terrain_neighbor_radius_ = 1.0f;

MapHandler::MapHandler() {
    // SemanticOcTree is a class template, so merely including its header does
    // not guarantee that OctoMap's factory-registration static is emitted by
    // the linker. Construct one process-lifetime probe before the first
    // octomap_msgs::msgToMap(); its constructor calls ensureLinking().
    static SemanticOctree semantic_registration_probe(0.1);
    (void)semantic_registration_probe;
}

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    semantic_params_ = params.semantic_params;
    obstacle_groups_ = params.obstacle_groups;
    terrain_support_groups_ = params.terrain_support_groups;
    dynamic_obstacle_groups_ = params.dynamic_obstacle_groups;
    if (semantic_params_.local_window_radius <= 0.0f) {
        semantic_params_.local_window_radius = std::max(0.0f, map_params_.sensor_range);
    }
    terrain_search_radius_ = semantic_params_.terrain_search_radius > 0.0f
        ? semantic_params_.terrain_search_radius : 0.8f;
    terrain_neighbor_radius_ = semantic_params_.terrain_neighbor_radius > 0.0f
        ? semantic_params_.terrain_neighbor_radius : 1.0f;
    semantic_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    persistent_static_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    semantic_terrain_support_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    current_dynamic_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    effective_dynamic_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    collision_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    dynamic_added_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    dynamic_removed_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    changed_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    previous_local_obs_voxels_.clear();
    previous_local_dynamic_voxels_.clear();
    persistent_static_obs_voxels_.clear();
    flat_terrain_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    if (!local_terrain_support_octree_) {
        local_terrain_support_octree_.reset(new octomap::OcTree(FARUtil::kLeafSize));
    } else {
        local_terrain_support_octree_->clear();
    }
    if (!kdtree_terrain_clould_) {
        kdtree_terrain_clould_.reset(new pcl::KdTreeFLANN<PCLPoint>());
    }
    FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
}

void MapHandler::RefreshLocalTerrainSupportOctomap() {
    if (!semantic_tree_snapshot_ || !is_init_) return;

    const auto* semantic_tree =
        dynamic_cast<const SemanticOctree*>(semantic_tree_snapshot_.get());
    const auto* color_tree =
        dynamic_cast<const octomap::ColorOcTree*>(semantic_tree_snapshot_.get());
    if (!semantic_tree && !color_tree) {
        if (local_terrain_support_octree_) local_terrain_support_octree_->clear();
        if (semantic_obs_cloud_) semantic_obs_cloud_->clear();
        // Do not erase persistent static collision memory merely because one
        // message cannot be decoded. Only Reset or explicit-free evidence is
        // authorised to remove it.
        if (semantic_terrain_support_cloud_) semantic_terrain_support_cloud_->clear();
        if (current_dynamic_obs_cloud_) current_dynamic_obs_cloud_->clear();
        if (effective_dynamic_obs_cloud_) effective_dynamic_obs_cloud_->clear();
        if (collision_obs_cloud_) collision_obs_cloud_->clear();
        if (dynamic_added_cloud_) dynamic_added_cloud_->clear();
        if (dynamic_removed_cloud_) dynamic_removed_cloud_->clear();
        if (changed_obs_cloud_) changed_obs_cloud_->clear();
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
        return;
    }

    const double source_resolution = semantic_tree_snapshot_->getResolution();
    if (!local_terrain_support_octree_ || std::abs(local_terrain_support_octree_->getResolution() - source_resolution) > 1e-6) {
        local_terrain_support_octree_.reset(new octomap::OcTree(source_resolution));
    }
    local_terrain_support_octree_->clear();

    const float horizontal_half_extent = semantic_params_.local_window_radius;
    const float vertical_half_extent = std::max(
        std::max(FARUtil::kTolerZ, FARUtil::kCellHeight * 2.0f),
        FARUtil::vehicle_height + static_cast<float>(source_resolution));
    const QueryBox local_box{
        point3d(robot_pos_cache_.x - horizontal_half_extent,
                robot_pos_cache_.y - horizontal_half_extent,
                robot_pos_cache_.z - vertical_half_extent),
        point3d(robot_pos_cache_.x + horizontal_half_extent,
                robot_pos_cache_.y + horizontal_half_extent,
                robot_pos_cache_.z + vertical_half_extent)};

    std::unordered_map<uint64_t, PCLPoint> current_obs_voxels;
    std::unordered_map<uint64_t, PCLPoint> current_dynamic_voxels;

    if (semantic_tree) {
        const auto get_semantic_color = [](const SemanticOctree::leaf_bbx_iterator& it) {
            const SemanticOcTreeNode* node = it.operator->();
            return node->isSemanticsSet()
                ? node->getSemantics().getSemanticColor()
                : node->getColor();
        };
        ExtractClassifiedCloudInBox(*semantic_tree, local_box, obstacle_groups_,
                                    get_semantic_color, semantic_obs_cloud_);
        ExtractClassifiedCloudInBox(*semantic_tree, local_box,
                                    dynamic_obstacle_groups_,
                                    get_semantic_color,
                                    current_dynamic_obs_cloud_);
        ExtractClassifiedCloudInBox(*semantic_tree, local_box, terrain_support_groups_,
                                    get_semantic_color, semantic_terrain_support_cloud_);
        BuildDenseVoxelMap(*semantic_tree, semantic_obs_cloud_, current_obs_voxels);
        BuildDenseVoxelMap(*semantic_tree, current_dynamic_obs_cloud_,
                           current_dynamic_voxels);
    } else {
        const auto get_color = [](const octomap::ColorOcTree::leaf_bbx_iterator& it) {
            return it->getColor();
        };
        ExtractClassifiedCloudInBox(*color_tree, local_box, obstacle_groups_,
                                    get_color, semantic_obs_cloud_);
        ExtractClassifiedCloudInBox(*color_tree, local_box,
                                    dynamic_obstacle_groups_, get_color,
                                    current_dynamic_obs_cloud_);
        ExtractClassifiedCloudInBox(*color_tree, local_box, terrain_support_groups_,
                                    get_color, semantic_terrain_support_cloud_);
        BuildDenseVoxelMap(*color_tree, semantic_obs_cloud_, current_obs_voxels);
        BuildDenseVoxelMap(*color_tree, current_dynamic_obs_cloud_,
                           current_dynamic_voxels);
    }

    // Navigation-corner matching may simplify, merge or retire a vertex, but
    // it must never delete the physical wall used by later visibility checks.
    // Maintain that static collision authority once per accepted semantic
    // snapshot, independently of the Graph topology.
    this->UpdatePersistentStaticObstacleLayer();

    dynamic_added_cloud_->clear();
    dynamic_removed_cloud_->clear();
    for (const auto& entry : current_dynamic_voxels) {
        if (previous_local_dynamic_voxels_.count(entry.first) == 0) {
            dynamic_added_cloud_->points.push_back(entry.second);
        }
    }
    // The map builder owns dynamic-object tracking. FAR treats the newest
    // robot-local semantic snapshot as the sole source of truth: a dynamic
    // voxel that disappeared, was reclassified, became unknown/free, or left
    // the moving query window is removed in this same update. Every
    // previous-minus-current point must be emitted so the changed-point
    // KD-tree invalidates its old contour and Graph connections, including
    // points that have just crossed the local-window boundary.
    for (const auto& entry : previous_local_dynamic_voxels_) {
        if (current_dynamic_voxels.count(entry.first) == 0) {
            dynamic_removed_cloud_->points.push_back(entry.second);
        }
    }
    FinalizeCloud(dynamic_added_cloud_);
    FinalizeCloud(dynamic_removed_cloud_);

    // FAR adds no downstream persistence or clearance timer.
    *effective_dynamic_obs_cloud_ = *current_dynamic_obs_cloud_;

    collision_obs_cloud_->clear();
    AppendCloud(semantic_obs_cloud_, collision_obs_cloud_);
    AppendCloud(effective_dynamic_obs_cloud_, collision_obs_cloud_);
    FinalizeCloud(collision_obs_cloud_);

    changed_obs_cloud_->clear();
    for (const auto& entry : current_obs_voxels) {
        if (previous_local_obs_voxels_.count(entry.first) == 0) {
            changed_obs_cloud_->points.push_back(entry.second);
        }
    }
    for (const auto& entry : previous_local_obs_voxels_) {
        if (current_obs_voxels.count(entry.first) == 0) {
            changed_obs_cloud_->points.push_back(entry.second);
        }
    }
    AppendCloud(dynamic_added_cloud_, changed_obs_cloud_);
    AppendCloud(dynamic_removed_cloud_, changed_obs_cloud_);
    FinalizeCloud(changed_obs_cloud_);
    previous_local_obs_voxels_.swap(current_obs_voxels);
    previous_local_dynamic_voxels_.swap(current_dynamic_voxels);

    for (const auto& point : semantic_terrain_support_cloud_->points) {
        local_terrain_support_octree_->updateNode(
            point3d(point.x, point.y, point.z), true);
    }

    local_terrain_support_octree_->prune();
    if (local_terrain_support_octree_->size() == 0) {
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
        return;
    }

    this->AssignFlatTerrainCloud(semantic_terrain_support_cloud_, flat_terrain_cloud_);
    kdtree_terrain_clould_->setInputCloud(flat_terrain_cloud_);
}


bool MapHandler::SetSemanticOctomap(const octomap_msgs::OctomapConstPtr& msg) {
    if (!msg) return false;

    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*msg));
    if (!tree) {
        ROS_WARN_THROTTLE(1.0, "MH: failed to deserialize semantic octomap message.");
        return false;
    }

    if (!dynamic_cast<SemanticOctree*>(tree.get()) &&
        !dynamic_cast<octomap::ColorOcTree*>(tree.get())) {
        ROS_WARN_THROTTLE(1.0, "MH: octomap type has no supported semantic colors.");
        return false;
    }
    if (!msg->header.frame_id.empty() && !FARUtil::worldFrameId.empty() &&
        !FARUtil::IsSameFrameID(msg->header.frame_id, FARUtil::worldFrameId)) {
        ROS_ERROR_THROTTLE(1.0,
            "MH: semantic octomap frame does not match the planner world frame.");
        return false;
    }

    const double previous_resolution = semantic_tree_snapshot_
        ? semantic_tree_snapshot_->getResolution() : tree->getResolution();
    if (std::fabs(previous_resolution - tree->getResolution()) > 1e-6) {
        previous_local_obs_voxels_.clear();
        previous_local_dynamic_voxels_.clear();
    }

    semantic_tree_snapshot_ = std::move(tree);
    semantic_stamp_ = msg->header.stamp;
    semantic_frame_id_ = msg->header.frame_id;
    has_semantic_map_ = true;
    if (is_init_) this->RefreshLocalTerrainSupportOctomap();
    return true;
}

void MapHandler::ResetGripMapCloud() {
    semantic_tree_snapshot_.reset();
    semantic_stamp_ = ros::Time();
    semantic_frame_id_.clear();
    if (semantic_obs_cloud_) semantic_obs_cloud_->clear();
    if (persistent_static_obs_cloud_) persistent_static_obs_cloud_->clear();
    if (semantic_terrain_support_cloud_) semantic_terrain_support_cloud_->clear();
    if (current_dynamic_obs_cloud_) current_dynamic_obs_cloud_->clear();
    if (effective_dynamic_obs_cloud_) effective_dynamic_obs_cloud_->clear();
    if (collision_obs_cloud_) collision_obs_cloud_->clear();
    if (dynamic_added_cloud_) dynamic_added_cloud_->clear();
    if (dynamic_removed_cloud_) dynamic_removed_cloud_->clear();
    if (changed_obs_cloud_) changed_obs_cloud_->clear();
    previous_local_obs_voxels_.clear();
    previous_local_dynamic_voxels_.clear();
    persistent_static_obs_voxels_.clear();
    if (!flat_terrain_cloud_) {
        flat_terrain_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    } else {
        flat_terrain_cloud_->clear();
    }
    if (local_terrain_support_octree_) local_terrain_support_octree_->clear();
    has_semantic_map_ = false;
    if (kdtree_terrain_clould_) {
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
    }
}

// semantic-only 兼容接口：射线清障已由上游八叉树占用概率更新负责。
void MapHandler::ClearObsCellThroughPosition(const Point3D& point) {
    (void)point;
}

void MapHandler::GetCloudOfPoint(const Point3D& center, const PointCloudPtr& cloudOut, const CloudType& type, const bool& is_large)
{
    if (!cloudOut) return;
    cloudOut->clear();
    if (!has_semantic_map_ || !semantic_tree_snapshot_) return;
    if (type != CloudType::OBS_CLOUD && type != CloudType::FREE_CLOUD) {
        if (FARUtil::IsDebug) ROS_ERROR("MH: Assigned cloud type invalid.");
        return;
    }

    const float horizontal_half_extent = is_large
        ? semantic_params_.local_window_radius
        : semantic_params_.local_window_radius * 0.5f;
    if (horizontal_half_extent <= 0.0f) return;
    const float vertical_half_extent = std::max(
        std::max(FARUtil::kTolerZ, FARUtil::kCellHeight * 2.0f),
        FARUtil::vehicle_height +
            static_cast<float>(semantic_tree_snapshot_->getResolution()));
    const QueryBox query_box{
        point3d(center.x - horizontal_half_extent,
                center.y - horizontal_half_extent,
                center.z - vertical_half_extent),
        point3d(center.x + horizontal_half_extent,
                center.y + horizontal_half_extent,
                center.z + vertical_half_extent)};

    const auto* semantic_tree =
        dynamic_cast<const SemanticOctree*>(semantic_tree_snapshot_.get());
    const auto* color_tree =
        dynamic_cast<const octomap::ColorOcTree*>(semantic_tree_snapshot_.get());
    if (!semantic_tree && !color_tree) return;

    const auto& groups = type == CloudType::OBS_CLOUD
        ? obstacle_groups_ : terrain_support_groups_;
    if (semantic_tree) {
        const auto get_semantic_color = [](const SemanticOctree::leaf_bbx_iterator& it) {
            const SemanticOcTreeNode* node = it.operator->();
            return node->isSemanticsSet()
                ? node->getSemantics().getSemanticColor()
                : node->getColor();
        };
        ExtractClassifiedCloudInBox(*semantic_tree, query_box, groups,
                                    get_semantic_color, cloudOut);
    } else {
        const auto get_color = [](const octomap::ColorOcTree::leaf_bbx_iterator& it) {
            return it->getColor();
        };
        ExtractClassifiedCloudInBox(*color_tree, query_box, groups,
                                    get_color, cloudOut);
    }
}

// 设置随机器人移动的局部语义查询中心。
void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    robot_pos_cache_ = ori_robot_pos;
    is_init_ = true;
}
// 移动局部查询窗并从已验证快照重建障碍/地面派生缓存。
void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos);
    robot_pos_cache_ = odom_pos;
    if (has_semantic_map_) {
        this->RefreshLocalTerrainSupportOctomap();
    }
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    if (!obsCloudOut) return;
    // FAR's original incremental graph update is driven by the obstacle
    // contours in this cloud.  Feed it the effective collision view so a
    // currently observed dynamic obstacle creates local contour vertices and
    // invalidates intersecting connections.  Dynamic additions/removals are
    // also emitted through changed_obs_cloud_, so old contour vertices are
    // re-evaluated in the same update when the latest snapshot clears them or
    // they leave the moving local window.
    if (!collision_obs_cloud_) {
        obsCloudOut->clear();
        return;
    }
    *obsCloudOut = *collision_obs_cloud_;
}

void MapHandler::GetCurrentStaticObsCloud(
    const PointCloudPtr& obsCloudOut) const {
    if (!obsCloudOut) return;
    if (!semantic_obs_cloud_) {
        obsCloudOut->clear();
        return;
    }
    *obsCloudOut = *semantic_obs_cloud_;
}

void MapHandler::GetPersistentStaticObsCloud(
    const PointCloudPtr& obsCloudOut) const {
    if (!obsCloudOut) return;
    if (!persistent_static_obs_cloud_) {
        obsCloudOut->clear();
        return;
    }
    *obsCloudOut = *persistent_static_obs_cloud_;
}

StaticNodeEvidence MapHandler::QueryStaticTreeEvidence(
    const Point3D& point) const {
    if (!semantic_tree_snapshot_) return StaticNodeEvidence::UNKNOWN;
    const auto* semantic_tree =
        dynamic_cast<const SemanticOctree*>(semantic_tree_snapshot_.get());
    const auto* color_tree = dynamic_cast<const octomap::ColorOcTree*>(
        semantic_tree_snapshot_.get());
    if (!semantic_tree && !color_tree) return StaticNodeEvidence::UNKNOWN;

    const float resolution = static_cast<float>(
        semantic_tree_snapshot_->getResolution());
    int known_free_samples = 0;
    const float z_samples[] = {
        point.z,
        point.z - FARUtil::vehicle_height * 0.5f,
        point.z + resolution * 0.5f};
    for (const float z : z_samples) {
        const point3d query(point.x, point.y, z);
        if (semantic_tree) {
            const SemanticOcTreeNode* node = semantic_tree->search(query);
            if (!node) continue;
            if (!semantic_tree->isNodeOccupied(node)) {
                ++known_free_samples;
                continue;
            }
            const ColorOcTreeNode::Color color = node->isSemanticsSet()
                ? node->getSemantics().getSemanticColor()
                : node->getColor();
            const uint32_t rgb = MakeRgbKey(color.r, color.g, color.b);
            if (MatchRgbKey(obstacle_groups_, rgb)) {
                return StaticNodeEvidence::STATIC_OCCUPIED;
            }
            if (MatchRgbKey(dynamic_obstacle_groups_, rgb)) {
                return StaticNodeEvidence::UNKNOWN;
            }
            // local_grid stores traversable semantic cells as occupied
            // endpoints because they are terrain observations, not sensor-ray
            // free voxels.  For the lifetime of an old *obstacle* cell this is
            // nevertheless authoritative explicit-free evidence.
            if (MatchRgbKey(terrain_support_groups_, rgb)) {
                return StaticNodeEvidence::EXPLICIT_FREE;
            }
        } else {
            const octomap::ColorOcTreeNode* node = color_tree->search(query);
            if (!node) continue;
            if (!color_tree->isNodeOccupied(node)) {
                ++known_free_samples;
                continue;
            }
            const auto color = node->getColor();
            const uint32_t rgb = MakeRgbKey(color.r, color.g, color.b);
            if (MatchRgbKey(obstacle_groups_, rgb)) {
                return StaticNodeEvidence::STATIC_OCCUPIED;
            }
            if (MatchRgbKey(dynamic_obstacle_groups_, rgb)) {
                return StaticNodeEvidence::UNKNOWN;
            }
            if (MatchRgbKey(terrain_support_groups_, rgb)) {
                return StaticNodeEvidence::EXPLICIT_FREE;
            }
        }
    }
    return known_free_samples >= 2
        ? StaticNodeEvidence::EXPLICIT_FREE
        : StaticNodeEvidence::UNKNOWN;
}

void MapHandler::UpdatePersistentStaticObstacleLayer() {
    if (!persistent_static_obs_cloud_) {
        persistent_static_obs_cloud_.reset(new pcl::PointCloud<PCLPoint>());
    }
    const float resolution = std::max(1e-3f, FARUtil::kLeafSize);
    std::unordered_set<uint64_t> current_keys;
    if (semantic_obs_cloud_) {
        current_keys.reserve(semantic_obs_cloud_->size());
        for (const auto& point : semantic_obs_cloud_->points) {
            const uint64_t key = PersistentStaticKey(point, resolution);
            current_keys.insert(key);
            persistent_static_obs_voxels_[key] =
                PersistentStaticCellCenter(point, resolution);
        }
    }

    std::size_t explicit_free_removed = 0;
    for (auto it = persistent_static_obs_voxels_.begin();
         it != persistent_static_obs_voxels_.end();) {
        const PCLPoint& point = it->second;
        const bool in_current_square =
            std::abs(point.x - robot_pos_cache_.x) <=
                semantic_params_.local_window_radius &&
            std::abs(point.y - robot_pos_cache_.y) <=
                semantic_params_.local_window_radius;
        if (in_current_square && current_keys.count(it->first) == 0) {
            const Point3D query(point.x, point.y, point.z);
            if (QueryStaticTreeEvidence(query) ==
                StaticNodeEvidence::EXPLICIT_FREE) {
                it = persistent_static_obs_voxels_.erase(it);
                ++explicit_free_removed;
                continue;
            }
        }
        ++it;
    }

    persistent_static_obs_cloud_->clear();
    persistent_static_obs_cloud_->reserve(
        persistent_static_obs_voxels_.size());
    for (const auto& entry : persistent_static_obs_voxels_) {
        persistent_static_obs_cloud_->points.push_back(entry.second);
    }
    FinalizeCloud(persistent_static_obs_cloud_);
    ROS_INFO_THROTTLE(
        5.0,
        "MH persistent static collision layer: cells=%zu current=%zu explicit_free_removed=%zu resolution=%.2fm",
        persistent_static_obs_voxels_.size(), current_keys.size(),
        explicit_free_removed, resolution);
}

StaticNodeEvidence MapHandler::QueryStaticNodeEvidence(
    const Point3D& point) const {
    if (!has_semantic_map_ || !semantic_tree_snapshot_ || !is_init_) {
        return StaticNodeEvidence::UNKNOWN;
    }
    const float dx = point.x - robot_pos_cache_.x;
    const float dy = point.y - robot_pos_cache_.y;
    // SetSemanticOctomap extracts an axis-aligned square BBX. Evidence must
    // use the same footprint; a radial test incorrectly labelled the square's
    // visible corner regions as UNKNOWN.
    if (std::abs(dx) > semantic_params_.local_window_radius ||
        std::abs(dy) > semantic_params_.local_window_radius) {
        return StaticNodeEvidence::UNKNOWN;
    }

    const float resolution = static_cast<float>(
        semantic_tree_snapshot_->getResolution());
    const float horizontal_radius = std::max(
        resolution * 1.25f, FARUtil::kLeafSize);
    const float vertical_radius = FARUtil::vehicle_height + resolution;
    if (semantic_obs_cloud_) {
        for (const auto& sample : semantic_obs_cloud_->points) {
            if (std::hypot(sample.x - point.x, sample.y - point.y) <=
                    horizontal_radius &&
                std::abs(sample.z - point.z) <= vertical_radius) {
                return StaticNodeEvidence::STATIC_OCCUPIED;
            }
        }
    }

    return QueryStaticTreeEvidence(point);
}

void MapHandler::GetCollisionObsCloud(const PointCloudPtr& obsCloudOut) const {
    if (!obsCloudOut) return;
    if (!collision_obs_cloud_) {
        obsCloudOut->clear();
        return;
    }
    *obsCloudOut = *collision_obs_cloud_;
}

void MapHandler::GetCurrentDynamicObsCloud(const PointCloudPtr& obsCloudOut) const {
    if (!obsCloudOut) return;
    if (!current_dynamic_obs_cloud_) {
        obsCloudOut->clear();
        return;
    }
    *obsCloudOut = *current_dynamic_obs_cloud_;
}

void MapHandler::GetEffectiveDynamicObsCloud(const PointCloudPtr& obsCloudOut) const {
    if (!obsCloudOut) return;
    if (!effective_dynamic_obs_cloud_) {
        obsCloudOut->clear();
        return;
    }
    *obsCloudOut = *effective_dynamic_obs_cloud_;
}

void MapHandler::BuildLocalPlannerObstacleCloud(
    const PointCloudPtr& source, const PointCloudPtr& cloudOut) const {
    if (!cloudOut) return;
    cloudOut->clear();
    if (!source || source->empty() || !semantic_tree_snapshot_ || !is_init_) {
        FinalizeCloud(cloudOut);
        return;
    }

    const float radius = semantic_params_.local_planner_radius;
    const float output_resolution = semantic_params_.local_planner_resolution;
    if (radius <= 0.0f || output_resolution <= 0.0f) {
        FinalizeCloud(cloudOut);
        return;
    }

    bool terrain_associated = false;
    const float ground_height = NearestTerrainHeightofNavPoint(
        robot_pos_cache_, terrain_associated);
    const float source_resolution = static_cast<float>(
        semantic_tree_snapshot_->getResolution());
    const float source_half = source_resolution * 0.5f;
    // Keep any occupied voxel whose vertical interval intersects the robot's
    // swept body band. The terrain-analysis local planner then performs its
    // collision lookup in XY, so repeated wall levels are deliberately
    // collapsed below.
    const float body_min_z = ground_height - source_half;
    const float body_max_z = ground_height + FARUtil::vehicle_height + source_half;
    const float radius_with_voxel = radius + source_half;
    const int samples_per_axis = std::max(
        1, static_cast<int>(std::ceil(source_resolution / output_resolution)));
    const float sample_span = samples_per_axis * output_resolution;

    std::unordered_set<uint64_t> occupied_xy;
    for (const auto& source_point : source->points) {
        const float dx = source_point.x - robot_pos_cache_.x;
        const float dy = source_point.y - robot_pos_cache_.y;
        if (dx * dx + dy * dy > radius_with_voxel * radius_with_voxel) continue;
        if (source_point.z + source_half < body_min_z ||
            source_point.z - source_half > body_max_z) continue;

        const float first_x = source_point.x - sample_span * 0.5f +
                              output_resolution * 0.5f;
        const float first_y = source_point.y - sample_span * 0.5f +
                              output_resolution * 0.5f;
        for (int ix = 0; ix < samples_per_axis; ++ix) {
            for (int iy = 0; iy < samples_per_axis; ++iy) {
                const float x = first_x + ix * output_resolution;
                const float y = first_y + iy * output_resolution;
                const float qdx = x - robot_pos_cache_.x;
                const float qdy = y - robot_pos_cache_.y;
                if (qdx * qdx + qdy * qdy > radius * radius) continue;
                const int32_t qx = static_cast<int32_t>(
                    std::floor(x / output_resolution));
                const int32_t qy = static_cast<int32_t>(
                    std::floor(y / output_resolution));
                const uint64_t key =
                    (static_cast<uint64_t>(static_cast<uint32_t>(qx)) << 32) |
                    static_cast<uint32_t>(qy);
                if (!occupied_xy.insert(key).second) continue;

                PCLPoint output;
                output.x = x;
                output.y = y;
                output.z = ground_height;
                output.intensity =
                    semantic_params_.local_planner_obstacle_intensity;
                cloudOut->points.push_back(output);
            }
        }
    }
    FinalizeCloud(cloudOut);
}

void MapHandler::GetLocalPlannerStaticObsCloud(
    const PointCloudPtr& cloudOut) const {
    BuildLocalPlannerObstacleCloud(semantic_obs_cloud_, cloudOut);
}

void MapHandler::GetLocalPlannerDynamicObsCloud(
    const PointCloudPtr& cloudOut) const {
    BuildLocalPlannerObstacleCloud(effective_dynamic_obs_cloud_, cloudOut);
}

void MapHandler::GetDynamicAddedCloud(const PointCloudPtr& cloudOut) const {
    if (!cloudOut) return;
    if (!dynamic_added_cloud_) {
        cloudOut->clear();
        return;
    }
    *cloudOut = *dynamic_added_cloud_;
}

void MapHandler::GetDynamicRemovedCloud(const PointCloudPtr& cloudOut) const {
    if (!cloudOut) return;
    if (!dynamic_removed_cloud_) {
        cloudOut->clear();
        return;
    }
    *cloudOut = *dynamic_removed_cloud_;
}

void MapHandler::GetChangedObsCloud(const PointCloudPtr& changedCloudOut) const {
    if (!changedCloudOut) return;
    if (!changed_obs_cloud_) {
        changedCloudOut->clear();
        return;
    }
    *changedCloudOut = *changed_obs_cloud_;
}

// 以下更新接口仅为下游兼容保留；全局占用状态只由语义八叉树维护。
void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
    (void)obsCloudInOut;
}

void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
    (void)freeCloudIn;
}

// 在“查询某个点的地面高度”，并通过 is_matched 告诉你是否直接匹配成功。
float MapHandler::TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search) {
    is_matched = false;
    if (!local_terrain_support_octree_ ||
        local_terrain_support_octree_->size() == 0) return p.z;

    const Point3D expected_ground(p.x, p.y,
                                  p.z - FARUtil::vehicle_height);
    const float exact_column_radius = std::max(
        static_cast<float>(local_terrain_support_octree_->getResolution()) * 0.75f,
        1e-3f);
    float distance_square = FARUtil::kINF;
    float terrain_height = NearestHeightOfPoint(
        expected_ground, distance_square, exact_column_radius);
    if (distance_square < FARUtil::kINF) {
        is_matched = true;
        return terrain_height;
    }

    if (is_search) {
        terrain_height = NearestHeightOfPoint(
            expected_ground, distance_square, terrain_search_radius_);
        if (distance_square < FARUtil::kINF) {
            is_matched = true;
            return terrain_height;
        }
    }
    return p.z;
}

// 获取导航点最近的地形高度。
float MapHandler::NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated) {
    const float fallback_height = point.z - FARUtil::vehicle_height;
    is_associated = false;
    if (!local_terrain_support_octree_ ||
        local_terrain_support_octree_->size() == 0 ||
        !kdtree_terrain_clould_ ||
        !kdtree_terrain_clould_->getInputCloud() ||
        kdtree_terrain_clould_->getInputCloud()->empty()) {
        return fallback_height;
    }

    const Point3D expected_ground(point.x, point.y, fallback_height);
    const float exact_column_radius = std::max(
        static_cast<float>(local_terrain_support_octree_->getResolution()) * 0.75f,
        1e-3f);
    float distance_square = FARUtil::kINF;
    float terrain_height = NearestHeightOfPoint(
        expected_ground, distance_square, exact_column_radius);
    if (distance_square >= FARUtil::kINF) {
        terrain_height = NearestHeightOfPoint(
            expected_ground, distance_square, terrain_search_radius_);
    }
    if (distance_square < FARUtil::kINF) {
        is_associated = true;
        return terrain_height;
    }
    return fallback_height;
}


// 判断导航点的脚底高度是否位于局部地形高度带内。
bool MapHandler::IsNavPointOnTerrainNeighbor(const Point3D& point, const bool& is_extend) {
    if (!local_terrain_support_octree_ ||
        local_terrain_support_octree_->size() == 0) {
        return false;
    }

    // 原实现查询的是导航点减去车体高度后的脚底位置，而不是语义障碍占据状态。
    const Point3D ground_point(point.x, point.y,
                               point.z - FARUtil::vehicle_height);

    // ObsNeighborCloudWithTerrain 在半格对角线范围内取得 minH/maxH。
    // 这里直接从局部语义地形缓存重建同一个高度带，不再恢复旧 Grid 类。
    const float terrain_radius = std::max(terrain_neighbor_radius_, 1e-3f);
    float min_height = ground_point.z;
    float max_height = ground_point.z;
    bool in_range = false;
    NearestHeightOfRadius(ground_point, terrain_radius,
                          min_height, max_height, in_range);

    if (!in_range) return false;

    const float cell_height = FARUtil::kCellHeight > 0.0f
        ? FARUtil::kCellHeight
        : static_cast<float>(local_terrain_support_octree_->getResolution());
    float lower_bound = min_height - cell_height;
    if (is_extend) {
        // 原 extend_obs_indices_ 只采用 z 偏移 {-1, 0}，即向下多扩一层。
        lower_bound -= cell_height;
    }
    const float upper_bound = max_height + FARUtil::kTolerZ + cell_height;
    return ground_point.z > lower_bound && ground_point.z < upper_bound;
}

// 把一组导航节点的 z 高度“贴地修正”，让节点高度和当前地形更一致，同时避免改到不该改的节点。
void MapHandler::AdjustNodesHeight(const NodePtrStack& nodes) {
    if (nodes.empty()) return;
    for (const auto& node_ptr : nodes) {
        if (!node_ptr) continue;
        const float dx = node_ptr->position.x - robot_pos_cache_.x;
        const float dy = node_ptr->position.y - robot_pos_cache_.y;
        const bool in_semantic_square =
            std::abs(dx) <= semantic_params_.local_window_radius &&
            std::abs(dy) <= semantic_params_.local_window_radius &&
            FARUtil::IsPointInToleratedHeight(
                node_ptr->position, FARUtil::kTolerZ + FARUtil::kHeightVoxel);
        if (!node_ptr->is_active || node_ptr->is_boundary || FARUtil::IsFreeNavNode(node_ptr) || FARUtil::IsOutsideGoal(node_ptr) || !in_semantic_square) {
            continue;
        } 
        bool is_match = false;
        float terrain_h = TerrainHeightOfPoint(node_ptr->position, is_match, false);
        if (is_match) {
            terrain_h += FARUtil::vehicle_height;
            if (node_ptr->pos_filter_vec.empty()) {
                node_ptr->position.z = terrain_h;
            } else {
                node_ptr->pos_filter_vec.back().z = terrain_h; // assign to position filter
                node_ptr->position.z = FARUtil::AveragePoints(node_ptr->pos_filter_vec).z;
            }
        }
    }
}

// CTNode 做高度修正的，目标是让它们“贴地”且不要偏离机器人当前高度太多。
// 贴地修正：把 CT 节点的高度改成附近地形的估计高度，避免节点在斜坡、台阶或地表起伏上看起来“漂浮”。
// 保持可行性：修正后的高度会再限制在机器人当前高度附近的容差范围内，防止因为地形误差导致节点过高或过低。
// 供后续规划使用：后面的路径/可通行性判断会基于这个新的高度来判断是否可走、是否碰撞。
void MapHandler::AdjustCTNodeHeight(const CTNodeStack& ctnodes) {
    if (ctnodes.empty()) return;
    const float H_MAX = FARUtil::robot_pos.z + FARUtil::kTolerZ;
    const float H_MIN = FARUtil::robot_pos.z - FARUtil::kTolerZ;
    for (auto& ctnode_ptr : ctnodes) {
        if (!ctnode_ptr) continue;

        bool is_matched = false;
        float terrain_height = TerrainHeightOfPoint(
            ctnode_ptr->position, is_matched, false);

        if (!is_matched) {
            float nearest_dist_square = FARUtil::kINF;
            const Point3D expected_ground(
                ctnode_ptr->position.x, ctnode_ptr->position.y,
                ctnode_ptr->position.z - FARUtil::vehicle_height);
            terrain_height = NearestHeightOfPoint(
                expected_ground, nearest_dist_square, terrain_search_radius_);
            is_matched = nearest_dist_square < FARUtil::kINF;
        }

        ctnode_ptr->is_ground_associate = is_matched;
        if (!is_matched) continue;

        const float adjusted_height = terrain_height + FARUtil::vehicle_height;
        ctnode_ptr->position.z = std::max(
            std::min(adjusted_height, H_MAX), H_MIN);
    }
}

void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn,
                                         const PointCloudPtr& terrainHeightOut) {
    (void)freeCloudIn;
    (void)terrainHeightOut;
}

void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    (void)neighbor_centers;
}

void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    (void)occupancy_centers;
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    (void)obsCloud;
}
