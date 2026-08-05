/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/map_handler.h"
#include <cmath>
#include <octomap/OcTree.h>
#include <sstream>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

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

inline void FillCloudFromOccupiedLeaves(const octomap::OcTree& tree, PointCloudPtr& cloud_out) {
    cloud_out->clear();
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (!tree.isNodeOccupied(*it)) continue;
        const octomap::point3d coord = it.getCoordinate();
        PCLPoint point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        point.intensity = 0.0f;
        cloud_out->points.push_back(point);
    }
}

inline void CropCloudAroundCenter(const PointCloudPtr& cloud_in,
                                  const Point3D& center,
                                  const float radius,
                                  PointCloudPtr& cloud_out) {
    cloud_out->clear();
    if (!cloud_in || cloud_in->empty()) return;
    const float r2 = radius * radius;
    for (const auto& point : cloud_in->points) {
        const float dx = point.x - center.x;
        const float dy = point.y - center.y;
        const float dz = point.z - center.z;
        if (dx * dx + dy * dy + dz * dz <= r2) {
            cloud_out->points.push_back(point);
        }
    }
}

}  // namespace

void MapHandler::CopyOccupancyTree(const octomap::AbstractOcTree& source_tree,
                                   std::shared_ptr<octomap::OcTree>& target_tree) const {
    if (!target_tree || std::abs(target_tree->getResolution() - source_tree.getResolution()) > 1e-6) {
        target_tree.reset(new octomap::OcTree(source_tree.getResolution()));
    }

    std::stringstream tree_stream;
    const auto* occupancy_tree = dynamic_cast<const octomap::AbstractOccupancyOcTree*>(&source_tree);
    if (!occupancy_tree) {
        target_tree->clear();
        return;
    }

    occupancy_tree->writeBinaryData(tree_stream);
    target_tree->clear();
    target_tree->readBinaryData(tree_stream);
}

void MapHandler::RebuildDerivedOctomapCachesFromSemanticTree(const octomap::AbstractOcTree& tree) {
    this->CopyOccupancyTree(tree, semantic_obs_octree_);
    this->CopyOccupancyTree(tree, semantic_terrain_support_octree_);

    const auto* semantic_tree = dynamic_cast<const SemanticOctree*>(&tree);
    if (semantic_tree != nullptr) {
        for (auto it = semantic_tree->begin_leafs(), end = semantic_tree->end_leafs(); it != end; ++it) {
            if (!semantic_tree->isNodeOccupied(*it)) continue;
            const SemanticOcTreeNode* node = it.operator->();
            if (!node) continue;

            ColorOcTreeNode::Color semantic_color = node->isSemanticsSet()
                ? node->getSemantics().getSemanticColor()
                : node->getColor();
            const uint32_t rgb_key = MakeRgbKey(semantic_color.r, semantic_color.g, semantic_color.b);
            const octomap::OcTreeKey key = it.getKey();
            const unsigned int depth = it.getDepth();

            if (MatchRgbKey(obstacle_groups_, rgb_key)) {
                semantic_terrain_support_octree_->deleteNode(key, depth);
                continue;
            }
            if (MatchRgbKey(terrain_support_groups_, rgb_key)) {
                semantic_obs_octree_->deleteNode(key, depth);
                continue;
            }

            semantic_terrain_support_octree_->deleteNode(key, depth);
        }
        semantic_obs_octree_->prune();
        semantic_terrain_support_octree_->prune();
        return;
    }

    const auto* color_tree = dynamic_cast<const octomap::ColorOcTree*>(&tree);
    if (!color_tree) return;

    for (auto it = color_tree->begin_leafs(), end = color_tree->end_leafs(); it != end; ++it) {
        if (!color_tree->isNodeOccupied(*it)) continue;

        const auto color = color_tree->getNodeColor(*it);
        const uint32_t rgb_key = MakeRgbKey(color.r, color.g, color.b);
        const octomap::OcTreeKey key = it.getKey();
        const unsigned int depth = it.getDepth();

        if (MatchRgbKey(obstacle_groups_, rgb_key)) {
            semantic_terrain_support_octree_->deleteNode(key, depth);
            continue;
        }
        if (MatchRgbKey(terrain_support_groups_, rgb_key)) {
            semantic_obs_octree_->deleteNode(key, depth);
            continue;
        }

        semantic_terrain_support_octree_->deleteNode(key, depth);
    }

    semantic_obs_octree_->prune();
    semantic_terrain_support_octree_->prune();
}


void MapHandler::SetSemanticOctomap(const octomap_msgs::OctomapConstPtr& msg) {
    if (!msg) return;
    semantic_map_msg_ = msg;
    semantic_stamp_ = msg->header.stamp;
    semantic_frame_id_ = msg->header.frame_id;
    has_semantic_map_ = true;

    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*semantic_map_msg_));
    if (!tree) {
        ROS_WARN_THROTTLE(1.0, "MH: failed to deserialize semantic octomap message.");
        semantic_obs_cloud_->clear();
        semantic_terrain_support_cloud_->clear();
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
        return;
    }

    this->RebuildDerivedOctomapCachesFromSemanticTree(*tree);
}

void MapHandler::ResetGripMapCloud() {
    semantic_obs_cloud_->clear();
    semantic_terrain_support_cloud_->clear();
    if (semantic_obs_octree_) semantic_obs_octree_->clear();
    if (semantic_terrain_support_octree_) semantic_terrain_support_octree_->clear();
    has_semantic_map_ = false;
    FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
}

// 沿着“机器人当前位置 → 给定点 point”的射线，把这条线上附近高度层里的障碍栅格清空，相当于做一条“视线清障”。octreemap 自带功能。
void MapHandler::ClearObsCellThroughPosition(const Point3D& point) {
    (void)point;
}

// is_large 就是控制“水平范围是否扩大一圈”的开关
// 实现逻辑，因为这个是在找整个全局地图中的邻域点，所以需要遍历帧整个八叉树地图，找到八叉树内不同属性的点集。

int getSphereBoxState(const point3d& node_center, double node_size, 
                      const point3d& sphere_center, double radius) {
    double half_size = node_size / 2.0;
    double r_sq = radius * radius;
    
    // 1. 判断是否完全在球形区域内 (计算正方体上距离球心最远点的距离)
    double dx_max = std::fabs(sphere_center.x() - node_center.x()) + half_size;
    double dy_max = std::fabs(sphere_center.y() - node_center.y()) + half_size;
    double dz_max = std::fabs(sphere_center.z() - node_center.z()) + half_size;
    double dist_max_sq = dx_max*dx_max + dy_max*dy_max + dz_max*dz_max;
    
    if (dist_max_sq <= r_sq) {
        return 1; // 全包：整个方形区域都在球形区域内
    }
    
    // 2. 判断是否完全在球形区域外 (计算正方体上距离球心最近点的距离)
    double dx_min = std::max(std::fabs(sphere_center.x() - node_center.x()) - half_size, 0.0);
    double dy_min = std::max(std::fabs(sphere_center.y() - node_center.y()) - half_size, 0.0);
    double dz_min = std::max(std::fabs(sphere_center.z() - node_center.z()) - half_size, 0.0);
    double dist_min_sq = dx_min*dx_min + dy_min*dy_min + dz_min*dz_min;
    
    if (dist_min_sq > r_sq) {
        return 0; // 未包：完全在球形区域外
    }
    
    return 2; // 半包：与球形区域相交
}
 
// 辅助函数：全包状态下，直接高速生成该节点包含的所有最大深度节点坐标
void expandToMaxDepth(const point3d& node_center, double node_size, double resolution, 
                      std::vector<point3d>& results) {
    // 如果当前节点尺寸已经达到最大分辨率，直接作为结果收录
    if (node_size <= resolution + 1e-6) {
        results.push_back(node_center);
        return;
    }
    
    // 虚拟拆分为8个子节点
    double half = node_size / 2.0;
    double quarter = half / 2.0;
    for (int i = 0; i < 8; ++i) {
        point3d child_center = node_center;
        child_center.x() += (i & 1) ? quarter : -quarter;
        child_center.y() += (i & 2) ? quarter : -quarter;
        child_center.z() += (i & 4) ? quarter : -quarter;
        // 递归展开到最大深度
        expandToMaxDepth(child_center, half, resolution, results);
    }
}
 
// 主处理函数：递归拆分与判断
void processNode(const point3d& node_center, double node_size, double resolution, 
                 const point3d& sphere_center, double radius, std::vector<point3d>& results) {
    int state = getSphereBoxState(node_center, node_size, sphere_center, radius);
    
    if (state == 0) {
        return; // 未包：直接忽略
    }
    
    // 如果已经达到最大深度，无论全包还是半包，都作为最终结果收录
    // (半包状态下，最大深度的体素与球体有交集，通常也是我们需要的)
    if (node_size <= resolution + 1e-6) {
        results.push_back(node_center);
        return;
    }
    
    double half = node_size / 2.0;
    double quarter = half / 2.0;
    
    if (state == 1) {
        // 全包：无需再对子节点做球体相交判断，直接全量展开到最大深度
        expandToMaxDepth(node_center, node_size, resolution, results);
    } else if (state == 2) {
        // 半包：必须拆分为8个子节点，递归重新判断状态
        for (int i = 0; i < 8; ++i) {
            point3d child_center = node_center;
            child_center.x() += (i & 1) ? quarter : -quarter;
            child_center.y() += (i & 2) ? quarter : -quarter;
            child_center.z() += (i & 4) ? quarter : -quarter;
            processNode(child_center, half, resolution, sphere_center, radius, results);
        }
    }
}
 

void MapHandler::GetCloudOfPoint(const Point3D& center, 
                                 const PointCloudPtr& cloudOut,
                                 const CloudType& type,
                                 const bool& is_large) 
{
    cloudOut->clear();
    if (!has_semantic_map_ || !semantic_map_msg_) return;
    if (type != CloudType::OBS_CLOUD && type != CloudType::FREE_CLOUD) {
        if (FARUtil::IsDebug) ROS_ERROR("MH: Assigned cloud type invalid.");
        return;
    }

    const float radius = is_large ? semantic_params_.local_window_radius : semantic_params_.local_window_radius * 0.5f;
    if (radius <= 0.0f) return;

    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*semantic_map_msg_));
    if (!tree) {
        ROS_WARN_THROTTLE(1.0, "MH: failed to deserialize semantic octomap in GetCloudOfPoint.");
        return;
    }

    const auto* semantic_tree = dynamic_cast<const SemanticOctree*>(tree.get());
    const auto* color_tree = dynamic_cast<const octomap::ColorOcTree*>(tree.get());
    if (!semantic_tree && !color_tree) return;

    std::vector<point3d> query_voxel_centers;
    query_voxel_centers.reserve(4096);
    const point3d sphere_center(center.x, center.y, center.z);
    const double resolution = tree->getResolution();
    processNode(sphere_center, static_cast<double>(radius) * 2.0, resolution, sphere_center, radius, query_voxel_centers);

    for (const auto& query_center : query_voxel_centers) {
        ColorOcTreeNode::Color semantic_color;
        bool matched = false;

        if (semantic_tree) {
            const SemanticOcTreeNode* node = semantic_tree->search(query_center);
            if (!node || !semantic_tree->isNodeOccupied(node)) continue;
            semantic_color = node->isSemanticsSet()
                ? node->getSemantics().getSemanticColor()
                : node->getColor();
            const uint32_t rgb_key = MakeRgbKey(semantic_color.r, semantic_color.g, semantic_color.b);
            matched = (type == CloudType::OBS_CLOUD)
                ? MatchRgbKey(obstacle_groups_, rgb_key)
                : MatchRgbKey(terrain_support_groups_, rgb_key);
        } else {
            const octomap::ColorOcTreeNode* node = color_tree->search(query_center);
            if (!node || !color_tree->isNodeOccupied(node)) continue;
            semantic_color = node->getColor();
            const uint32_t rgb_key = MakeRgbKey(semantic_color.r, semantic_color.g, semantic_color.b);
            matched = (type == CloudType::OBS_CLOUD)
                ? MatchRgbKey(obstacle_groups_, rgb_key)
                : MatchRgbKey(terrain_support_groups_, rgb_key);
        }

        if (!matched) continue;
        PCLPoint p;
        p.x = query_center.x();
        p.y = query_center.y();
        p.z = query_center.z();
        p.intensity = 0.0f;
        cloudOut->points.push_back(p);
    }

    if (!cloudOut->empty()) {
        FARUtil::FilterCloud(cloudOut, FARUtil::kLeafSize);
    }
}

// SetOrigin 设置的是栅格包围盒地图中心点。
void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    robot_pos_cache_ = ori_robot_pos;
    is_init_ = true;
}
// 每次里程计更新时重算
// 机器人当前体素
// 邻域障碍索引集合
// 邻域自由索引集合
// 地形栅格原点（跟随机器人）
// 主要做的是根据当前机器人位置，重新计算当前邻域对应的体素索引集合，并不会自己去“采集新点”或“写入障碍数据”。
void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos);
    robot_pos_cache_ = odom_pos;
}

// void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
//     if (!is_init_) return;
//     obsCloudOut->clear();
//     if (!has_semantic_map_ || !semantic_obs_cloud_ || semantic_obs_cloud_->empty()) return;
//     CropCloudAroundCenter(semantic_obs_cloud_, robot_pos_cache_, semantic_params_.local_window_radius, obsCloudOut);
// }

// void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
//     if (!is_init_) return;
//     freeCloudOut->clear();
//     if (!has_semantic_map_ || !semantic_terrain_support_cloud_ || semantic_terrain_support_cloud_->empty()) return;
//     CropCloudAroundCenter(semantic_terrain_support_cloud_, robot_pos_cache_, semantic_params_.local_window_radius, freeCloudOut);
// }

// 把当前帧障碍点云写入“障碍栅格地图”，只保留机器人邻域内的点，并对被修改过的格子做降采样滤波。
// 同时把输入点云替换为“有效点”（在范围内且被接受）。
// void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
//     (void)obsCloudInOut;
// }

// 把新来的 free 点云写入全局 free 栅格地图，并对改动过的格子做去冗余滤波
// void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
//     (void)freeCloudIn;
// }

// 在“查询某个点的地面高度”，并通过 is_matched 告诉你是否直接匹配成功。
float MapHandler::TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search) {

    if (!semantic_terrain_support_octree_) {
        return NAN; // 地形树未构建
    }
 
    // 设定射线起点 (x, y, 高空) 和方向 (垂直向下)
    // 注意：起点Z需要高于地图中可能的最高地形
    octomap::point3d start(x, y, 10.0); 
    octomap::point3d direction(0.0f, 0.0f, -1.0f);
    octomap::point3d end;
 
    // 执行射线投射，最大距离设为20米
    bool hit = semantic_terrain_support_octree_->castRay(start, direction, end, true, 20.0);
    
    if (hit) {
        // end.z() 即为射线击中地形表层体素的中心 Z 坐标
        return end.z();
    }
 
    return NAN; // 该点下方未找到地形

    
    is_matched = false;
    if (kdtree_terrain_clould_ && kdtree_terrain_clould_->getInputCloud() && !kdtree_terrain_clould_->getInputCloud()->empty()) {
        float dist_square = FARUtil::kINF;
        const float terrain_h = NearestHeightOfPoint(p, dist_square);
        if (dist_square < FARUtil::kINF) {
            is_matched = true;
            return terrain_h;
        }
    }
    if (is_search) {
        float matched_dist_squre;
        const float terrain_h = NearestHeightOfPoint(p, matched_dist_squre);
        return terrain_h;
    }
    return p.z; 
}

// 获取导航点最近的地形高度。
float MapHandler::NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated) {
    const float p_th = point.z-FARUtil::vehicle_height;
    is_associated = false;
    if (kdtree_terrain_clould_ && kdtree_terrain_clould_->getInputCloud() && !kdtree_terrain_clould_->getInputCloud()->empty()) {
        bool is_matched = false;
        float min_h = p_th;
        float max_h = p_th;
        const float avg_h = NearestHeightOfRadius(point, FARUtil::kMatchDist, min_h, max_h, is_matched);
        if (is_matched) {
            is_associated = true;
            return avg_h;
        }
    }
    return p_th;
}


// 一个“布尔判断器”：判断某个导航点是否落在当前维护的地形邻域障碍集合里。
bool MapHandler::IsNavPointOnTerrainNeighbor(const Point3D& point, const bool& is_extend) {
    if (has_semantic_map_ && semantic_obs_cloud_ && !semantic_obs_cloud_->empty()) {
        PointCloudPtr local_obs(new pcl::PointCloud<PCLPoint>());
        const float radius = is_extend ? FARUtil::kMatchDist : FARUtil::kNavClearDist;
        CropCloudAroundCenter(semantic_obs_cloud_, point, radius, local_obs);
        return !local_obs->empty();
    }
    return false;
}

// 把一组导航节点的 z 高度“贴地修正”，让节点高度和当前地形更一致，同时避免改到不该改的节点。
void MapHandler::AdjustNodesHeight(const NodePtrStack& nodes) {
    if (nodes.empty()) return;
    for (const auto& node_ptr : nodes) {
        if (!node_ptr->is_active || node_ptr->is_boundary || FARUtil::IsFreeNavNode(node_ptr) || FARUtil::IsOutsideGoal(node_ptr) || !FARUtil::IsPointInLocalRange(node_ptr->position, true)) {
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
        float min_th, max_th;
        const float avg_h = NearestHeightOfRadius(ctnode_ptr->position, FARUtil::kMatchDist, min_th, max_th, ctnode_ptr->is_ground_associate);
        if (ctnode_ptr->is_ground_associate) {
            ctnode_ptr->position.z = min_th + FARUtil::vehicle_height;
            ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, H_MAX), H_MIN);
        } else {
            ctnode_ptr->position.z = TerrainHeightOfPoint(ctnode_ptr->position, ctnode_ptr->is_ground_associate, true);
            ctnode_ptr->position.z += FARUtil::vehicle_height;
            ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, H_MAX), H_MIN);
        }
    }
}

void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut) {
    (void)freeCloudIn;
    terrainHeightOut->clear();

    if (!has_semantic_map_) {
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
        return;
    }

    if (!semantic_terrain_support_cloud_ || semantic_terrain_support_cloud_->empty()) {
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
        return;
    }

    *terrainHeightOut = *semantic_terrain_support_cloud_;
    FARUtil::FilterCloud(terrainHeightOut, FARUtil::kLeafSize);
    this->AssignFlatTerrainCloud(terrainHeightOut, flat_terrain_cloud_);
    kdtree_terrain_clould_->setInputCloud(flat_terrain_cloud_);
}

// void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
//     if (!is_init_) return;
//     neighbor_centers.clear();
//     if (!has_semantic_map_ || !semantic_obs_cloud_) return;
//     PointCloudPtr local_obs(new pcl::PointCloud<PCLPoint>());
//     CropCloudAroundCenter(semantic_obs_cloud_, robot_pos_cache_, semantic_params_.local_window_radius, local_obs);
//     for (const auto& point : local_obs->points) {
//         neighbor_centers.emplace_back(point.x, point.y, point.z);
//     }
// }

// void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
//     if (!is_init_) return;
//     occupancy_centers.clear();
//     if (!has_semantic_map_) return;
//     if (semantic_obs_cloud_) {
//         PointCloudPtr local_obs(new pcl::PointCloud<PCLPoint>());
//         CropCloudAroundCenter(semantic_obs_cloud_, robot_pos_cache_, semantic_params_.local_window_radius, local_obs);
//         for (const auto& point : local_obs->points) {
//             occupancy_centers.emplace_back(point.x, point.y, point.z);
//         }
//     }
//     if (semantic_terrain_support_cloud_) {
//         PointCloudPtr local_terrain(new pcl::PointCloud<PCLPoint>());
//         CropCloudAroundCenter(semantic_terrain_support_cloud_, robot_pos_cache_, semantic_params_.local_window_radius, local_terrain);
//         for (const auto& point : local_terrain->points) {
//             occupancy_centers.emplace_back(point.x, point.y, point.z);
//         }
//     }
// }

// void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
//     (void)obsCloud;
// }

