/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/map_handler.h"

/***************************************************************************************/

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    const int row_num = std::ceil(map_params_.grid_max_length / map_params_.cell_length);
    const int col_num = row_num;
    int level_num = std::ceil(map_params_.grid_max_height / map_params_.cell_height);
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0f / map_params_.cell_length) + 1; 
    neighbor_Hnum_ = 5; 
    if (level_num % 2 == 0) level_num ++;         // force to odd number, robot will be at center
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_ ++; // force to odd number

    // inlitialize grid 
    Eigen::Vector3i pointcloud_grid_size(row_num, col_num, level_num);
    Eigen::Vector3d pointcloud_grid_origin(0,0,0);
    Eigen::Vector3d pointcloud_grid_resolution(map_params_.cell_length, map_params_.cell_length, map_params_.cell_height);
    PointCloudPtr cloud_ptr_tmp;
    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    world_free_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    const int n_cell  = world_obs_cloud_grid_->GetCellNumber();
    for (int i = 0; i < n_cell; i++) {
        world_obs_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
        world_free_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
    }
    global_visited_induces_.resize(n_cell), util_remove_check_list_.resize(n_cell);
    util_obs_modified_list_.resize(n_cell), util_free_modified_list_.resize(n_cell);
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);

    // init terrain height map
    int height_dim = std::ceil((map_params_.sensor_range + map_params_.cell_length) * 2.0f / FARUtil::robot_dim);
    if (height_dim % 2 == 0) height_dim ++;
    Eigen::Vector3i height_grid_size(height_dim, height_dim, 1);
    Eigen::Vector3d height_grid_origin(0,0,0);
    Eigen::Vector3d height_grid_resolution(FARUtil::robot_dim, FARUtil::robot_dim, FARUtil::kLeafSize);
    std::vector<float> temp_vec;
    terrain_height_grid_ = std::make_unique<grid_ns::Grid<std::vector<float>>>(
        height_grid_size, temp_vec, height_grid_origin, height_grid_resolution, 3);
    
    const int n_terrain_cell = terrain_height_grid_->GetCellNumber();
    terrain_grid_occupy_list_.resize(n_terrain_cell), terrain_grid_traverse_list_.resize(n_terrain_cell);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);

    INFLATE_N = 1;
    flat_terrain_cloud_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    kdtree_terrain_clould_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
    kdtree_terrain_clould_->setSortedResults(false);
}

void MapHandler::ResetGripMapCloud() {
    const int n_cell = world_obs_cloud_grid_->GetCellNumber();
    for (int i=0; i<n_cell; i++) {
        world_obs_cloud_grid_->GetCell(i)->clear();
        world_free_cloud_grid_->GetCell(i)->clear();
    }
    std::fill(global_visited_induces_.begin(),     global_visited_induces_.end(),     0);
    std::fill(util_obs_modified_list_.begin(),     util_obs_modified_list_.end(),     0);
    std::fill(util_free_modified_list_.begin(),    util_free_modified_list_.end(),    0);
    std::fill(util_remove_check_list_.begin(),     util_remove_check_list_.end(),     0);
    std::fill(terrain_grid_occupy_list_.begin(),   terrain_grid_occupy_list_.end(),   0);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);
}

// 沿着“机器人当前位置 → 给定点 point”的射线，把这条线上附近高度层里的障碍栅格清空，相当于做一条“视线清障”。
void MapHandler::ClearObsCellThroughPosition(const Point3D& point) {
    const Eigen::Vector3i psub = world_obs_cloud_grid_->Pos2Sub(point.x, point.y, point.z);
    std::vector<Eigen::Vector3i> ray_subs;
    // 从机器人当前格子 robot_cell_sub_ 向目标格子 psub 做射线遍历，拿到经过的二维/三维格子序列 ray_subs。
    world_obs_cloud_grid_->RayTraceSubs(robot_cell_sub_, psub, ray_subs);
    // 对射线上的每个格子 sub，在 z 方向再扩一段高度（上下若干层），得到 csub。
    // 也就是不是只清一层，而是清一个“竖向柱状区域”。
    const int H = neighbor_Hnum_ / 2;
    for (const auto& sub : ray_subs) {
        for (int k = -H; k <= H; k++) {
            Eigen::Vector3i csub = sub;
            csub.z() += k;
            const int ind = world_obs_cloud_grid_->Sub2Ind(csub);
            // 在地图范围内的格子，并且在“机器人邻域障碍索引集合”里的格子
            if (!world_obs_cloud_grid_->InRange(csub) || neighbor_obs_indices_.find(ind) == neighbor_obs_indices_.end()) continue; 
            // 把该格子的障碍点云清空（认为这里不再是障碍）。
            world_obs_cloud_grid_->GetCell(ind)->clear();
            // 如果该格子里也没有 free 点云，就把“已访问/已观测”标记清零，表示这个格子现在既非障碍也非自由观测。
            if (world_free_cloud_grid_->GetCell(ind)->empty()) {
                global_visited_induces_[ind] = 0;
            }
        }
    }
}

// is_large 就是控制“水平范围是否扩大一圈”的开关
void MapHandler::GetCloudOfPoint(const Point3D& center, 
                                 const PointCloudPtr& cloudOut,
                                 const CloudType& type,
                                 const bool& is_large) 
{
    cloudOut->clear();
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(center.x, center.y, center.z);
    const int N = is_large ? 1 : 0;
    const int H = neighbor_Hnum_ / 2;
    for (int i = -N; i <= N; i++) {
        for (int j = -N; j <= N; j++) {
            for (int k = -H; k <= H; k++) {
                Eigen::Vector3i csub = sub;
                csub.x() += i, csub.y() += j, csub.z() += k;
                if (!world_obs_cloud_grid_->InRange(csub)) continue;
                if (type == CloudType::FREE_CLOUD) {
                    *cloudOut += *(world_free_cloud_grid_->GetCell(csub));
                } else if (type == CloudType::OBS_CLOUD) {
                    *cloudOut += *(world_obs_cloud_grid_->GetCell(csub));
                } else {
                    if (FARUtil::IsDebug) ROS_ERROR("MH: Assigned cloud type invalid.");
                    return;
                }
            }
        }
    }
}

// SetOrigin 设置的是栅格包围盒的左下后角（最小角点），不是地图中心点。
void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    Point3D map_origin;
    const Eigen::Vector3i dim = world_obs_cloud_grid_->GetSize();
    map_origin.x = ori_robot_pos.x - (map_params_.cell_length * dim.x()) / 2.0f;
    map_origin.y = ori_robot_pos.y - (map_params_.cell_length * dim.y()) / 2.0f;
    // 你看到 z 还减了 FARUtil::vehicle_height，是因为这里把地图 z 原点对齐到“地面参考系”， 而不是对齐到机器人底盘。
    map_origin.z = ori_robot_pos.z - (map_params_.cell_height * dim.z()) / 2.0f - FARUtil::vehicle_height; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x, map_origin.y, map_origin.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    world_free_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    is_init_ = true;
    if (FARUtil::IsDebug) ROS_INFO("MH: Global Cloud Map Grid Initialized.");
}
// 每次里程计更新时重算
// 机器人当前体素
// 邻域障碍索引集合
// 邻域自由索引集合
// 地形栅格原点（跟随机器人）
// 主要做的是根据当前机器人位置，重新计算当前邻域对应的体素索引集合，并不会自己去“采集新点”或“写入障碍数据”。
void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    // 首次调用时初始化地图原点
    if (!is_init_) this->SetMapOrigin(odom_pos);
    robot_cell_sub_ = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z));
    // Get neighbor indices
    // 把机器人世界坐标换成栅格下标
    neighbor_free_indices_.clear(), neighbor_obs_indices_.clear();
    const int N = neighbor_Lnum_ / 2;
    const int H = neighbor_Hnum_ / 2;
    Eigen::Vector3i neighbor_sub;
    // 重算机器人周围“邻域格子索引”
    for (int i = -N; i <= N; i++) {
        neighbor_sub.x() = robot_cell_sub_.x() + i;
        for (int j = -N; j <= N; j++) {
            neighbor_sub.y() = robot_cell_sub_.y() + j;
            // additional terrain points -1
            neighbor_sub.z() = robot_cell_sub_.z() - H - 1;
            // 这个栅格下标是否合法、有没有落在网格边界内
            if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                // 三维栅格下标 (x, y, z) 压成一个一维数组索引，方便直接访问 
                int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                neighbor_free_indices_.insert(ind);
            }
            for (int k =-H; k <= H; k++) {
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_obs_indices_.insert(ind), neighbor_free_indices_.insert(ind);
                }
            }
        }
    }
    // 更新地形高度栅格的原点
    // 调用 SetTerrainHeightGridOrigin，把高度图中心跟随机器人移动
    this->SetTerrainHeightGridOrigin(odom_pos);
}

void MapHandler::SetTerrainHeightGridOrigin(const Point3D& robot_pos) {
    // update terrain height grid center
    const Eigen::Vector3d res = terrain_height_grid_->GetResolution();
    const Eigen::Vector3i dim = terrain_height_grid_->GetSize();
    Eigen::Vector3d grid_origin;
    grid_origin.x() = robot_pos.x - (res.x() * dim.x()) / 2.0f;
    grid_origin.y() = robot_pos.y - (res.y() * dim.y()) / 2.0f;
    grid_origin.z() = 0.0f        - (res.z() * dim.z()) / 2.0f;
    terrain_height_grid_->SetOrigin(grid_origin);
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    if (!is_init_) return;
    obsCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_obs_indices_) {
        if (world_obs_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *obsCloudOut += *(world_obs_cloud_grid_->GetCell(neighbor_ind));
    }
}
// neighbor_free_indices_ 这个集合保存的是机器人附近、被认为是障碍的栅格索引。
void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
    if (!is_init_) return;
    freeCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_free_indices_) {
        if (world_free_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *freeCloudOut += *(world_free_cloud_grid_->GetCell(neighbor_ind));
    }
}

// 把当前帧障碍点云写入“障碍栅格地图”，只保留机器人邻域内的点，并对被修改过的格子做降采样滤波。
// 同时把输入点云替换为“有效点”（在范围内且被接受）。
void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
    if (!is_init_ || obsCloudInOut->empty()) return;
    // 清零“本次被改动格子”标记
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    // obs_valid_ptr 用来收集“真正被接受”的障碍点
    PointCloudPtr obs_valid_ptr(new pcl::PointCloud<PCLPoint>());
    for (const auto& point : obsCloudInOut->points) {
        // 世界坐标转栅格下标 sub
        // 越界就丢弃
        // 转成一维索引 ind
        // 只有在 neighbor_obs_indices_（机器人邻域障碍索引集合）里才接受
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        if (neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
            // 追加到对应障碍栅格 cell
            world_obs_cloud_grid_->GetCell(ind)->points.push_back(point);
            obs_valid_ptr->points.push_back(point);
            util_obs_modified_list_[ind] = 1;
            global_visited_induces_[ind] = 1;
        }
    }
    // 回写输入参数（原地输出）函数结束后，传进来的点云会被替换成“筛选后的有效障碍点云”。
    *obsCloudInOut = *obs_valid_ptr;
    // Filter Modified Ceils
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_obs_modified_list_[i] == 1) FARUtil::FilterCloud(world_obs_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

// 把新来的 free 点云写入全局 free 栅格地图，并对改动过的格子做去冗余滤波
void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
    if (!is_init_ || freeCloudIn->empty()) return;
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    for (const auto& point : freeCloudIn->points) {
        Eigen::Vector3i sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        world_free_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_free_modified_list_[ind] = 1;
        global_visited_induces_[ind]  = 1;
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_free_cloud_grid_->GetCellNumber(); ++i) {
      if (util_free_modified_list_[i] == 1) FARUtil::FilterCloud(world_free_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

// 在“查询某个点的地面高度”，并通过 is_matched 告诉你是否直接匹配成功。
float MapHandler::TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search) {
    is_matched = false;
    const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(p.x, p.y, 0.0f));
    if (terrain_height_grid_->InRange(sub)) {
        const int ind = terrain_height_grid_->Sub2Ind(sub);
        if (terrain_grid_traverse_list_[ind] != 0) {
            is_matched = true;
            return terrain_height_grid_->GetCell(ind)[0];
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
    const Eigen::Vector3i ori_sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, p_th));
    is_associated = false;
    if (world_free_cloud_grid_->InRange(ori_sub)) {
        // downward seach
        bool is_dw_associated = false;
        Eigen::Vector3i dw_near_sub = ori_sub;
        float dw_terrain_h = p_th;
        while (world_free_cloud_grid_->InRange(dw_near_sub)) {
            if (!world_free_cloud_grid_->GetCell(dw_near_sub)->empty()) {
                int counter = 0;
                dw_terrain_h = 0.0f;
                for (const auto& pcl_p : world_free_cloud_grid_->GetCell(dw_near_sub)->points) {
                    dw_terrain_h += pcl_p.z, counter ++;
                }
                dw_terrain_h /= (float)counter;
                is_dw_associated = true;
                break;
            }
            dw_near_sub.z() --;
        }
        // upward search
        bool is_up_associated = false;
        Eigen::Vector3i up_near_sub = ori_sub;
        float up_terrain_h = p_th;
        while (world_free_cloud_grid_->InRange(up_near_sub)) {
            if (!world_free_cloud_grid_->GetCell(up_near_sub)->empty()) {
                int counter = 0;
                up_terrain_h = 0.0f;
                for (const auto& pcl_p : world_free_cloud_grid_->GetCell(up_near_sub)->points) {
                    up_terrain_h += pcl_p.z, counter ++;
                }
                up_terrain_h /= (float)counter;
                is_up_associated = true;
                break;
            }
            up_near_sub.z() ++;
            
        }
        is_associated = (is_up_associated || is_dw_associated) ? true : false;
        if (is_up_associated && is_dw_associated) { // compare nearest
            if (up_near_sub.z() - ori_sub.z() < ori_sub.z() - dw_near_sub.z()) return up_terrain_h;
            else return dw_terrain_h;
        } else if (is_up_associated) return up_terrain_h;
        else return dw_terrain_h;
    }
    return p_th;
}


// 一个“布尔判断器”：判断某个导航点是否落在当前维护的地形邻域障碍集合里。
bool MapHandler::IsNavPointOnTerrainNeighbor(const Point3D& point, const bool& is_extend) {
    const float h = point.z - FARUtil::vehicle_height; 
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, h));
    if (!world_obs_cloud_grid_->InRange(sub)) return false;
    const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
    if (is_extend && extend_obs_indices_.find(ind) != extend_obs_indices_.end()) {
        return true;
    }
    if (!is_extend && neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
        return true;
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

// 用地形高度信息“过滤并扩展”邻域障碍索引，让障碍集合更符合真实可通行地形。
void MapHandler::ObsNeighborCloudWithTerrain(std::unordered_set<int>& neighbor_obs, std::unordered_set<int>& extend_terrain_obs) {
    std::unordered_set<int> neighbor_copy = neighbor_obs;
    neighbor_obs.clear();
    // 把障碍栅格索引 idx 转为世界坐标 pos
    // 以 pos 为中心、半径 (R = cell_length \times 0.7071)（约等于半格对角）查询附近地形高度范围 minH/maxH
    // 只有满足下面高度重叠条件，才保留该障碍索引：
    // pos.z + cell_height > minH
    // pos.z - cell_height < maxH + kTolerZ
    const float R = map_params_.cell_length * 0.7071f; // sqrt(2)/2
    for (const auto& idx : neighbor_copy) {
        const Point3D pos = Point3D(world_obs_cloud_grid_->Ind2Pos(idx)); 
        const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(pos.x, pos.y, 0.0f));
        const int terrain_ind = terrain_height_grid_->Sub2Ind(sub);
        bool inRange = false;
        float minH, maxH;
        const float avgH = NearestHeightOfRadius(pos, R, minH, maxH, inRange);
        if (inRange && pos.z + map_params_.cell_height > minH &&
                       pos.z - map_params_.cell_height < maxH + FARUtil::kTolerZ) // use map_params_.cell_height/2.0 as a tolerance margin
        {
            neighbor_obs.insert(idx);
        }
    }
    extend_terrain_obs.clear(); // assign extended terrain obs indices
    // 障碍体素的高度区间要和附近地形高度区间有重叠，否则认为这个障碍不贴地形，剔除掉。
    // 生成扩展障碍集合 extend_terrain_obs
    // 清空 extend_terrain_obs 后，对过滤后的每个障碍索引做 z 方向轻微膨胀（-1 和 0 两层）：
    // 取当前索引对应的栅格 csub
    // 对 csub.z 加 -1、0
    // 在范围内就加入 extend_terrain_obs
    // 效果是：得到一个“贴地后 + 轻微向下扩展”的障碍集合，常用于更保守的碰撞/邻域判断。
    const std::vector<int> inflate_vec{-1, 0};
    for (const int& idx : neighbor_obs) {
        const Eigen::Vector3i csub = world_obs_cloud_grid_->Ind2Sub(idx);
        for (const int& plus : inflate_vec) {
            Eigen::Vector3i sub = csub; 
            sub.z() += plus;
            if (!world_obs_cloud_grid_->InRange(sub)) continue;
            const int plus_idx = world_obs_cloud_grid_->Sub2Ind(sub);
            extend_terrain_obs.insert(plus_idx);
        }
    }
}

// 这个函数是“用当前自由点云更新地形高度图”，并同步更新地形相关的局部障碍集合。
// 把 free 点云投影到地形栅格并扩张写入，形成每格高度样本集；然后调用 TraversableAnalysis 提取“可通行地形”；更新地形 KDTree；最后用地形去筛邻域障碍。
void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut) {
    if (freeCloudIn->empty()) return;
    PointCloudPtr copy_free_ptr(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*freeCloudIn, *copy_free_ptr);
    FARUtil::FilterCloud(copy_free_ptr, terrain_height_grid_->GetResolution());
    // 重置地形占据标记，把 terrain_grid_occupy_list_ 清零，准备重新写入本轮地形高度数据。
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    
    // 投影到地形栅格坐标（x,y，z按点高保存）
    // 做一次 2D 扩张（Expansion2D，半径由 INFLATE_N 控制）
    // 对扩张后的每个格子：
    // 首次写入：新建高度列表并放入该点 z
    // 非首次：把该点 z 追加进去
    // 标记该格子被占据
    for (const auto& point : copy_free_ptr->points) {
        Eigen::Vector3i csub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, 0.0f));
        std::vector<Eigen::Vector3i> subs;
        this->Expansion2D(csub, subs, INFLATE_N);
        for (const auto& sub : subs) {
            if (!terrain_height_grid_->InRange(sub)) continue;
            const int ind = terrain_height_grid_->Sub2Ind(sub);
            if (terrain_grid_occupy_list_[ind] == 0) {
                terrain_height_grid_->GetCell(ind).resize(1);
                terrain_height_grid_->GetCell(ind)[0] = point.z;
            } else {
                terrain_height_grid_->GetCell(ind).push_back(point.z);
            }
            terrain_grid_occupy_list_[ind] = 1;
        }
    }
    const int N = terrain_grid_occupy_list_.size();
    // 运行可通行地形分析
    // 调用 TraversableAnalysis，把可通行地形提取到 map_handler.cpp:433，并更新 terrain_grid_traverse_list_
    this->TraversableAnalysis(terrainHeightOut);
    if (terrainHeightOut->empty()) { // set terrain height kdtree
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
    } else {
        this->AssignFlatTerrainCloud(terrainHeightOut, flat_terrain_cloud_);
        kdtree_terrain_clould_->setInputCloud(flat_terrain_cloud_);
    }
    // update surrounding obs cloud grid indices based on terrain
    this->ObsNeighborCloudWithTerrain(neighbor_obs_indices_, extend_obs_indices_);
}

// 它用“机器人高度锚点 + 四连通 BFS + 高差阈值”从地形栅格里提取连通可通行地面，避免把高度断层或不连通区域误当成可走区域。

// 这是地形语义最关键的一步：
// 从机器人所在地形格出发
// 用高度阈值做四连通扩展
// 保留高度连续、可走通的区域
// 输出 terrainHeightOut 并写 terrain_grid_traverse_list_
// 本质是“机器人可达地面分割”。
void MapHandler::TraversableAnalysis(const PointCloudPtr& terrainHeightOut) {
    const Eigen::Vector3i robot_sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(FARUtil::robot_pos.x, 
                                                                                    FARUtil::robot_pos.y, 0.0f));
    terrainHeightOut->clear();
    // 先把机器人当前位置投影到地形栅格，清空 terrainHeightOut。如果机器人不在地形栅格范围内，直接报错返回。
    if (!terrain_height_grid_->InRange(robot_sub)) {
        ROS_ERROR("MH: terrain height analysis error: robot position is not in range");
        return;
    }
    // 作为高度连续性阈值
    const float H_THRED = map_params_.height_voxel_dim;
    // 全部清零，后续用于标记“可通行格”。
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);
    // Lambda Function
    // 只接受 |ref_h - cur_h| <= H_THRED 的高度样本，并把 ref_id 的高度收敛成这些样本的均值。
    auto IsTraversableNeighbor = [&] (const int& cur_id, const int& ref_id) {
        if (terrain_grid_occupy_list_[ref_id] == 0) return false;
        const float cur_h = terrain_height_grid_->GetCell(cur_id)[0];
        float ref_h = 0.0f;
        int counter = 0;
        for (const auto& e : terrain_height_grid_->GetCell(ref_id)) {
            if (abs(e - cur_h) > H_THRED) continue;
            ref_h += e, counter ++;
        }
        if (counter > 0) {
            terrain_height_grid_->GetCell(ref_id).resize(1);
            terrain_height_grid_->GetCell(ref_id)[0] = ref_h / (float)counter;
            return true;
        }
        return false;
    };
    // 把该格的高度点写入 terrainHeightOut，并标记 terrain_grid_traverse_list_[idx]=1。
    auto AddTraversePoint = [&] (const int& idx) {
        Eigen::Vector3d cpos = terrain_height_grid_->Ind2Pos(idx);
        cpos.z() = terrain_height_grid_->GetCell(idx)[0];
        const PCLPoint p = FARUtil::Point3DToPCLPoint(Point3D(cpos));
        terrainHeightOut->points.push_back(p);
        terrain_grid_traverse_list_[idx] = 1;
    };

    const int robot_idx = terrain_height_grid_->Sub2Ind(robot_sub);
    const std::array<int, 4> dx = {-1, 0, 1, 0};
    const std::array<int, 4> dy = { 0, 1, 0,-1};
    std::deque<int> q;
    bool is_robot_terrain_init = false;
    std::unordered_set<int> visited_set;
    q.push_back(robot_idx), visited_set.insert(robot_idx);
    while (!q.empty()) {
        const int cur_id = q.front();
        q.pop_front();
        if (terrain_grid_occupy_list_[cur_id] != 0) {
            if (!is_robot_terrain_init) {
                float avg_h = 0.0f;
                int counter = 0;
                for (const auto& e : terrain_height_grid_->GetCell(cur_id)) {
                    if (abs(e - FARUtil::robot_pos.z + FARUtil::vehicle_height) > H_THRED) continue;
                    avg_h += e, counter ++;
                }
                if (counter > 0) {
                    avg_h /= (float)counter;
                    terrain_height_grid_->GetCell(cur_id).resize(1);
                    terrain_height_grid_->GetCell(cur_id)[0] = avg_h;
                    AddTraversePoint(cur_id);
                    is_robot_terrain_init = true; // init terrain height map current robot height
                    q.clear();
                }
            } else {
                AddTraversePoint(cur_id);
            }
        } else if (is_robot_terrain_init) {
            continue;
        }
        const Eigen::Vector3i csub = terrain_height_grid_->Ind2Sub(cur_id);
        for (int i=0; i<4; i++) {
            Eigen::Vector3i ref_sub = csub;
            ref_sub.x() += dx[i], ref_sub.y() += dy[i];
            if (!terrain_height_grid_->InRange(ref_sub)) continue;
            const int ref_id = terrain_height_grid_->Sub2Ind(ref_sub);
            if (!visited_set.count(ref_id) && (!is_robot_terrain_init || IsTraversableNeighbor(cur_id, ref_id))) {
                q.push_back(ref_id);
                visited_set.insert(ref_id);
            }
        }
    }
}


void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    if (!is_init_) return;
    neighbor_centers.clear();
    for (const auto& ind : neighbor_obs_indices_) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}

void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    if (!is_init_) return;
    occupancy_centers.clear();
    const int N = world_obs_cloud_grid_->GetCellNumber();
    for (int ind=0; ind<N; ind++) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
    for (const auto& point : obsCloud->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        util_remove_check_list_[ind] = 1;
    }
    // util_remove_check_list_[ind] == 1：这个格子被标记为“本轮需要检查删除”。
    // global_visited_induces_[ind] == 1：这个格子以前确实有被观测/写入过，不是空白未使用格子。
    for (const auto& ind : neighbor_obs_indices_) {
        if (util_remove_check_list_[ind] == 1 && global_visited_induces_[ind] == 1) {
            FARUtil::RemoveOverlapCloud(world_obs_cloud_grid_->GetCell(ind), obsCloud);
        }
    }
}

