#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"

enum CloudType {
    FREE_CLOUD = 0,
    OBS_CLOUD  = 1
};

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float floor_height;
    float cell_length;
    float cell_height;
    float grid_max_length;
    float grid_max_height;
    // local terrain height map
    float height_voxel_dim;
};

// 局部三维地图与地形语义”的核心中枢
// 维护局部障碍栅格与自由栅格（点云形式）
// 维护地形高度栅格（可通行地面）
// 提供导航相关查询接口（取邻域点云、查高度、判断邻域等）
// 对导航图节点做高度贴地与一致性修正
// 它不是纯“地图存储器”，而是“地图 + 地形理解 + 导航约束”的组合层。
// 上游输入通常是障碍点云、free 点云、机器人位姿；下游受益模块通常是动态图构建、节点高度修正、边连接判定、局部规划与可视化。

// 它更像是一个跨时间累积的环境记忆。即使局部地图还保留着，全局地图维护仍然有几个直接价值。
// 第一，它让图不是只依赖“这一帧附近看到的东西”。局部地图只能描述机器人周围一小块区域，但图更新、边可达性判断、目标重评估都需要更稳定的历史环境信息。FAR 里像 far_planner.cpp 的 TerrainCallBack 会持续更新 surround_obs_cloud_、surround_free_cloud_、stack_new_cloud_、stack_dyobs_cloud_，这些就是给图和路径判断提供“持续可用”的环境上下文。
// 第二，它能缓解局部感知的不完整。局部传感器经常有遮挡、空洞、瞬时误检，单靠局部地图很容易把暂时看不全的地方误判成不可通行或可通行。全局维护的 obstacle/free/terrain 统计，会把多个时刻的信息合起来，图的边和节点重评估会更稳。这个思路在 map_handler.cpp 里很明显：它不只管当前点云，还在维护 world_obs_cloud_grid_、world_free_cloud_grid_、terrain_height_grid_ 这类长期结构。
// 第三，它支持“图的几何语义修正”。graph 负责拓扑连通，但不负责高度、障碍膨胀、局部可走宽度这些细节。FAR 里节点高度要贴地，连接关系要经过 terrain 检查，waypoint 也要结合局部障碍做投影和平滑，所以全局地图是 graph 的底层几何依据，不是 graph 的重复品。你前面看到的 AdjustCTNodeHeight、AdjustNodesHeight、ProjectNavWaypoint 就是这个分工。
// 第四，它对保存、恢复、调试和多轮运行很有用。只有 graph 的话，很多“为什么这条边被删了”“为什么这个目标点被挪了”都很难复盘；而全局维护的地图状态可以让规划器重建环境、保存/加载图、做可视化检查。你贴的 launch 里还包含了 graph decoder，这说明系统本身就把“图的持久化”和“地图状态”当成一条链路在用。
// 所以更准确地说：
// 局部地图解决当前能不能走，全局地图解决长期上下文、历史累计和图更新稳定性，graph 解决拓扑搜索。三者不是替代关系，是层次关系。

class MapHandler {

public:
    MapHandler() = default;
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos);

    void AdjustNodesHeight(const NodePtrStack& nodes);

    void AdjustCTNodeHeight(const CTNodeStack& ctnodes);

    static float TerrainHeightOfPoint(const Point3D& p, 
                                      bool& is_matched, 
                                      const bool& is_search);

    static bool IsNavPointOnTerrainNeighbor(const Point3D& p, const bool& is_extend);

    static float NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated);

    /**
     * @brief Calculate the terrain height of a given point and radius around it
     * @param p A given position
     * @param radius The radius distance around the given posiitn p
     * @param minH[out] The mininal terrain height in the radius
     * @param maxH[out] The maximal terrain height in the radius
     * @param is_match[out] Whether or not find terrain association in radius
     * @return The average terrain height
     */
    template <typename Position>
    static inline float NearestHeightOfRadius(const Position& p, const float& radius, float& minH, float& maxH, bool& is_matched) {
        std::vector<int> pIdxK;
        std::vector<float> pdDistK;
        PCLPoint pcl_p;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        minH = maxH = p.z;
        is_matched = false;
        if (kdtree_terrain_clould_->radiusSearch(pcl_p, radius, pIdxK, pdDistK) > 0) {
            float avgH = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]].intensity;
            minH = maxH = avgH;
            for (int i=1; i<pIdxK.size(); i++) {
                const float temp = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[i]].intensity;
                if (temp < minH) minH = temp;
                if (temp > maxH) maxH = temp;
                avgH += temp;
            }
            avgH /= (float)pIdxK.size();
            is_matched = true;
            return avgH;
        }
        return p.z;
    }

    /** Update global cloud grid with incoming clouds 
     * @param CloudInOut incoming cloud ptr and output valid in range points
    */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param center the position of the grid that want to extract
     * @param cloudOut output cloud ptr
     * @param type choose free or obstacle cloud for extraction
     * @param is_large whether or not using the surrounding cells
    */
    void GetCloudOfPoint(const Point3D& center, 
                         const PointCloudPtr& CloudOut, 
                         const CloudType& type,
                         const bool& is_large);

    /**
     * Get neihbor cells center positions
     * @param neighbor_centers[out] neighbor centers stack
    */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor cells center positions
     * @param occupancy_centers[out] occupanied cells center stack
    */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param obsCloud obstacle cloud points that need to be removed
    */ 
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /**
     * @brief Reset Current Grip Map Clouds
     */
    void ResetGripMapCloud();

    /**
     * @brief Clear the cells that from the robot position to the given position
     * @param point Give point location
     */
    void ClearObsCellThroughPosition(const Point3D& point);

private:
    MapHandlerParams map_params_;
    int neighbor_Lnum_, neighbor_Hnum_;
    Eigen::Vector3i robot_cell_sub_;
    int INFLATE_N;
    bool is_init_ = false;
    PointCloudPtr flat_terrain_cloud_;
    static PointKdTreePtr kdtree_terrain_clould_;

    template <typename Position>
    static inline float NearestHeightOfPoint(const Position& p, float& dist_square) {
        // Find the nearest node in graph
        std::vector<int> pIdxK(1);
        std::vector<float> pdDistK(1);
        PCLPoint pcl_p;
        dist_square = FARUtil::kINF;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        if (kdtree_terrain_clould_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
            pcl_p = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]];
            dist_square = pdDistK[0];
            return pcl_p.intensity;
        }
        return p.z;
    }

    void SetTerrainHeightGridOrigin(const Point3D& robot_pos);

    void TraversableAnalysis(const PointCloudPtr& terrainHeightOut);

    inline void AssignFlatTerrainCloud(const PointCloudPtr& terrainRef, PointCloudPtr& terrainFlatOut) {
        const int N = terrainRef->size();
        terrainFlatOut->resize(N);
        for (int i = 0; i<N; i++) {
            PCLPoint pcl_p = terrainRef->points[i];
            pcl_p.intensity = pcl_p.z, pcl_p.z = 0.0f;
            terrainFlatOut->points[i] = pcl_p;
        }
    }

    inline void Expansion2D(const Eigen::Vector3i& csub, std::vector<Eigen::Vector3i>& subs, const int& n) {
        subs.clear();
        for (int ix=-n; ix<=n; ix++) {
            for (int iy=-n; iy<=n; iy++) {
                Eigen::Vector3i sub = csub;
                sub.x() += ix, sub.y() += iy;
                subs.push_back(sub); 
            }
        }
    }

    void ObsNeighborCloudWithTerrain(std::unordered_set<int>& neighbor_obs,
                                     std::unordered_set<int>& extend_terrain_obs);

    std::unordered_set<int> neighbor_free_indices_;        // surrounding free cloud grid indices stack
    static std::unordered_set<int> neighbor_obs_indices_;  // surrounding obs cloud grid indices stack
    static std::unordered_set<int> extend_obs_indices_;    // extended surrounding obs cloud grid indices stack

    // 哪些体素被观测过
    std::vector<int> global_visited_induces_;
    std::vector<int> util_obs_modified_list_;
    std::vector<int> util_free_modified_list_;
    std::vector<int> util_remove_check_list_;
    static std::vector<int> terrain_grid_occupy_list_;
    static std::vector<int> terrain_grid_traverse_list_;

    // 创建一个三维网格容器，每个 cell 的数据类型是 PointCloudPtr也就是每个格子里放的是一个点云指针（指向 pcl::PointCloud）。
    // 考虑是否需要高程？为什么需要三个地图
    // 自由栅格
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_free_cloud_grid_;
    // 障碍栅格 
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_obs_cloud_grid_;
    // 地形高度栅格，二维平面格子，每格存一组高度样本（后续会压成代表高度）。
    static std::unique_ptr<grid_ns::Grid<std::vector<float>>> terrain_height_grid_;
 
};

#endif