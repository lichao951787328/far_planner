#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>

enum CloudType {
    FREE_CLOUD = 0,
    OBS_CLOUD  = 1
};

struct SemanticClassGroup {
    SemanticClassGroup() = default;
    SemanticClassGroup(const std::string& group_name, const uint32_t key)
        : name(group_name), rgb_key(key) {}
    std::string name;
    uint32_t rgb_key = 0;
};

struct SemanticMapParams {
    SemanticMapParams() = default;
    float local_window_radius = 0.0f;
    float terrain_search_radius = 0.8f;
    float terrain_neighbor_radius = 1.0f;
    float local_planner_radius = 5.0f;
    float local_planner_resolution = 0.2f;
    float local_planner_obstacle_intensity = 200.0f;
    bool  use_top1_only = true;
    float min_semantic_prob = 0.55f;
};

struct MapHandlerParams {
    MapHandlerParams() = default;

    float sensor_range = 0.0f;
    float floor_height = 0.0f;
    SemanticMapParams semantic_params;
    std::vector<SemanticClassGroup> obstacle_groups;
    std::vector<SemanticClassGroup> terrain_support_groups;
    std::vector<SemanticClassGroup> dynamic_obstacle_groups;
};

struct SemanticVoxelSample {
    SemanticVoxelSample() = default;
    Point3D center;
    uint32_t rgb_key = 0;
    bool is_occupied = false;
    bool is_terrain_support = false;
};

// MapHandler 是 semantic octomap 的局部查询层。
// FARMaster 只负责订阅和转交，MapHandler 负责语义地图解析、局部裁剪和地形查询。
// 对外接口名保持稳定，便于 DynamicGraph/GraphPlanner 继续复用。

class MapHandler {

public:
    MapHandler();
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    // 仅在消息被成功验证并替换当前快照时返回 true。
    bool SetSemanticOctomap(const octomap_msgs::OctomapConstPtr& msg);
    bool HasSemanticMap() const { return has_semantic_map_; }
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos);

    void AdjustNodesHeight(const NodePtrStack& nodes);

    void AdjustCTNodeHeight(const CTNodeStack& ctnodes);

    static float TerrainHeightOfPoint(const Point3D& p, 
                                      bool& is_matched, 
                                      const bool& is_search);

    /**
     * Check whether the nav point's ground-referenced height overlaps the
     * terrain-aligned local grid band. is_extend adds the legacy one-cell
     * downward inflation; it does not enlarge the horizontal search radius.
     */
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
        if (!local_terrain_support_octree_ ||
            local_terrain_support_octree_->size() == 0 ||
            !kdtree_terrain_clould_ ||
            !kdtree_terrain_clould_->getInputCloud() ||
            kdtree_terrain_clould_->getInputCloud()->empty()) {
            return p.z;
        }
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

    /** Compatibility APIs retained for downstream stability; no-op in semantic-only mode. */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    /** Effective local obstacle view used by contour extraction and Graph updates.
     *  Includes persistent static semantics and dynamic obstacles from the
     *  latest local snapshot; every addition/removal also enters the
     *  incremental changed-obstacle pipeline.
     */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);
    /** Current static obstacles plus dynamic obstacles in the latest local snapshot. */
    void GetCollisionObsCloud(const PointCloudPtr& obsCloudOut) const;
    /** Dynamic obstacles currently reported occupied by the semantic octree. */
    void GetCurrentDynamicObsCloud(const PointCloudPtr& obsCloudOut) const;
    /** Dynamic obstacles in the latest local semantic-map snapshot. */
    void GetEffectiveDynamicObsCloud(const PointCloudPtr& obsCloudOut) const;
    /** Dense 2.5D static collision layer consumed by the trajectory local planner. */
    void GetLocalPlannerStaticObsCloud(const PointCloudPtr& cloudOut) const;
    /** Dense current dynamic-obstacle layer consumed through /added_obstacles. */
    void GetLocalPlannerDynamicObsCloud(const PointCloudPtr& cloudOut) const;
    /** Dynamic points that appeared in the latest accepted snapshot. */
    void GetDynamicAddedCloud(const PointCloudPtr& cloudOut) const;
    /** Previous-snapshot dynamic points absent from the latest local snapshot. */
    void GetDynamicRemovedCloud(const PointCloudPtr& cloudOut) const;
    /** Return obstacle positions added, removed, or reclassified by the latest local rebuild. */
    void GetChangedObsCloud(const PointCloudPtr& changedCloudOut) const;

    /** Extract local semantic clouds around a given center.
     * @param center the query center
     * @param cloudOut output cloud ptr
     * @param type choose free or obstacle cloud for extraction
     * @param is_large whether to use a larger local window
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

    /** Compatibility API retained; no-op in semantic-only mode. */
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /** Reset semantic local caches. */
    void ResetGripMapCloud();

    /** Compatibility API retained; no-op in semantic-only mode. */
    void ClearObsCellThroughPosition(const Point3D& point);

private:
    void RefreshLocalTerrainSupportOctomap();
    void BuildLocalPlannerObstacleCloud(const PointCloudPtr& source,
                                        const PointCloudPtr& cloudOut) const;

    // 经验证的 semantic octree 快照：每条消息只反序列化一次。
    std::unique_ptr<octomap::AbstractOcTree> semantic_tree_snapshot_;
    ros::Time semantic_stamp_;
    std::string semantic_frame_id_;
    SemanticMapParams semantic_params_;
    std::vector<SemanticClassGroup> obstacle_groups_;
    std::vector<SemanticClassGroup> terrain_support_groups_;
    std::vector<SemanticClassGroup> dynamic_obstacle_groups_;
    static std::shared_ptr<octomap::OcTree> local_terrain_support_octree_;
    PointCloudPtr semantic_obs_cloud_;
    PointCloudPtr semantic_terrain_support_cloud_;
    PointCloudPtr current_dynamic_obs_cloud_;
    PointCloudPtr effective_dynamic_obs_cloud_;
    PointCloudPtr collision_obs_cloud_;
    PointCloudPtr dynamic_added_cloud_;
    PointCloudPtr dynamic_removed_cloud_;
    PointCloudPtr changed_obs_cloud_;
    std::unordered_map<uint64_t, PCLPoint> previous_local_obs_voxels_;
    std::unordered_map<uint64_t, PCLPoint> previous_local_dynamic_voxels_;
    bool has_semantic_map_ = false;


    MapHandlerParams map_params_;
    Point3D robot_pos_cache_;
    bool is_init_ = false;
    PointCloudPtr flat_terrain_cloud_;
    static PointKdTreePtr kdtree_terrain_clould_;
    static float terrain_search_radius_;
    static float terrain_neighbor_radius_;

    template <typename Position>
    static inline float NearestHeightOfPoint(const Position& p,
                                             float& dist_square,
                                             const float search_radius = -1.0f) {
        std::vector<int> pIdxK;
        std::vector<float> pdDistK;
        PCLPoint pcl_p;
        dist_square = FARUtil::kINF;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        const float radius = search_radius > 0.0f
            ? search_radius
            : std::max(FARUtil::kLeafSize, 1e-3f);
        if (!local_terrain_support_octree_ ||
            local_terrain_support_octree_->size() == 0 ||
            !kdtree_terrain_clould_ ||
            !kdtree_terrain_clould_->getInputCloud() ||
            kdtree_terrain_clould_->getInputCloud()->empty()) {
            return p.z;
        }
        if (kdtree_terrain_clould_->radiusSearch(pcl_p, radius, pIdxK, pdDistK) > 0) {
            int best_idx = 0;
            float best_dist = pdDistK[0];
            float best_height_delta = std::fabs(
                kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]].intensity - p.z);
            for (int i = 1; i < static_cast<int>(pdDistK.size()); ++i) {
                const float height_delta = std::fabs(
                    kdtree_terrain_clould_->getInputCloud()->points[pIdxK[i]].intensity - p.z);
                if (pdDistK[i] < best_dist - 1e-6f ||
                    (std::fabs(pdDistK[i] - best_dist) <= 1e-6f &&
                     height_delta < best_height_delta)) {
                    best_dist = pdDistK[i];
                    best_height_delta = height_delta;
                    best_idx = i;
                }
            }
            dist_square = best_dist;
            pcl_p = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[best_idx]];
            return pcl_p.intensity;
        }
        return p.z;
    }

    inline void AssignFlatTerrainCloud(const PointCloudPtr& terrainRef, PointCloudPtr& terrainFlatOut) {
        const int N = terrainRef->size();
        terrainFlatOut->resize(N);
        for (int i = 0; i<N; i++) {
            PCLPoint pcl_p = terrainRef->points[i];
            pcl_p.intensity = pcl_p.z, pcl_p.z = 0.0f;
            terrainFlatOut->points[i] = pcl_p;
        }
    }
};

#endif
