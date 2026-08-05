#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"
#include <octomap/octomap.h>
#include <cstdint>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <unordered_set>

namespace octomap_msgs {
struct Octomap;
using OctomapConstPtr = boost::shared_ptr<const Octomap>;
}  // namespace octomap_msgs
enum CloudType {
    FREE_CLOUD = 0,
    OBS_CLOUD  = 1
};

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float floor_height;
};

struct SemanticClassGroup {
    SemanticClassGroup() = default;
    std::string name;
    uint32_t rgb_key = 0;
};

struct SemanticMapParams {
    SemanticMapParams() = default;
    float local_window_radius = 0.0f;
    float terrain_search_radius = 0.0f;
    bool  use_top1_only = true;
    float min_semantic_prob = 0.55f;
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
    MapHandler() = default;
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    // TODO: semantic octomap 先做原始消息缓存，后续再实现局部语义提取。
    void SetSemanticOctomap(const octomap_msgs::OctomapConstPtr& msg);
    bool HasSemanticMap() const { return has_semantic_map_; }
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

    /** Compatibility APIs retained for downstream stability; no-op in semantic-only mode. */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);

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
    void RebuildDerivedOctomapCachesFromSemanticTree(const octomap::AbstractOcTree& tree);
    void RebuildPointCloudCachesFromDerivedOctomaps();
    void CopyOccupancyTree(const octomap::AbstractOcTree& source_tree,
                           std::shared_ptr<octomap::OcTree>& target_tree) const;

    // 外部 semantic octomap 的派生缓存入口。
    // TODO: 当前先用点云缓存承接语义地图派生结果，后续可替换为更直接的 semantic octree 快照对象。
    // octomap::OcTree* octree_ = nullptr;
    octomap_msgs::OctomapConstPtr semantic_map_msg_;
    ros::Time semantic_stamp_;
    std::string semantic_frame_id_;
    SemanticMapParams semantic_params_;
    std::vector<SemanticClassGroup> obstacle_groups_;
    std::vector<SemanticClassGroup> terrain_support_groups_;
    std::shared_ptr<octomap::OcTree> semantic_obs_octree_;
    std::shared_ptr<octomap::OcTree> semantic_terrain_support_octree_;
    PointCloudPtr semantic_obs_cloud_;
    PointCloudPtr semantic_terrain_support_cloud_;
    bool has_semantic_map_ = false;


    MapHandlerParams map_params_;
    Point3D robot_pos_cache_;
    bool is_init_ = false;
    PointCloudPtr flat_terrain_cloud_;
    static PointKdTreePtr kdtree_terrain_clould_;

    template <typename Position>
    static inline float NearestHeightOfPoint(const Position& p, float& dist_square) {
        std::vector<int> pIdxK;
        std::vector<float> pdDistK;
        PCLPoint pcl_p;
        dist_square = FARUtil::kINF;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        const float radius = std::max(FARUtil::kLeafSize, 1e-3f);
        if (kdtree_terrain_clould_->radiusSearch(pcl_p, radius, pIdxK, pdDistK) > 0) {
            int best_idx = 0;
            float best_dist = pdDistK[0];
            for (int i = 1; i < static_cast<int>(pdDistK.size()); ++i) {
                if (pdDistK[i] < best_dist) {
                    best_dist = pdDistK[i];
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