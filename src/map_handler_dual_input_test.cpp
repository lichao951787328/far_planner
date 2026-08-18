#include <gtest/gtest.h>
#include <ros/ros.h>

#include <initializer_list>
#include <limits>
#include <memory>

#include "far_planner/map_handler.h"

namespace {

PCLPoint MakePoint(const float x, const float y, const float z,
                   const float intensity = 0.0f) {
    PCLPoint point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
}

PointCloudPtr MakeCloud(std::initializer_list<PCLPoint> points) {
    PointCloudPtr cloud(new PointCloud());
    cloud->points.assign(points.begin(), points.end());
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

std::unique_ptr<MapHandler> MakeDualInputHandler() {
    FARUtil::kLeafSize = 0.20f;
    FARUtil::vehicle_height = 0.50f;
    FARUtil::kCellHeight = 1.20f;
    FARUtil::kTolerZ = 2.60f;

    MapHandlerParams params;
    params.sensor_range = 5.0f;
    params.semantic_params.use_local_voxel_map = true;
    params.semantic_params.local_voxel_resolution = 0.10f;
    params.semantic_params.local_window_radius = 5.0f;
    params.semantic_params.local_planner_radius = 5.0f;
    params.semantic_params.local_planner_resolution = 0.20f;
    params.semantic_params.terrain_search_radius = 0.80f;
    params.semantic_params.terrain_neighbor_radius = 1.0f;

    std::unique_ptr<MapHandler> handler(new MapHandler());
    handler->Init(params);
    handler->SetMapOrigin(Point3D(0.0f, 0.0f, 0.0f));
    return handler;
}

TEST(MapHandlerDualInput, LocalStaticNeverEntersPersistentGlobalLayer) {
    std::unique_ptr<MapHandler> handler = MakeDualInputHandler();
    const PointCloudPtr local_static = MakeCloud({
        MakePoint(1.0f, 0.0f, 0.0f),
        MakePoint(1.1f, 0.0f, 0.0f)});
    const PointCloudPtr transient = MakeCloud({
        MakePoint(0.0f, 1.0f, 0.0f)});
    const PointCloudPtr terrain = MakeCloud({
        MakePoint(0.0f, 0.0f, -0.2f)});

    handler->SetLocalVoxelSnapshot(local_static, transient, terrain);

    PointCloudPtr output(new PointCloud());
    handler->GetCurrentStaticObsCloud(output);
    EXPECT_EQ(2u, output->size());
    handler->GetEffectiveDynamicObsCloud(output);
    EXPECT_EQ(1u, output->size());
    handler->GetSurroundObsCloud(output);
    EXPECT_EQ(3u, output->size());
    handler->GetPersistentStaticObsCloud(output);
    EXPECT_TRUE(output->empty());
    EXPECT_EQ(StaticNodeEvidence::UNKNOWN,
              handler->QueryStaticNodeEvidence(Point3D(1.0f, 0.0f, 0.0f)));

    handler->GetChangedObsCloud(output);
    EXPECT_EQ(3u, output->size());
    handler->GetDynamicAddedCloud(output);
    EXPECT_EQ(1u, output->size());
    handler->GetDynamicRemovedCloud(output);
    EXPECT_TRUE(output->empty());

    bool terrain_matched = false;
    EXPECT_NEAR(-0.2f,
                MapHandler::TerrainHeightOfPoint(
                    Point3D(0.0f, 0.0f, 0.0f), terrain_matched, false),
                1e-5f);
    EXPECT_TRUE(terrain_matched);
}

TEST(MapHandlerDualInput, AtomicReplacementReportsStaticAndTransientRemoval) {
    std::unique_ptr<MapHandler> handler = MakeDualInputHandler();
    handler->SetLocalVoxelSnapshot(
        MakeCloud({MakePoint(1.0f, 0.0f, 0.0f),
                   MakePoint(1.1f, 0.0f, 0.0f)}),
        MakeCloud({MakePoint(0.0f, 1.0f, 0.0f)}),
        MakeCloud({MakePoint(0.0f, 0.0f, -0.2f)}));

    handler->SetLocalVoxelSnapshot(
        MakeCloud({}), MakeCloud({}), MakeCloud({}));

    PointCloudPtr output(new PointCloud());
    handler->GetSurroundObsCloud(output);
    EXPECT_TRUE(output->empty());
    handler->GetChangedObsCloud(output);
    EXPECT_EQ(3u, output->size());
    handler->GetDynamicRemovedCloud(output);
    EXPECT_EQ(1u, output->size());
    handler->GetPersistentStaticObsCloud(output);
    EXPECT_TRUE(output->empty());
}

}  // namespace

// Production owns these static values in far_planner.cpp. This standalone
// target follows map_handler_test.cpp and supplies isolated test storage.
PointCloudPtr FARUtil::surround_obs_cloud_(new PointCloud());
PointCloudPtr FARUtil::stack_new_cloud_(new PointCloud());
PointCloudPtr FARUtil::stack_dyobs_cloud_(new PointCloud());
PointCloudPtr FARUtil::cur_new_cloud_(new PointCloud());
PointCloudPtr FARUtil::cur_dyobs_cloud_(new PointCloud());
PointCloudPtr FARUtil::cur_scan_cloud_(new PointCloud());
PointKdTreePtr FARUtil::kdtree_new_cloud_(
    new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_(
    new pcl::KdTreeFLANN<PCLPoint>());
const float FARUtil::kEpsilon = 1e-7f;
const float FARUtil::kINF = std::numeric_limits<float>::max();
bool FARUtil::IsStaticEnv = true;
bool FARUtil::IsDebug = true;
bool FARUtil::IsMultiLayer = false;
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float FARUtil::robot_dim = 0.8f;
float FARUtil::kAngleNoise = 0.0f;
float FARUtil::kCellLength = 0.5f;
float FARUtil::kCellHeight = 0.5f;
float FARUtil::vehicle_height = 0.5f;
float FARUtil::kLeafSize = 0.5f;
float FARUtil::kHeightVoxel = 1.0f;
float FARUtil::kNavClearDist = 0.9f;
float FARUtil::kNearDist = 0.8f;
float FARUtil::kMatchDist = 1.0f;
float FARUtil::kProjectDist = 0.5f;
float FARUtil::kNewPIThred = 2.0f;
float FARUtil::kSensorRange = 15.0f;
float FARUtil::kMarginDist = 14.0f;
float FARUtil::kLocalPlanRange = 15.0f;
float FARUtil::kMarginHeight = 1.0f;
float FARUtil::kTerrainRange = 15.0f;
float FARUtil::kFreeZ = 0.1f;
float FARUtil::kObsDecayTime = 10.0f;
float FARUtil::kNewDecayTime = 2.0f;
int FARUtil::kDyObsThred = 4;
int FARUtil::KNewPointC = 10;
int FARUtil::kObsInflate = 2;
float FARUtil::kTolerZ = 2.0f;
float FARUtil::kAcceptAlign = 0.0f;
float FARUtil::kVizRatio = 1.0f;
double FARUtil::systemStartTime = 0.0;
TimeMeasure FARUtil::Timer;
std::string FARUtil::worldFrameId = "map";
PointKdTreePtr MapHandler::kdtree_terrain_clould_;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_handler_dual_input_test",
              ros::init_options::AnonymousName);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
