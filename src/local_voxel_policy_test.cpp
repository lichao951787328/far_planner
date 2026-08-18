#include <gtest/gtest.h>

#include "far_planner/local_voxel_policy.h"

TEST(LocalVoxelPolicy, DynamicSemanticIsAlwaysTransient) {
    LocalVoxelPolicyParams params;
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(11, true, 1.0f, true, 0.1f, params));
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(18, true, 0.0f, false, 0.0f, params));
}

TEST(LocalVoxelPolicy, ConfidentStaticSemanticCreatesStaticCandidateGeometry) {
    LocalVoxelPolicyParams params;
    EXPECT_EQ(LocalVoxelLayer::STATIC_OBSTACLE,
              ClassifyLocalVoxel(3, true, 0.8f, true, 1.0f, params));
}

TEST(LocalVoxelPolicy, LowConfidenceStaticStaysTransientWhenDangerous) {
    LocalVoxelPolicyParams params;
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(3, true, 0.2f, true, 0.9f, params));
    EXPECT_EQ(LocalVoxelLayer::IGNORE,
              ClassifyLocalVoxel(3, true, 0.2f, true, 0.1f, params));
}

TEST(LocalVoxelPolicy, HighCostTerrainOrUnknownCannotBecomeStatic) {
    LocalVoxelPolicyParams params;
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(0, true, 1.0f, true, 0.8f, params));
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(999, true, 1.0f, true, 0.8f, params));
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(0, false, 0.0f, true, 0.8f, params));
}

TEST(LocalVoxelPolicy, LowCostTerrainProvidesHeightOnly) {
    LocalVoxelPolicyParams params;
    EXPECT_EQ(LocalVoxelLayer::TERRAIN_SUPPORT,
              ClassifyLocalVoxel(9, true, 0.1f, true, 0.2f, params));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
