#include <gtest/gtest.h>

#include "far_planner/local_voxel_policy.h"

namespace {

LocalVoxelPolicyParams RecordedDataPolicy() {
    LocalVoxelPolicyParams params;
    params.static_labels = {2, 3, 4, 5, 6, 7, 8};
    params.terrain_labels = {0, 1, 9};
    params.dynamic_labels = {11, 12, 13, 14, 15, 16, 17, 18};
    return params;
}

LocalVoxelPolicyParams GazeboColorPolicy() {
    LocalVoxelPolicyParams params;
    params.static_rgb_keys = {0x0021c1u, 0x808080u};
    params.terrain_rgb_keys = {0x000000u, 0xffa500u};
    params.dynamic_rgb_keys = {0xff00ffu};
    return params;
}

}  // namespace

TEST(LocalVoxelPolicy, DynamicSemanticIsAlwaysTransient) {
    const LocalVoxelPolicyParams params = RecordedDataPolicy();
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(11, true, 1.0f, true, 0.1f, params));
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(18, true, 0.0f, false, 0.0f, params));
}

TEST(LocalVoxelPolicy, ConfidentStaticSemanticCreatesStaticCandidateGeometry) {
    const LocalVoxelPolicyParams params = RecordedDataPolicy();
    EXPECT_EQ(LocalVoxelLayer::STATIC_OBSTACLE,
              ClassifyLocalVoxel(3, true, 0.8f, true, 1.0f, params));
}

TEST(LocalVoxelPolicy, LowConfidenceStaticStaysTransientWhenDangerous) {
    const LocalVoxelPolicyParams params = RecordedDataPolicy();
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(3, true, 0.2f, true, 0.9f, params));
    EXPECT_EQ(LocalVoxelLayer::IGNORE,
              ClassifyLocalVoxel(3, true, 0.2f, true, 0.1f, params));
}

TEST(LocalVoxelPolicy, HighCostTerrainOrUnknownCannotBecomeStatic) {
    const LocalVoxelPolicyParams params = RecordedDataPolicy();
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(0, true, 1.0f, true, 0.8f, params));
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(999, true, 1.0f, true, 0.8f, params));
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(0, false, 0.0f, true, 0.8f, params));
}

TEST(LocalVoxelPolicy, LowCostTerrainProvidesHeightOnly) {
    const LocalVoxelPolicyParams params = RecordedDataPolicy();
    EXPECT_EQ(LocalVoxelLayer::TERRAIN_SUPPORT,
              ClassifyLocalVoxel(9, true, 0.1f, true, 0.2f, params));
}

TEST(LocalVoxelPolicy, GazeboPackedRgbClassifiesWithoutIntegerClassIds) {
    const LocalVoxelPolicyParams params = GazeboColorPolicy();
    EXPECT_EQ(LocalVoxelLayer::STATIC_OBSTACLE,
              ClassifyLocalVoxel(999, true, 0x0021c1u, true,
                                 0.9f, true, 1.0f, params));
    EXPECT_EQ(LocalVoxelLayer::TERRAIN_SUPPORT,
              ClassifyLocalVoxel(999, true, 0xffa500u, true,
                                 0.9f, true, 0.1f, params));
}

TEST(LocalVoxelPolicy, DynamicRgbVetoesContradictoryStaticLabel) {
    LocalVoxelPolicyParams params = RecordedDataPolicy();
    params.dynamic_rgb_keys = {0xff00ffu};
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(3, true, 0xff00ffu, true,
                                 1.0f, true, 1.0f, params));
}

TEST(LocalVoxelPolicy, UnknownHighCostPointIsSafetyOnly) {
    const LocalVoxelPolicyParams params = GazeboColorPolicy();
    EXPECT_EQ(LocalVoxelLayer::TRANSIENT_OBSTACLE,
              ClassifyLocalVoxel(999, true, 0x123456u, true,
                                 1.0f, true, 0.9f, params));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
