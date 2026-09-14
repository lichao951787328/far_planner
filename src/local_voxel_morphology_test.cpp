#include "far_planner/local_voxel_morphology.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

TEST(LocalVoxelMorphology, OpeningRemovesIsolatedPixelAndKeepsSolidCore) {
    std::vector<cv::Point2f> points;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            points.emplace_back(x * 0.1f, y * 0.1f);
        }
    }
    const std::size_t duplicate_center = points.size();
    points.emplace_back(0.0f, 0.0f);  // another height sample in the same XY
    const std::size_t isolated = points.size();
    points.emplace_back(2.0f, 2.0f);

    const LocalVoxelMorphologyResult result =
        ComputeLocalVoxelMorphologyOpening(
            points, 0.1f, 0.1f, 0.1f, true);

    ASSERT_TRUE(result.applied);
    ASSERT_EQ(result.keep.size(), points.size());
    EXPECT_EQ(result.radius_cells, 1);
    EXPECT_EQ(result.keep[12], 1u);
    EXPECT_EQ(result.keep[duplicate_center], 1u);
    EXPECT_EQ(result.keep[isolated], 0u);
    EXPECT_GE(result.removed_input_pixels, 1u);
    EXPECT_GE(result.removed_points, 1u);
}

TEST(LocalVoxelMorphology, MetricRadiusUsesFineRasterResolution) {
    std::vector<cv::Point2f> block;
    for (int x = -4; x <= 4; ++x) {
        for (int y = -4; y <= 4; ++y) {
            block.emplace_back(x * 0.05f, y * 0.05f);
        }
    }
    const LocalVoxelMorphologyResult result =
        ComputeLocalVoxelMorphologyOpening(
            block, 0.05f, 0.01f, 0.10f, true);
    ASSERT_TRUE(result.applied);
    EXPECT_EQ(result.radius_cells, 10);
    EXPECT_EQ(result.keep[40], 1u);
}

TEST(LocalVoxelMorphology, FineRasterPreservesSourceVoxelFootprint) {
    const std::vector<cv::Point2f> point = {cv::Point2f(0.0f, 0.0f)};
    const LocalVoxelMorphologyResult fine_opening =
        ComputeLocalVoxelMorphologyOpening(
            point, 0.10f, 0.01f, 0.01f, true);
    ASSERT_TRUE(fine_opening.applied);
    EXPECT_EQ(fine_opening.radius_cells, 1);
    EXPECT_GT(fine_opening.input_pixels, 1u);
    EXPECT_EQ(fine_opening.keep[0], 1u);

    // A radius wider than half of an isolated 0.10 m source voxel removes it.
    const LocalVoxelMorphologyResult wider_opening =
        ComputeLocalVoxelMorphologyOpening(
            point, 0.10f, 0.01f, 0.06f, true);
    ASSERT_TRUE(wider_opening.applied);
    EXPECT_EQ(wider_opening.radius_cells, 6);
    EXPECT_EQ(wider_opening.keep[0], 0u);
}

TEST(LocalVoxelMorphology, DisabledAndInvalidExtentsFailOpen) {
    const std::vector<cv::Point2f> points = {
        cv::Point2f(0.0f, 0.0f), cv::Point2f(100.0f, 100.0f)};
    const LocalVoxelMorphologyResult disabled =
        ComputeLocalVoxelMorphologyOpening(
            points, 0.1f, 0.01f, 0.01f, false);
    EXPECT_FALSE(disabled.applied);
    EXPECT_EQ(disabled.keep,
              (std::vector<std::uint8_t>{1u, 1u}));

    const LocalVoxelMorphologyResult excessive_extent =
        ComputeLocalVoxelMorphologyOpening(
            points, 0.1f, 0.01f, 0.01f, true, 100u);
    EXPECT_FALSE(excessive_extent.applied);
    EXPECT_EQ(excessive_extent.keep,
              (std::vector<std::uint8_t>{1u, 1u}));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
