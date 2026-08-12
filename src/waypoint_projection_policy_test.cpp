#include <gtest/gtest.h>

#include <vector>

#include "far_planner/waypoint_projection_policy.h"

TEST(WaypointProjectionPolicy, EvaluatesNearToFarAndKeepsLastSafePoint) {
    std::vector<float> evaluated;
    const ProgressiveProjectionResult result =
        FindFurthestConsecutiveSafeProjection(
            0.6f, 5.0f, 0.2f,
            [&evaluated](const float distance) {
                evaluated.push_back(distance);
                return distance < 1.2f - 1e-4f;
            });

    ASSERT_TRUE(result.has_safe_projection);
    EXPECT_NEAR(1.0f, result.distance, 1e-4f);
    ASSERT_EQ(4u, evaluated.size());
    EXPECT_NEAR(0.6f, evaluated[0], 1e-4f);
    EXPECT_NEAR(0.8f, evaluated[1], 1e-4f);
    EXPECT_NEAR(1.0f, evaluated[2], 1e-4f);
    EXPECT_NEAR(1.2f, evaluated[3], 1e-4f);
}

TEST(WaypointProjectionPolicy, DoesNotSkipARejectedGap) {
    std::vector<float> evaluated;
    const ProgressiveProjectionResult result =
        FindFurthestConsecutiveSafeProjection(
            0.6f, 2.0f, 0.2f,
            [&evaluated](const float distance) {
                evaluated.push_back(distance);
                // A farther point would be safe, but the free-space extension
                // must stop at the first blocked interval.
                return distance < 1.0f - 1e-4f || distance > 1.4f;
            });

    ASSERT_TRUE(result.has_safe_projection);
    EXPECT_NEAR(0.8f, result.distance, 1e-4f);
    ASSERT_EQ(3u, evaluated.size());
    EXPECT_NEAR(1.0f, evaluated.back(), 1e-4f);
}

TEST(WaypointProjectionPolicy, FirstRejectedCandidateUsesCallerFallback) {
    const ProgressiveProjectionResult result =
        FindFurthestConsecutiveSafeProjection(
            0.6f, 5.0f, 0.2f,
            [](const float) { return false; });
    EXPECT_FALSE(result.has_safe_projection);
    EXPECT_FLOAT_EQ(0.0f, result.distance);
    EXPECT_EQ(1u, result.evaluated_candidates);
}

TEST(WaypointProjectionPolicy, EvaluatesMaximumDistanceExactly) {
    const ProgressiveProjectionResult result =
        FindFurthestConsecutiveSafeProjection(
            0.6f, 1.05f, 0.2f,
            [](const float) { return true; });
    ASSERT_TRUE(result.has_safe_projection);
    EXPECT_NEAR(1.05f, result.distance, 1e-4f);
    EXPECT_EQ(4u, result.evaluated_candidates);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
