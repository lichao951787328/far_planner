#include <gtest/gtest.h>

#include <vector>

#include "far_planner/goal_adjustment_policy.h"

TEST(GoalAdjustmentPolicy, StopsAtFirstValidCandidate) {
    const std::vector<int> candidates = {1, 2, 3, 4, 5};
    std::vector<int> visited;
    const BoundedGoalCandidateSearchResult result =
        FindFirstValidGoalAdjustmentCandidate(
            candidates, 5,
            [&visited](const int candidate) {
                visited.push_back(candidate);
                return candidate == 3;
            });

    EXPECT_TRUE(result.found);
    EXPECT_EQ(3u, result.evaluated);
    EXPECT_EQ((std::vector<int>{1, 2, 3}), visited);
}

TEST(GoalAdjustmentPolicy, NeverExceedsConfiguredCandidateBudget) {
    const std::vector<int> candidates = {1, 2, 3, 4, 5};
    std::vector<int> visited;
    const BoundedGoalCandidateSearchResult result =
        FindFirstValidGoalAdjustmentCandidate(
            candidates, 2,
            [&visited](const int candidate) {
                visited.push_back(candidate);
                return candidate == 3;
            });

    EXPECT_FALSE(result.found);
    EXPECT_EQ(2u, result.evaluated);
    EXPECT_EQ((std::vector<int>{1, 2}), visited);
}

TEST(GoalAdjustmentPolicy, ZeroBudgetPerformsNoValidation) {
    const std::vector<int> candidates = {1};
    bool called = false;
    const BoundedGoalCandidateSearchResult result =
        FindFirstValidGoalAdjustmentCandidate(
            candidates, 0,
            [&called](const int) {
                called = true;
                return true;
            });

    EXPECT_FALSE(result.found);
    EXPECT_EQ(0u, result.evaluated);
    EXPECT_FALSE(called);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
