#include "far_planner/semantic_confidence.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace {

using octomap::ColorOcTreeNode;
using octomap::ColorWithLogOdds;
using octomap::SemanticsLogOdds;

SemanticsLogOdds MakeDistribution(const float top1,
                                  const float second,
                                  const float third,
                                  const float others) {
    SemanticsLogOdds semantics;
    semantics.data[0] = ColorWithLogOdds(
        ColorOcTreeNode::Color(10, 20, 30), std::log(top1));
    semantics.data[1] = ColorWithLogOdds(
        ColorOcTreeNode::Color(40, 50, 60), std::log(second));
    semantics.data[2] = ColorWithLogOdds(
        ColorOcTreeNode::Color(70, 80, 90), std::log(third));
    semantics.others = std::log(others);
    return semantics;
}

TEST(SemanticConfidencePolicy, NormalizesTopThreeAndAggregateOthers) {
    const SemanticsLogOdds semantics =
        MakeDistribution(0.55f, 0.20f, 0.10f, 0.15f);
    EXPECT_NEAR(SemanticConfidence::Top1Probability(semantics), 0.55f,
                1e-6f);
}

TEST(SemanticConfidencePolicy, ThresholdIsInclusive) {
    const SemanticsLogOdds semantics =
        MakeDistribution(0.60f, 0.20f, 0.10f, 0.10f);
    const float probability = SemanticConfidence::Top1Probability(semantics);
    EXPECT_TRUE(SemanticConfidence::AcceptTop1(semantics, probability));
    EXPECT_TRUE(SemanticConfidence::AcceptTop1(semantics, 0.59f));
    EXPECT_FALSE(SemanticConfidence::AcceptTop1(semantics, 0.61f));
}

TEST(SemanticConfidencePolicy, MissingOrInvalidTop1HasZeroConfidence) {
    SemanticsLogOdds missing;
    EXPECT_FLOAT_EQ(SemanticConfidence::Top1Probability(missing), 0.0f);

    SemanticsLogOdds invalid =
        MakeDistribution(0.60f, 0.20f, 0.10f, 0.10f);
    invalid.data[0].logOdds = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FLOAT_EQ(SemanticConfidence::Top1Probability(invalid), 0.0f);
}

TEST(SemanticConfidencePolicy, LogSumExpRemainsStableForLargeWeights) {
    SemanticsLogOdds semantics;
    semantics.data[0] = ColorWithLogOdds(
        ColorOcTreeNode::Color(10, 20, 30), 1000.0f);
    semantics.data[1] = ColorWithLogOdds(
        ColorOcTreeNode::Color(40, 50, 60), 999.0f);
    semantics.data[2] = ColorWithLogOdds(
        ColorOcTreeNode::Color(70, 80, 90), 998.0f);
    semantics.others = 997.0f;

    const double expected =
        1.0 / (1.0 + std::exp(-1.0) + std::exp(-2.0) + std::exp(-3.0));
    EXPECT_NEAR(SemanticConfidence::Top1Probability(semantics), expected,
                1e-6);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
