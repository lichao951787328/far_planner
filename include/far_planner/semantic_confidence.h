#ifndef FAR_PLANNER_SEMANTIC_CONFIDENCE_H
#define FAR_PLANNER_SEMANTIC_CONFIDENCE_H

#include <algorithm>
#include <cmath>

#include <octomap/ColorOcTree.h>
#include <semantic_octree/Semantics.h>

namespace SemanticConfidence {

/**
 * Return the normalized probability mass of SemanticsLogOdds::data[0].
 *
 * SemanticsLogOdds stores up to three named classes plus one aggregate
 * `others` log-weight.  Log-sum-exp normalization keeps the result stable
 * after repeated semantic fusion.  A missing or non-finite top-1 entry has
 * zero confidence.
 */
inline float Top1Probability(const octomap::SemanticsLogOdds& semantics) {
    const octomap::ColorOcTreeNode::Color unset_color(255, 255, 255);
    if (semantics.data[0].color == unset_color ||
        !std::isfinite(semantics.data[0].logOdds)) {
        return 0.0f;
    }

    double max_log_weight = semantics.data[0].logOdds;
    if (std::isfinite(semantics.others)) {
        max_log_weight = std::max(
            max_log_weight, static_cast<double>(semantics.others));
    }
    for (int i = 1; i < NUM_SEMANTICS; ++i) {
        if (semantics.data[i].color != unset_color &&
            std::isfinite(semantics.data[i].logOdds)) {
            max_log_weight = std::max(
                max_log_weight,
                static_cast<double>(semantics.data[i].logOdds));
        }
    }

    double normalizer = 0.0;
    if (std::isfinite(semantics.others)) {
        normalizer += std::exp(
            static_cast<double>(semantics.others) - max_log_weight);
    }
    for (int i = 0; i < NUM_SEMANTICS; ++i) {
        if (semantics.data[i].color != unset_color &&
            std::isfinite(semantics.data[i].logOdds)) {
            normalizer += std::exp(
                static_cast<double>(semantics.data[i].logOdds) -
                max_log_weight);
        }
    }
    if (!(normalizer > 0.0) || !std::isfinite(normalizer)) return 0.0f;

    const double top1_weight = std::exp(
        static_cast<double>(semantics.data[0].logOdds) - max_log_weight);
    return static_cast<float>(top1_weight / normalizer);
}

inline bool AcceptTop1(const octomap::SemanticsLogOdds& semantics,
                       const float min_probability) {
    return Top1Probability(semantics) >= min_probability;
}

}  // namespace SemanticConfidence

#endif  // FAR_PLANNER_SEMANTIC_CONFIDENCE_H

