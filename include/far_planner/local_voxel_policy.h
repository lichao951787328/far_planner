#ifndef FAR_PLANNER_LOCAL_VOXEL_POLICY_H
#define FAR_PLANNER_LOCAL_VOXEL_POLICY_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

enum class LocalVoxelLayer {
    IGNORE = 0,
    TERRAIN_SUPPORT,
    STATIC_OBSTACLE,
    TRANSIENT_OBSTACLE
};

struct LocalVoxelPolicyParams {
    std::vector<uint32_t> static_labels{2, 3, 4, 5, 6, 7, 8};
    std::vector<uint32_t> terrain_labels{0, 1, 9};
    std::vector<uint32_t> dynamic_labels{11, 12, 13, 14, 15, 16, 17, 18};
    float minimum_semantic_confidence = 0.55f;
    float obstacle_cost_threshold = 0.60f;
};

inline bool LocalVoxelLabelInSet(
    const uint32_t label, const std::vector<uint32_t>& labels) {
    return std::find(labels.begin(), labels.end(), label) != labels.end();
}

/**
 * Classify one voxel from the atomic high-resolution local snapshot.
 *
 * Dynamic semantics are an unconditional veto on static persistence. A
 * low-confidence or unknown high-cost voxel remains a current safety obstacle
 * but is deliberately assigned to the transient layer, so it cannot create a
 * permanent static Graph node merely through a traversability fallback.
 */
inline LocalVoxelLayer ClassifyLocalVoxel(
    const uint32_t label, const bool has_semantic_label,
    const float semantic_confidence, const bool has_traversability,
    const float traversability, const LocalVoxelPolicyParams& params) {
    const float confidence = std::isfinite(semantic_confidence)
        ? std::max(0.0f, std::min(1.0f, semantic_confidence))
        : 0.0f;
    const bool high_cost = has_traversability && std::isfinite(traversability) &&
        traversability >= params.obstacle_cost_threshold;

    if (has_semantic_label &&
        LocalVoxelLabelInSet(label, params.dynamic_labels)) {
        return LocalVoxelLayer::TRANSIENT_OBSTACLE;
    }
    if (has_semantic_label &&
        LocalVoxelLabelInSet(label, params.static_labels)) {
        return confidence >= params.minimum_semantic_confidence
            ? LocalVoxelLayer::STATIC_OBSTACLE
            : (high_cost ? LocalVoxelLayer::TRANSIENT_OBSTACLE
                         : LocalVoxelLayer::IGNORE);
    }
    if (has_semantic_label &&
        LocalVoxelLabelInSet(label, params.terrain_labels)) {
        return high_cost ? LocalVoxelLayer::TRANSIENT_OBSTACLE
                         : LocalVoxelLayer::TERRAIN_SUPPORT;
    }
    return high_cost ? LocalVoxelLayer::TRANSIENT_OBSTACLE
                     : LocalVoxelLayer::IGNORE;
}

#endif  // FAR_PLANNER_LOCAL_VOXEL_POLICY_H
