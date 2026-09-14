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
    // Retained for schema/backward compatibility.  Non-dynamic high-cost
    // voxels no longer need to belong to this set to be ordinary obstacles.
    std::vector<uint32_t> static_labels{2, 3, 4, 5, 6, 7, 8};
    std::vector<uint32_t> terrain_labels{0, 1, 9};
    std::vector<uint32_t> dynamic_labels{11, 12, 13, 14, 15, 16, 17, 18};
    // Retained as an input-contract setting and diagnostic value. Confidence
    // no longer demotes high-cost geometry into the transient layer.
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
 * Only an explicitly dynamic semantic label enters the transient layer.
 * Every other voxel must pass the final fused-cost gate before it can be an
 * obstacle; once it does, it is treated as ordinary/static geometry even when
 * its label is unknown or its semantic confidence is low.  This deliberately
 * makes geometry conservative when the upstream model cannot distinguish a
 * person or vegetation: motion is never inferred merely from uncertainty.
 * Low-cost terrain labels provide height support and all other low-cost or
 * unscored voxels are ignored.
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

    if (!high_cost) {
        if (has_semantic_label &&
            LocalVoxelLabelInSet(label, params.terrain_labels)) {
            return LocalVoxelLayer::TERRAIN_SUPPORT;
        }
        return LocalVoxelLayer::IGNORE;
    }

    // label/confidence may help diagnostics, but they do not turn an
    // unrecognised high-cost obstacle into a dynamic obstacle.  The temporal
    // graph lifecycle later decides whether this ordinary obstacle is stable
    // enough to retain as historical topology.
    (void)confidence;
    return LocalVoxelLayer::STATIC_OBSTACLE;
}

#endif  // FAR_PLANNER_LOCAL_VOXEL_POLICY_H
