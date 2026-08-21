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
    std::vector<uint32_t> static_labels;
    std::vector<uint32_t> terrain_labels;
    std::vector<uint32_t> dynamic_labels;
    std::vector<uint32_t> static_rgb_keys;
    std::vector<uint32_t> terrain_rgb_keys;
    std::vector<uint32_t> dynamic_rgb_keys;
    float minimum_semantic_confidence = 0.55f;
    float obstacle_cost_threshold = 0.60f;
};

inline bool LocalVoxelLabelInSet(
    const uint32_t label, const std::vector<uint32_t>& labels) {
    return std::find(labels.begin(), labels.end(), label) != labels.end();
}

enum class LocalVoxelSemanticRole {
    UNKNOWN = 0,
    TERRAIN,
    STATIC,
    DYNAMIC
};

inline LocalVoxelSemanticRole MatchLocalVoxelSemanticRole(
    const uint32_t label, const bool has_semantic_label,
    const uint32_t rgb_key, const bool has_rgb,
    const LocalVoxelPolicyParams& params) {
    // The upstream local-map implementations always publish a label and an
    // RGB field, but the meaning of label differs by profile: Gazebo uses a
    // packed semantic RGB while the recorded-data pipeline uses integer class
    // IDs. A dynamic result from either representation is a conservative
    // veto: inconsistent metadata must never promote a moving object into the
    // permanent graph. For non-dynamic roles prefer the configured label and
    // use RGB as the portable fallback. No environment-specific branch is
    // needed in the planner.
    if (has_semantic_label) {
        if (LocalVoxelLabelInSet(label, params.dynamic_labels)) {
            return LocalVoxelSemanticRole::DYNAMIC;
        }
    }
    if (has_rgb &&
        LocalVoxelLabelInSet(rgb_key, params.dynamic_rgb_keys)) {
        return LocalVoxelSemanticRole::DYNAMIC;
    }
    if (has_semantic_label) {
        if (LocalVoxelLabelInSet(label, params.static_labels)) {
            return LocalVoxelSemanticRole::STATIC;
        }
        if (LocalVoxelLabelInSet(label, params.terrain_labels)) {
            return LocalVoxelSemanticRole::TERRAIN;
        }
    }
    if (has_rgb) {
        if (LocalVoxelLabelInSet(rgb_key, params.static_rgb_keys)) {
            return LocalVoxelSemanticRole::STATIC;
        }
        if (LocalVoxelLabelInSet(rgb_key, params.terrain_rgb_keys)) {
            return LocalVoxelSemanticRole::TERRAIN;
        }
    }
    return LocalVoxelSemanticRole::UNKNOWN;
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
    const uint32_t rgb_key, const bool has_rgb,
    const float semantic_confidence, const bool has_traversability,
    const float traversability, const LocalVoxelPolicyParams& params) {
    const float confidence = std::isfinite(semantic_confidence)
        ? std::max(0.0f, std::min(1.0f, semantic_confidence))
        : 0.0f;
    const bool high_cost = has_traversability && std::isfinite(traversability) &&
        traversability >= params.obstacle_cost_threshold;

    const LocalVoxelSemanticRole role = MatchLocalVoxelSemanticRole(
        label, has_semantic_label, rgb_key, has_rgb, params);
    if (role == LocalVoxelSemanticRole::DYNAMIC) {
        return LocalVoxelLayer::TRANSIENT_OBSTACLE;
    }
    if (role == LocalVoxelSemanticRole::STATIC) {
        return confidence >= params.minimum_semantic_confidence
            ? LocalVoxelLayer::STATIC_OBSTACLE
            : (high_cost ? LocalVoxelLayer::TRANSIENT_OBSTACLE
                         : LocalVoxelLayer::IGNORE);
    }
    if (role == LocalVoxelSemanticRole::TERRAIN) {
        return high_cost ? LocalVoxelLayer::TRANSIENT_OBSTACLE
                         : LocalVoxelLayer::TERRAIN_SUPPORT;
    }
    return high_cost ? LocalVoxelLayer::TRANSIENT_OBSTACLE
                     : LocalVoxelLayer::IGNORE;
}

// Compatibility overload for policy-only callers that provide no RGB field.
inline LocalVoxelLayer ClassifyLocalVoxel(
    const uint32_t label, const bool has_semantic_label,
    const float semantic_confidence, const bool has_traversability,
    const float traversability, const LocalVoxelPolicyParams& params) {
    return ClassifyLocalVoxel(
        label, has_semantic_label, 0u, false, semantic_confidence,
        has_traversability, traversability, params);
}

#endif  // FAR_PLANNER_LOCAL_VOXEL_POLICY_H
