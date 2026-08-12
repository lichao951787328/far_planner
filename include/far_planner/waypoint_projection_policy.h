#ifndef WAYPOINT_PROJECTION_POLICY_H
#define WAYPOINT_PROJECTION_POLICY_H

#include <algorithm>
#include <cmath>
#include <cstddef>

struct ProgressiveProjectionResult {
    bool has_safe_projection = false;
    float distance = 0.0f;
    std::size_t evaluated_candidates = 0;
};

/**
 * Evaluate waypoint projections strictly from near to far.  The accepted
 * prefix must be continuous: the first rejected distance terminates the
 * search, even if a farther isolated point would happen to be collision-free.
 * This matches the motion semantics of extending a waypoint through free
 * space rather than sampling unrelated destinations.
 */
template <typename Validator>
ProgressiveProjectionResult FindFurthestConsecutiveSafeProjection(
    const float first_distance, const float maximum_distance,
    const float distance_step, Validator validator) {
    ProgressiveProjectionResult result;
    if (first_distance < 0.0f || maximum_distance < first_distance ||
        distance_step <= 0.0f) {
        return result;
    }

    float distance = first_distance;
    while (true) {
        ++result.evaluated_candidates;
        if (!validator(distance)) break;
        result.has_safe_projection = true;
        result.distance = distance;
        if (distance >= maximum_distance) break;
        distance = std::min(maximum_distance, distance + distance_step);
    }
    return result;
}

#endif  // WAYPOINT_PROJECTION_POLICY_H
