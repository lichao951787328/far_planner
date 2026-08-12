#ifndef TERMINAL_VISIBILITY_POLICY_H
#define TERMINAL_VISIBILITY_POLICY_H

#include "far_planner/node_struct.h"

#include <algorithm>
#include <cstddef>

struct TerminalProjectionSearchResult {
    EdgeValidationResult validation;
    std::size_t evaluated_candidates = 0;
};

/**
 * Search an obstacle-corner projection from near to far and return the first
 * geometry that passes the caller's complete collision check.  Unlike
 * waypoint extension, a rejected near projection does not make a farther
 * projection unreachable: the near point may simply still lie inside the
 * robot-clearance band around the same obstacle corner.
 *
 * Surface direction is deliberately not part of the acceptance policy.  The
 * caller may use it to construct each candidate, but only the validator can
 * accept or reject the resulting route.
 */
template <typename Validator>
TerminalProjectionSearchResult FindNearestSafeTerminalProjection(
    const float first_distance, const float maximum_distance,
    const float distance_step, const bool can_project,
    Validator validator) {
    TerminalProjectionSearchResult result;

    if (!can_project) {
        result.evaluated_candidates = 1;
        result.validation = validator(0.0f);
        return result;
    }
    if (first_distance < 0.0f || maximum_distance < first_distance ||
        distance_step <= 0.0f) {
        result.validation.reason = EdgeRejectReason::OFFSET_FAILED;
        return result;
    }

    float distance = first_distance;
    while (true) {
        ++result.evaluated_candidates;
        result.validation = validator(distance);
        constexpr float kDistanceEpsilon = 1e-6f;
        if (result.validation.valid ||
            distance >= maximum_distance - kDistanceEpsilon) {
            return result;
        }
        distance = std::min(maximum_distance, distance + distance_step);
    }
}

#endif  // TERMINAL_VISIBILITY_POLICY_H
