#ifndef GOAL_ADJUSTMENT_POLICY_H
#define GOAL_ADJUSTMENT_POLICY_H

#include <algorithm>
#include <cstddef>
#include <vector>

struct BoundedGoalCandidateSearchResult {
    bool found = false;
    std::size_t evaluated = 0;
};

/** Evaluate candidates in caller-defined priority order and stop on either
 * the first accepted candidate or the configured complete-check budget. */
template <typename Candidate, typename Validator>
BoundedGoalCandidateSearchResult FindFirstValidGoalAdjustmentCandidate(
    const std::vector<Candidate>& candidates,
    const std::size_t maximum_evaluations,
    Validator&& validator) {
    BoundedGoalCandidateSearchResult result;
    for (const Candidate& candidate : candidates) {
        if (result.evaluated >= maximum_evaluations) break;
        ++result.evaluated;
        if (!validator(candidate)) continue;
        result.found = true;
        break;
    }
    return result;
}

#endif
